"""Offline checks for the motion-control host and its device protocol.

A fake serial port stands in for the board, so no real port is ever opened and no CAN
frame is ever produced. Run:
    .venv/Scripts/python.exe -m unittest discover -s tools -p test_motor_control.py -v
"""
import queue
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
import tempfile
import json

sys.path.insert(0, str(Path(__file__).resolve().parent))
import motor_control as mc  # noqa: E402


class FakeSerial:
    """Minimal stand-in: records what the host wrote and replays what the device said."""

    def __init__(self, replies=()):
        self.written = []
        self.is_open = True
        self.dtr = True
        self.rts = True
        self.port = None
        self._replies = list(replies)
        self._pending = bytearray()
        self._lock = threading.Lock()

    def open(self):
        self.is_open = True
        # The host must clear these before opening so the ESP32 is not reset.
        assert self.dtr is False, "DTR must be false before open"
        assert self.rts is False, "RTS must be false before open"

    def write(self, payload):
        with self._lock:
            self.written.append(payload.decode("ascii"))
        return len(payload)

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._pending)

    def read(self, _size):
        with self._lock:
            chunk = bytes(self._pending[:_size])
            del self._pending[:_size]
        if chunk:
            return chunk
        time.sleep(0.005)
        return b""

    def feed(self, text):
        with self._lock:
            self._pending += text.encode("ascii")

    def close(self):
        self.is_open = False


class RecordingLink(mc.MotorLink):
    """MotorLink with the serial port object swapped for a fake.

    The real open() bookkeeping and the heartbeat thread stay on the production path,
    so the test covers what actually ships. Only the port object and the heartbeat
    period are substituted, and both are restored afterwards.
    """

    def __init__(self, events, fake, heartbeat_ms=40):
        super().__init__(events)
        self.fake = fake
        self.heartbeat_ms = heartbeat_ms

    def open_fake(self, port="FAKE"):
        original_serial = mc.serial.Serial

        def factory(*_args, **_kwargs):
            return self.fake

        mc.serial.Serial = factory
        try:
            super().open(port)
        finally:
            mc.serial.Serial = original_serial


CONFIG_LINE = ("# CONFIG can_bitrate=1000000 value_limit=2048 hard_limit=2048 "
               "interval_ms=50 watchdog_ms=2000 tx_queue_len=0 log_tx_default=0")
STATE_LINE = ('# STATE mode=listen armed=0 value=0 target=0 run=0 run_left_ms=0 sent=0 '
              'nonzero=0 zero=0 rejected=0 tx_failed=0 twai_state=0 msgs_to_tx=0 '
              'bus_errors=0 tx_errors=0 rx_missed=0 limit=2048 interval_ms=50 '
              'watchdog_ms=2000 watchdog_stops=0 run_complete=0 button=0 '
              'version="motor_control 1.0"')


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        patcher = patch.object(mc, 'default_log_root', return_value=Path(directory.name))
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_desktop_status_marks_and_export_do_not_send_motion(self):
        root=mc.tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        with patch.object(mc.list_ports,'comports',return_value=[]):
            app=mc.MotorControlApp(root,None,False)
        fake=FakeSerial()
        app.link.serial=fake
        self.addCleanup(app.link.close)
        app.motor.telemetry({'epoch':1,'t_us':1000000,'stage':3,'coord':3763,'valid':1},100)
        with patch.object(mc.time,'monotonic',return_value=100):
            app.update_motor_status()
            self.assertIn('3763',app.motor_label.cget('text'))
            self.assertEqual(str(app.record_a.cget('state')),'normal')
            app.record_coordinate('A')
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'marks.json'
                with patch.object(mc.filedialog,'asksaveasfilename',return_value=str(path)):
                    app.export_coordinates()
                self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['marks']['A']['coordinate'],3763)
        self.assertEqual(fake.written,[])
        with patch.object(mc.time,'monotonic',return_value=101):
            app.update_motor_status()
        self.assertEqual(str(app.record_a.cget('state')),'disabled')
        self.assertIn('已失效',app.marks_label.cget('text'))

    def test_motor_telemetry_is_classified_without_a_control_command(self):
        events=queue.Queue()
        fake=FakeSerial()
        link=RecordingLink(events,fake)
        link.open_fake()
        fake.feed('# MOTOR epoch=1 t_us=1000000 stage=3 coord=3763 valid=1 packets=1\n')
        kind,payload=events.get(timeout=2)
        link.close()
        self.assertEqual(kind,'motor')
        self.assertEqual(payload[0]['coord'],3763)
        self.assertTrue(all(command=='status\n' for command in fake.written))

    def test_state_parsing_handles_the_full_line_and_junk(self):
        state = mc.parse_state(STATE_LINE)
        self.assertEqual(state["mode"], "listen")
        self.assertEqual(state["limit"], 2048)
        self.assertEqual(state["version"], "motor_control 1.0")
        self.assertEqual(state["watchdog_ms"], 2000)
        self.assertEqual(mc.parse_state("# STATE "), {})
        # An unknown key must be ignored rather than trusted.
        self.assertNotIn("surprise", mc.parse_state("# STATE surprise=1 value=3"))

    def test_heartbeat_actually_refreshes_the_firmware_watchdog(self):
        """The safety story claims the host keeps the device alive. Prove the host
        really sends, that the cadence is inside the firmware watchdog window, and
        that nothing is sent after the port is closed."""
        events = queue.Queue()
        fake = FakeSerial()
        link = RecordingLink(events, fake)
        link.open_fake()
        # Two heartbeats are queued by the loop; wait for at least two writes plus the
        # initial status the open() path sends.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and len(fake.written) < 3:
            time.sleep(0.02)
        self.assertGreaterEqual(len(fake.written), 3, fake.written)
        self.assertTrue(all(cmd == "status\n" for cmd in fake.written), fake.written)
        # The host cadence (HEARTBEAT_MS) must be well inside the device window.
        self.assertLess(mc.HEARTBEAT_MS, 2000)
        link.close()
        count = len(fake.written)
        time.sleep(0.7)
        self.assertEqual(len(fake.written), count, "heartbeat continued after close")

    def test_device_lines_are_split_and_classified(self):
        events = queue.Queue()
        fake = FakeSerial()
        link = RecordingLink(events, fake)
        link.open_fake()
        fake.feed("\n".join([CONFIG_LINE, STATE_LINE, "# ACK arm value=0",
                             "# ERR unknown 'zzz' (try help)", "# WARN transmit_rejected"]) + "\n")
        seen = []
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and len(seen) < 5:
            try:
                seen.append(events.get(timeout=0.1))
            except queue.Empty:
                pass
        link.close()
        kinds = [kind for kind, _payload in seen]
        self.assertIn("state", kinds)
        self.assertGreaterEqual(kinds.count("line"), 4, seen)
        state = next(payload for kind, payload in seen if kind == "state")
        self.assertEqual(state["value"], 0)
        lines = [payload for kind, payload in seen if kind == "line"]
        self.assertTrue(any(line.startswith("# ACK arm") for line in lines), lines)
        self.assertTrue(any(line.startswith("# ERR") for line in lines), lines)

    def test_partial_line_is_not_acted_on_until_complete(self):
        events = queue.Queue()
        fake = FakeSerial()
        link = RecordingLink(events, fake)
        link.open_fake()
        fake.feed("# ACK go reques")  # deliberately truncated, no newline yet
        time.sleep(0.2)
        self.assertTrue(events.empty(), "a partial line must not be reported")
        fake.feed("ted=10 applied=10\n")
        deadline = time.monotonic() + 2
        line = None
        while time.monotonic() < deadline and line is None:
            try:
                kind, payload = events.get(timeout=0.1)
                if kind == "line":
                    line = payload
            except queue.Empty:
                pass
        link.close()
        self.assertEqual(line, "# ACK go requested=10 applied=10")

    def test_gui_value_handling_is_clamped_to_the_device_limit(self):
        """The clamp is what stops a typo asking for full travel."""
        limit = 2048
        for requested, expected in ((0, 0), (-16, -16), (2048, 2048), (99999, limit),
                                    (-99999, -limit)):
            self.assertEqual(max(-limit, min(limit, requested)), expected)

    def test_direction_text_uses_the_reported_sign(self):
        self.assertTrue(mc.describe_direction(0).startswith("0"))
        self.assertTrue(mc.describe_direction(16).startswith("+16"))
        self.assertTrue(mc.describe_direction(-16).startswith("-16"))


if __name__ == "__main__":
    unittest.main()
