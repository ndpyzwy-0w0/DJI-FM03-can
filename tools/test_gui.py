"""Run: .venv/Scripts/python.exe -m unittest discover -s tools -p test_gui.py"""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import serial
from gui_server import CaptureSession, Handler, ThreadingHTTPServer


RAW = (b"# STATUS t_us=100 bitrate=1000000 rx=5 logger_dropped=2 rx_missed=0 rx_overrun=0 bus_errors=1\n"
       b"101,1,00000123,0,0,2,AABB\n"
       b"110,3,001ABCDE,1,1,8,\n"
       b"# STATUS t_us=200 bitrate=1000000 rx=7 logger_dropped=3 rx_missed=1 rx_overrun=0 bus_errors=3\n")


class FakeSerial:
    def __init__(self, **_):
        self.chunks = [RAW[:19], RAW[19:117], RAW[117:]]
        self.closed = False
        self.disconnect = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def open(self):
        pass

    def write(self, data):
        assert data == b"s"
        return 1

    def read(self, _):
        if self.chunks:
            return self.chunks.pop(0)
        if self.disconnect:
            raise serial.SerialException("test device disconnected")
        time.sleep(0.01)
        return b""

    def close(self):
        self.closed = True


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("capture did not reach expected state")


class GuiTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.patch_ports = patch("gui_server.available_ports", return_value=[
            {"port": "COM_TEST", "description": "test fixture", "usb": True}])
        self.patch_ports.start()
        self.addCleanup(self.patch_ports.stop)
        self.devices = []

    def factory(self, **kwargs):
        device = FakeSerial(**kwargs)
        self.devices.append(device)
        return device

    def session(self):
        session = CaptureSession(Path(self.temp.name), self.factory)
        self.addCleanup(session.stop)
        return session

    def test_fragmented_raw_bytes_stop_and_restart(self):
        session = self.session()
        session.start("COM_TEST", "idle")
        wait_for(lambda: session.snapshot()["frames"] == 2)
        with self.assertRaises(ValueError):
            session.start("COM_TEST", "second")
        session.stop()
        state = session.snapshot()
        self.assertEqual(state["phase"], "stopped")
        self.assertEqual(session.raw_path.read_bytes(), RAW)
        self.assertTrue(self.devices[0].closed)
        self.assertEqual(state["sequence_gaps"], 1)
        self.assertEqual(state["counter_deltas"], {"logger_dropped": 1, "rx_missed": 1, "rx_overrun": 0, "bus_errors": 2})
        self.assertTrue(state["latest"][0]["rtr"])
        self.assertEqual(state["latest"][0]["data"], "")
        summary = json.loads(session.summary_path.read_text(encoding="utf-8"))
        self.assertEqual(summary["frames"], 2)
        first_path = session.raw_path
        session.start("COM_TEST", "slow_forward")
        wait_for(lambda: session.snapshot()["frames"] == 2)
        session.stop()
        self.assertNotEqual(first_path, session.raw_path)
        self.assertEqual(first_path.read_bytes(), RAW)
        session.stop()  # Idempotent stop, port is still closed.

    def test_disconnect_keeps_raw_and_summary(self):
        session = self.session()
        def broken_factory(**kwargs):
            device = self.factory(**kwargs)
            device.disconnect = True
            return device
        session.serial_factory = broken_factory
        session.start("COM_TEST", "disconnect_check")
        wait_for(lambda: session.snapshot()["phase"] == "error")
        self.assertEqual(session.raw_path.read_bytes(), RAW)
        self.assertTrue(session.summary_path.is_file())
        self.assertTrue(self.devices[0].closed)
        self.assertIn("disconnected", session.snapshot()["error"])

    def test_display_limit_and_reboot_counters(self):
        session = self.session()
        session._consume(RAW)
        session._consume(b"# STATUS t_us=1 bitrate=1000000 logger_dropped=1 rx_missed=0 rx_overrun=0 bus_errors=0\n")
        self.assertEqual(session.snapshot()["counter_deltas"]["logger_dropped"], 2)
        session._consume(b"".join(f"{n},{n},00000123,0,0,1,AA\n".encode() for n in range(1, 201)))
        state = session.snapshot()
        self.assertEqual(state["frames"], 202)
        self.assertEqual(len(state["latest"]), 150)
        self.assertEqual(state["sequence_resets"], 1)

    def test_invalid_port_and_label(self):
        session = self.session()
        for port, label in (("missing", "idle"), ("COM_TEST", "../oops"), ("COM_TEST", None)):
            with self.assertRaises(ValueError):
                session.start(port, label)

    def test_http_controls_and_download_guard(self):
        session = self.session()
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.capture, server.token = session, "test-token"
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        def close_server():
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
        self.addCleanup(close_server)
        base = f"http://127.0.0.1:{server.server_port}"
        def request(route, data=None, token=None, extra=None):
            headers = extra or {}
            if token:
                headers["X-FM03-Token"] = token
            payload = json.dumps(data).encode() if data is not None else None
            return urlopen(Request(base + route, data=payload, headers=headers), timeout=3)
        with request("/api/state") as response:
            self.assertEqual(json.load(response)["product"], "fm03_capture")
        for headers in ({}, {"X-FM03-Token": "test-token", "Origin": "http://example.com"}):
            with self.assertRaises(HTTPError) as caught:
                request("/api/start", {"port": "COM_TEST", "label": "idle"}, extra=headers)
            self.assertEqual(caught.exception.code, 403)
        with self.assertRaises(HTTPError) as caught:
            request("/api/state", extra={"Host": "example.com"})
        self.assertEqual(caught.exception.code, 403)
        with request("/api/start", {"port": "COM_TEST", "label": "idle"}, "test-token"):
            pass
        wait_for(lambda: session.snapshot()["frames"] == 2)
        with request("/api/stop", {}, "test-token") as response:
            result = json.load(response)
        self.assertEqual(result["phase"], "stopped")
        with request("/captures/" + result["raw_file"]) as response:
            self.assertEqual(response.read(), RAW)
        with self.assertRaises(HTTPError) as caught:
            request("/captures/..%2FREADME.md")
        self.assertEqual(caught.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
