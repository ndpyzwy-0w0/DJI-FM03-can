"""Calibration UI/serial guard checks using fake serial only."""
import json
from pathlib import Path
import queue
import tempfile
import time
import unittest
from unittest.mock import patch
import motor_control as mc
from control_session import summarize
from test_motor_control import FakeSerial, RecordingLink
from check_calibration_candidate import check


READY = dict(armed=1, value=0, target=0, run=0, pulse=0,
             cal_pending=0, cal_attempts=0, cal_observed=0, cal_active_seen=0)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        patcher = patch.object(mc, 'default_log_root', return_value=Path(self.temp.name))
        patcher.start()
        self.addCleanup(patcher.stop)

    def app(self):
        root = mc.tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        with patch.object(mc.list_ports, 'comports', return_value=[]):
            app = mc.MotorControlApp(root, None, False)
        app.link.serial = FakeSerial()
        self.addCleanup(app.link.close)
        app.link.last_state = dict(READY)
        app.link.last_state_at = time.monotonic()
        return app

    def test_candidate_matches_two_valid_captures(self):
        result = check()
        self.assertEqual(result['fragment_lengths'], [8, 8])

    def test_cancel_old_firmware_motion_and_stale_never_trigger(self):
        app = self.app()
        with patch.object(mc.messagebox, 'askyesno', return_value=False), patch.object(mc.messagebox, 'showwarning'):
            app.calibrate()
            for change in ({'value':16}, {'target':-16}, {'run':1}, {'pulse':1}, {'armed':0}, {'cal_pending':1}):
                app.link.last_state = dict(READY, **change)
                app.calibrate()
            app.link.last_state = {'armed':1, 'value':0}
            app.calibrate()
            app.link.last_state = dict(READY)
            app.link.last_state_at -= 10
            app.calibrate()
        self.assertEqual(app.link.serial.written, [])

    def test_one_click_one_request_no_arm_zero_or_repeat(self):
        app = self.app()
        with patch.object(mc.messagebox, 'askyesno', return_value=True), patch.object(mc.messagebox, 'showwarning'):
            app.calibrate()
            app.calibrate()
        self.assertEqual(app.link.serial.written, ['calibrate\n'])
        for command in ('go 16', 'go -16', 'run 16 100', 'pulse 16 100 100 2', 'sweep 16', 'calibrate'):
            self.assertFalse(app.link.send(command))
        for command in ('stop', 'disarm', 'status', 'go 0'):
            self.assertTrue(app.link.send(command))
        app.link.last_state = dict(READY, armed=0)
        with patch.object(mc.messagebox, 'askyesno', return_value=True):
            app.clear_calibration()
        self.assertEqual(app.link.serial.written[-1], 'calibration_clear\n')
        self.assertTrue(app.link.calibration_unconfirmed)  # No ACK; no claim of cancellation.

    def test_active_stage_and_changed_state_during_modal_block(self):
        app = self.app()
        app.motor.telemetry(dict(epoch=1,t_us=1000000,stage=1,coord=0,valid=0),time.monotonic())
        self.assertFalse(app.calibration_ready())
        app.motor.reset('test')
        def changed(*args):
            app.link.last_state['target'] = 16
            return True
        with patch.object(mc.messagebox, 'askyesno', side_effect=changed), patch.object(mc.messagebox, 'showwarning'):
            app.calibrate()
        self.assertEqual(app.link.serial.written, [])

    def test_short_write_remains_guarded(self):
        app = self.app()
        with patch.object(app.link.serial, 'write', return_value=3):
            self.assertFalse(app.link.send('calibrate'))
        self.assertFalse(app.link.send('go 16'))
        self.assertTrue(app.link.send('stop'))

    def test_explicit_rejection_or_clear_ack_releases_host_uncertainty(self):
        fake = FakeSerial()
        link = RecordingLink(queue.Queue(), fake, heartbeat_ms=10000)
        link.open_fake()
        self.addCleanup(link.close)
        for reply in ('# ERR calibrate stop_ordinary_motion_first\n',
                      '# ACK calibration_clear local_only=1 no_motor_cancel_sent=1\n'):
            self.assertTrue(link.send('calibrate'))
            fake.feed(reply)
            deadline = time.monotonic()+2
            while link.calibration_unconfirmed and time.monotonic()<deadline:
                time.sleep(.01)
            self.assertFalse(link.calibration_unconfirmed)

    def test_ack_and_state_logged_without_claiming_mechanical_success(self):
        fake = FakeSerial()
        link = RecordingLink(queue.Queue(), fake, heartbeat_ms=10000)
        link.open_fake()
        self.addCleanup(link.close)
        link.last_state = dict(READY)
        link.record_event('calibration_request', operator_confirmed=True)
        self.assertTrue(link.send('calibrate'))
        fake.feed('# ACK calibrate candidate_sent=1 motor_started_unverified=1\n'
                  '# STATE cal_pending=1 cal_attempts=1 cal_sent=1 cal_observed=0\n')
        deadline = time.monotonic()+2
        while link.last_state.get('cal_pending') != 1 and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertEqual(link.last_state['cal_observed'],0)
        self.assertFalse(link.send('go 16'))
        link.close('test')
        report = summarize(link.last_session_dir)
        self.assertEqual(len(report['calibration_events']),2)
        self.assertEqual(report['last_state']['fields']['cal_observed'],0)
        self.assertTrue(report['integrity']['rx_offsets_cover_raw'])


if __name__ == '__main__':
    unittest.main()
