"""Real session/serial paths with fake ports only; no physical hardware is opened."""
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import motor_control as mc
from control_session import SessionLog, summarize
from test_motor_control import FakeSerial, STATE_LINE


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def open_link(self, fake):
        link = mc.MotorLink(queue.Queue(), self.root)
        with patch.object(mc.serial, 'Serial', return_value=fake), patch.object(mc.list_ports, 'comports', return_value=[]):
            link.open('FAKE')
        self.addCleanup(link.close)
        return link

    def wait_for(self, predicate):
        end = time.monotonic() + 2
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.01)
        self.fail('fake-port condition timed out')

    def test_complete_session_keeps_exact_bytes_typed_feedback_commands_and_marks(self):
        fake = FakeSerial()
        link = self.open_link(fake)
        self.assertTrue(link.send('arm'))
        self.assertTrue(link.send('go -16'))
        prefix = (STATE_LINE + '\r\n# MOTOR epoch=1 t_us=1000000 stage=3 coord=3763 valid=1 packets=1\n').encode('ascii')
        suffix = b'# ACK go requested=-16 applied=-16\n\xffunknown\npartial'
        with fake._lock:
            fake._pending += prefix + suffix
        self.wait_for(lambda: link.lines_read >= 4)
        link.record_event('observation', text='绿灯常亮；从齿轮端看反向转了一小段', source='user')
        link.record_event('coordinate_mark', mark={'marker':'A', 'coordinate':3763})
        link.record_event('coordinate_snapshot', snapshot={'marks':{'A':{'coordinate':3763,'valid':False}},'reason':'连接已断开'})
        link.close('test_finished')
        self.assertEqual((link.last_session_dir/'rx.log').read_bytes(), prefix+suffix)
        report = json.loads((link.last_session_dir/'summary.json').read_text(encoding='utf-8'))
        self.assertTrue(report['ended_normally'])
        self.assertTrue(report['integrity']['rx_offsets_cover_raw'])
        self.assertEqual(report['integrity']['rx_unterminated_tail_bytes'],7)
        self.assertEqual(report['integrity']['commands_without_write_result'], [])
        self.assertEqual(report['state_samples'], 1)
        self.assertEqual(report['working_coordinate_range'], [3763,3763])
        self.assertEqual(report['observations'][0]['text'], '绿灯常亮；从齿轮端看反向转了一小段')
        self.assertEqual(report['last_coordinate_snapshot']['marks']['A']['valid'], False)
        self.assertEqual([v['command'] for v in report['command_timeline_excluding_heartbeat']], ['arm','go -16'])
        rows=[json.loads(line) for line in (link.last_session_dir/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertTrue(all(row['at_beijing'].endswith('+08:00') for row in rows))
        self.assertEqual([row['seq'] for row in rows],list(range(1,len(rows)+1)))
        self.assertTrue(any(row['kind']=='rx_line' and '\ufffdunknown' in row['line'] for row in rows))

    def test_interrupted_partial_journal_is_recoverable_without_rewriting_evidence(self):
        session=SessionLog(self.root,'FAKE',921600)
        session.receive(b'unterminated\xff')
        session.event('tx_attempt',tx_id=1,command='go 16')
        session.journal.write('{"seq":')
        session.journal.close()
        session.raw.close()
        before=(session.path/'rx.log').read_bytes(),(session.path/'events.jsonl').read_bytes()
        report=summarize(session.path)
        self.assertFalse(report['ended_normally'])
        self.assertEqual(report['integrity']['malformed_jsonl_lines'],[4])
        self.assertEqual(report['integrity']['commands_without_write_result'],[1])
        self.assertTrue(report['integrity']['rx_offsets_cover_raw'])
        self.assertEqual(before,((session.path/'rx.log').read_bytes(),(session.path/'events.jsonl').read_bytes()))

    def test_reconnect_creates_separate_sessions_and_counter_resets_are_not_negative_deltas(self):
        fake=FakeSerial()
        link=self.open_link(fake)
        first=link.last_session_dir
        for value in (20,25,0,4):
            link.record_event('rx_line',category='state',line='# STATE',fields={'sent':value,'bus_errors':0,'version':'fake'})
        link.close()
        self.assertEqual(summarize(first)['observed_counter_positive_deltas']['sent'],9)
        self.assertEqual(summarize(first)['counter_decrease_events'],1)
        before=(first/'events.jsonl').read_bytes()
        with patch.object(mc.serial,'Serial',return_value=fake),patch.object(mc.list_ports,'comports',return_value=[]):
            link.open('FAKE')
        self.assertNotEqual(first,link.last_session_dir)
        link.close()
        self.assertEqual(before,(first/'events.jsonl').read_bytes())

    def test_write_failure_is_not_reported_as_success(self):
        fake=FakeSerial()
        link=self.open_link(fake)
        with patch.object(fake,'write',return_value=2):
            self.assertFalse(link.send('go 256'))
        link.close()
        report=summarize(link.last_session_dir)
        self.assertTrue(any(item['type']=='command_write_failed' for item in report['anomalies']))
        command=next(item for item in report['command_timeline_excluding_heartbeat'] if item['command']=='go 256')
        self.assertFalse(command['write_ok'])

    def test_logging_failure_blocks_new_motion_but_allows_stop(self):
        fake=FakeSerial()
        link=self.open_link(fake)
        with patch.object(link.session.journal,'flush',side_effect=OSError('disk full')):
            self.assertFalse(link.send('go 16'))
            self.assertTrue(link.send('stop'))
        self.assertNotIn('go 16\n',fake.written)
        self.assertIn('stop\n',fake.written)
        self.assertIn('disk full',link.logging_error)
        link.close()
        self.assertFalse(summarize(link.last_session_dir)['ended_normally'])

    def test_gui_observations_and_ab_records_are_automatic_and_expire_without_export(self):
        root=mc.tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        with patch.object(mc.list_ports,'comports',return_value=[]):
            app=mc.MotorControlApp(root,None,False)
        app.link.log_root=self.root
        app.port_var.set('FAKE')
        fake=FakeSerial()
        with patch.object(mc.serial,'Serial',return_value=fake),patch.object(mc.list_ports,'comports',return_value=[]):
            app.observation_var.set('RS3已断开，电机绿灯常亮')
            app.connect()
        self.addCleanup(app.link.close)
        app.motor.telemetry({'epoch':1,'t_us':1000000,'stage':3,'coord':100,'valid':1},time.monotonic())
        app.record_coordinate('A')
        app.motor.telemetry({'epoch':1,'t_us':1100000,'stage':3,'coord':200,'valid':1},time.monotonic())
        app.record_coordinate('B')
        app.motor.seen-=1
        app.update_motor_status()
        app.disconnect()
        report=summarize(app.link.last_session_dir)
        self.assertEqual(len(report['coordinate_marks_at_click']),2)
        self.assertEqual(report['observations'][0]['text'],'RS3已断开，电机绿灯常亮')
        self.assertTrue(all(not mark['valid'] for mark in report['last_coordinate_snapshot']['records']))
        self.assertTrue(all(command=='status\n' for command in fake.written))

    def test_read_failure_stops_blind_heartbeats_and_records_error(self):
        fake=FakeSerial()
        link=self.open_link(fake)
        with patch.object(fake,'read',side_effect=OSError('unplugged')):
            self.wait_for(link.stopping.is_set)
        self.assertFalse(link.send('arm'))
        link.close('serial_error')
        report=summarize(link.last_session_dir)
        self.assertEqual(report['end_reason'],'serial_error')
        self.assertTrue(any(item['type']=='io_error' for item in report['anomalies']))

    def test_concurrent_evidence_is_sequenced_and_log_creation_failure_never_opens_serial(self):
        session=SessionLog(self.root,'FAKE',921600)
        workers=[threading.Thread(target=lambda:[session.event('observation',text='parallel') for _ in range(50)]) for _ in range(3)]
        for worker in workers:worker.start()
        for worker in workers:worker.join()
        session.close()
        report=summarize(session.path)
        self.assertEqual(report['event_counts']['observation'],150)
        self.assertFalse(report['anomalies'])
        blocked=self.root/'not_a_folder'
        blocked.write_text('keep',encoding='utf-8')
        link=mc.MotorLink(queue.Queue(),blocked)
        with patch.object(mc.serial,'Serial') as factory:
            with self.assertRaises(OSError):link.open('FAKE')
            factory.assert_not_called()


if __name__=='__main__':
    unittest.main()
