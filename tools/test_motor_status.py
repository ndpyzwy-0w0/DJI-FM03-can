"""Offline regression checks using saved bytes and fake serial/HTTP only."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from analyze_calibration import state09
from analyze_protocol import crc, reconstruct_frames
from capture import parse_frame
from gui_server import CaptureSession
from motor_status import LiveMotor, FramePackets, parse_motor_line

ROOT = Path(__file__).resolve().parents[1]


def state_packet(stage=3, coordinate=1878, receiver=0xe5):
    packet = bytearray([0x55,28,4,0,0xbf,receiver,1,0,0,4,0x2f,9,0,stage])
    packet += coordinate.to_bytes(4,'little',signed=True)+bytes(8)
    packet[3] = crc(packet[:3],0x77,0x8c)
    packet += crc(packet,0x3692,0x8408).to_bytes(2,'little')
    return bytes(packet)


def feed_state(motor, packet, now=1, stamp=1000000):
    for offset in range(0,len(packet),8):
        data = packet[offset:offset+8]
        motor.feed((stamp+offset,1,0x504,0,0,len(data),data),now)


class MotorStatusTests(unittest.TestCase):
    def test_validity_marks_recalibration_sentinels_and_timeout(self):
        motor = LiveMotor()
        with self.assertRaises(ValueError): motor.record('A',1)
        feed_state(motor,state_packet(1,-2147483648))
        self.assertEqual(motor.snapshot(1)['stage'],1)
        self.assertFalse(motor.snapshot(1)['can_record'])
        feed_state(motor,state_packet(),1.1,1100000)
        a = motor.record('A',1.1)
        self.assertEqual(a['coordinate'],1878)
        feed_state(motor,state_packet(3,2562),1.2,1200000)
        motor.record('B',1.2)
        self.assertEqual(motor.snapshot(1.2)['marks']['B']['coordinate']-a['coordinate'],684)
        feed_state(motor,state_packet(2,2147483647),1.3,1300000)
        self.assertFalse(motor.snapshot(1.3)['marks']['A']['valid'])
        feed_state(motor,state_packet(3,3000),1.4,1400000)
        self.assertFalse(motor.snapshot(1.4)['marks']['A']['valid'])
        motor.record('A',1.4)
        feed_state(motor,state_packet(3,-2147483648),1.45,1450000)
        self.assertFalse(motor.snapshot(1.45)['marks']['A']['valid'])
        feed_state(motor,state_packet(3,3000),1.5,1500000)
        motor.record('A',1.5)
        self.assertFalse(motor.snapshot(2.1)['can_record'])
        self.assertFalse(motor.snapshot(2.1)['marks']['A']['valid'])

    def test_packet_crc_wrong_route_and_gap_do_not_fake_a_coordinate(self):
        motor = LiveMotor()
        bad = bytearray(state_packet());bad[-1] ^= 1
        feed_state(motor,bad)
        feed_state(motor,state_packet(receiver=0x52),1.1,1100000)
        self.assertIsNone(motor.snapshot(1.1)['coordinate'])
        packet = state_packet()
        motor.feed((1200000,1,0x504,0,0,8,packet[:8]),1.2)
        motor.reset('序号缺口')
        for offset in (8,16,24):
            data=packet[offset:offset+8]
            motor.feed((1200000+offset,1,0x504,0,0,len(data),data),1.2)
        self.assertIsNone(motor.snapshot(1.2)['coordinate'])
        feed_state(motor,packet,1.3,1300000)
        self.assertEqual(motor.snapshot(1.3)['coordinate'],1878)

    def test_continuation_with_valid_header_is_not_a_new_packet(self):
        packet = bytearray(state_packet())
        packet[16:20] = bytes.fromhex('55 0D 04 33')
        packet[-2:] = crc(packet[:-2],0x3692,0x8408).to_bytes(2,'little')
        stream = FramePackets()
        results = []
        for offset in range(0,len(packet),8):
            results += stream.feed(offset,bytes(packet[offset:offset+8]))
        self.assertEqual(results,[(0,bytes(packet))])

    def test_uart_repeated_sample_cannot_refresh_stale_status(self):
        motor = LiveMotor()
        fields = parse_motor_line('# MOTOR epoch=2 t_us=1000000 stage=3 coord=3763 valid=1 packets=1')
        motor.telemetry(fields,1)
        motor.record('A',1)
        motor.telemetry(fields,2)
        self.assertFalse(motor.snapshot(2)['can_record'])
        self.assertFalse(motor.snapshot(2)['marks']['A']['valid'])
        motor.telemetry({**fields,'epoch':3,'t_us':1100000},2.1)
        self.assertTrue(motor.snapshot(2.1)['can_record'])
        self.assertFalse(motor.snapshot(2.1)['marks']['A']['valid'])
        self.assertIsNone(parse_motor_line('# MOTOR stage=3 coord=1'))
        self.assertIsNone(parse_motor_line('# MOTOR epoch=1 t_us=1 stage=3 coord=x valid=1'))

    def test_current_bus_frames_make_an_old_coordinate_stale(self):
        motor=LiveMotor()
        feed_state(motor,state_packet(),1,1000000)
        motor.record('A',1)
        motor.feed((1700000,20,0x123,0,0,1,b'\x00'),1.01)
        self.assertFalse(motor.snapshot(1.01)['can_record'])
        self.assertFalse(motor.snapshot(1.01)['marks']['A']['valid'])

    def test_all_saved_coordinate_samples_agree_with_offline_reconstructor(self):
        path = ROOT/'captures/20261009_132137_540179_calibration.log'
        raw = path.read_bytes()
        expected = []
        for entry in reconstruct_frames(raw)['packets']:
            b = entry['packet']
            state = state09(b)
            if entry['can_id']==0x504 and b[4:6]==b'\xbf\xe5' and b[8]==0 and state:
                expected.append((entry['stamp'],*state))
        motor = LiveMotor()
        actual = []
        for line in raw.split(b'\n')[:-1]:
            frame = parse_frame(line.decode('ascii',errors='replace'))
            if frame:
                before = motor.packets
                motor.feed(frame,frame[0]/1e6)
                if motor.packets != before:
                    actual.append((motor.stamp,motor.stage,motor.coordinate))
        self.assertEqual(actual,expected)
        self.assertEqual(sum(stage==3 for _,stage,_ in actual),360)
        self.assertEqual(motor.coordinate,1878)

    def test_gui_records_sidecar_without_writing_serial_or_raw(self):
        with tempfile.TemporaryDirectory() as directory:
            session = CaptureSession(Path(directory))
            session.phase='capturing'
            session.started=time.monotonic()
            session.raw_path=Path(directory)/'20261010_000000_000000_calibration.log'
            session.raw_path.write_bytes(b'ORIGINAL\n')
            packet=state_packet(3,4096)
            for i,offset in enumerate(range(0,len(packet),8),1):
                data=packet[offset:offset+8]
                session._consume(f'{1000000+offset},{i},00000504,0,0,{len(data)},{data.hex()}\n'.encode())
            session.record_coordinate('A')
            self.assertEqual(session.raw_path.read_bytes(),b'ORIGINAL\n')
            path = session.raw_path.with_suffix('.coordinates.json')
            result=json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(result['marks']['A']['coordinate'],4096)
            self.assertEqual(session.snapshot()['coordinate_file'],path.name)
            self.assertIsNone(session.thread)
            session._consume(b'# STATUS t_us=2000000 logger_dropped=0 rx_missed=0 rx_overrun=0\n')
            session._consume(b'# STATUS t_us=2100000 logger_dropped=1 rx_missed=0 rx_overrun=0\n')
            self.assertFalse(session.snapshot()['motor']['marks']['A']['valid'])
            with self.assertRaises(ValueError): session.record_coordinate('B')


if __name__=='__main__':
    unittest.main()
