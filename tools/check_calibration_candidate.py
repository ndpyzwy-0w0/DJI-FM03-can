"""Offline byte-evidence check. Never opens a serial port or sends CAN."""
import json
from pathlib import Path
import re
from analyze_protocol import crc, reconstruct_frames


def check():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/motor_control.cpp').read_text(encoding='utf-8')
    template = re.search(r'CALIBRATE_042F\[16\]\s*=\s*\{([^}]+)\}', source).group(1)
    packet = bytes(int(v, 16) for v in re.findall(r'0x([0-9a-fA-F]+)', template))
    assert len(packet) == 16 and packet[11:14] == bytes.fromhex('02 00 01')
    assert crc(packet[:3], 0x77, 0x8c) == packet[3]
    assert crc(packet[:-2], 0x3692, 0x8408) == int.from_bytes(packet[-2:], 'little')
    evidence = []
    for name in ('20261009_003846_528282_calibration', '20261009_132137_540179_calibration'):
        capture = root / 'captures' / (name + '.log')
        matches = [p for p in reconstruct_frames(capture.read_bytes())['packets']
                   if p['can_id'] == 0x503 and p['packet'][4:6] == bytes.fromhex('E5 BF')
                   and p['packet'][8:14] == bytes.fromhex('00 04 2F 02 00 01')]
        assert len(matches) == 1
        captured = matches[0]['packet']
        assert matches[0]['frames_used'] == 2
        assert captured[:6] == packet[:6] and captured[8:14] == packet[8:14]
        evidence.append({'capture': str(capture.relative_to(root)), 'packet_hex': captured.hex(),
                         'frames': 2, 'stamp': matches[0]['stamp']})
    assert bytes.fromhex(evidence[-1]['packet_hex']) == packet
    return {'candidate_hex': packet.hex(), 'first_manual_attempt_sequence': '0xdb21',
            'fragment_lengths': [8, 8], 'evidence': evidence,
            'limits': 'Byte evidence only; no replay, cancel, persistence, or mechanical success verification.'}


if __name__ == '__main__':
    print(json.dumps(check(), ensure_ascii=False, indent=2))
