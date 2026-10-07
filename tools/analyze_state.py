"""Offline periodic state evidence. No serial access or CAN transmission."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict, deque
from hashlib import sha256
import json
import math
from pathlib import Path
from statistics import median
import struct

from analyze_protocol import reconstruct
from capture import summarize


def profile(packet):
    payload = packet[11:-2]
    subtype = payload.hex(" ") if packet[9:11] == bytes.fromhex("2420") else (
        payload[:2].hex(" ") if packet[9:11] == bytes.fromhex("042f") else "")
    return (packet[4], packet[5], packet[8], packet[9], packet[10], len(payload), subtype)


def period(rows):
    intervals = [(b[1] - a[1]) / 1000 for a, b in zip(rows, rows[1:])
                 if a[0] == b[0] and b[1] > a[1]]
    return median(intervals) if intervals else None


def copies(left, right):
    lookup = defaultdict(deque)
    for row in right:
        p = row[2]
        lookup[(row[0], p[6:11], len(p))].append(row)
    pairs = []
    for row in left:
        p = row[2]
        if lookup[(row[0], p[6:11], len(p))]:
            pairs.append((row, lookup[(row[0], p[6:11], len(p))].popleft()))
    changed = set()
    for a, b in pairs:
        changed.update(i for i, (x, y) in enumerate(zip(a[2], b[2])) if x != y)
    return {"pairs": len(pairs), "same_entire_packet": sum(a[2] == b[2] for a, b in pairs),
            "same_payload": sum(a[2][11:-2] == b[2][11:-2] for a, b in pairs),
            "same_packet_except_sender_and_crc": sum(a[2][:4] + a[2][5:-2] == b[2][:4] + b[2][5:-2] for a, b in pairs),
            "changed_packet_offsets": sorted(changed),
            "unpaired_left": len(left) - len(pairs), "unpaired_right": len(right) - len(pairs)}


def analyze(path):
    raw = path.read_bytes()
    integrity = summarize(raw)
    packets = list(reconstruct(raw))
    groups = defaultdict(list)
    dial = defaultdict(list)
    request_keys = set()
    for identifier, extended, segment, stamp, packet in packets:
        if not packet[8] & 0x80:
            request_keys.add((segment, packet[4], packet[5], packet[6:8], packet[9:11]))
        if not extended:
            groups[(identifier, *profile(packet))].append((segment, stamp, packet))
            if identifier == 0x503 and packet[4:6] == bytes.fromhex("e5bf") and packet[8:11] == bytes.fromhex("000469") and len(packet) == 19:
                dial[segment].append((stamp, int.from_bytes(packet[15:17], "little", signed=True)))
    dial_times = {segment: [row[0] for row in rows] for segment, rows in dial.items()}
    def dial_at(segment, stamp):
        rows = dial[segment]
        index = bisect_right(dial_times.get(segment, []), stamp) - 1
        if index >= 0 and 0 <= stamp - rows[index][0] <= 50000:
            return {"value": rows[index][1], "age_ms": round((stamp - rows[index][0]) / 1000, 3)}
        return None
    details = {}
    for key, rows in sorted(groups.items()):
        identifier, source, dest, flags, cmdset, command, length, subtype = key
        if (cmdset, command) == (4, 0x69):
            continue
        payloads = [row[2][11:-2] for row in rows]
        name = f'0x{identifier:03X} {source:02X}>{dest:02X} flags={flags:02X} cmd={cmdset:02X}:{command:02X} len={length} subtype={subtype}'
        item = {"packets": len(rows), "median_interval_ms": period(rows),
                "unique_payloads": len(set(payloads)), "payload_counts": Counter(p.hex(" ").upper() for p in payloads).most_common(5),
                "variable_payload_offsets": [i for i in range(length) if len({p[i] for p in payloads}) > 1]}
        if cmdset == 4 and command == 0x2F and length == 15:
            changes = [rows[0]] + [b for a, b in zip(rows, rows[1:]) if a[0] != b[0] or a[2][11:-2] != b[2][11:-2]]
            item["int32_le_payload_offset3_counts"] = dict(Counter(int.from_bytes(p[3:7], "little", signed=True) for p in payloads))
            item["state_changes"] = [{"segment": seg, "relative_s": round((t-integrity["first_t_us"]) / 1e6, 6),
                                      "int32_le_offset3": int.from_bytes(p[14:18], "little", signed=True),
                                      "preceding_04_69": dial_at(seg, t)} for seg, t, p in changes]
        if cmdset == 4 and command == 0x2F and length == 13:
            item["offset2_value_counts"] = dict(Counter(p[2] for p in payloads))
            item["cycle_1_2_3_pairs"] = sum(b[2][13] == a[2][13] % 3 + 1 for a, b in zip(rows, rows[1:]) if a[0] == b[0])
            item["same_segment_pairs"] = sum(a[0] == b[0] for a, b in zip(rows, rows[1:]))
        if flags & 0x80:
            item["opposite_address_same_sequence_request_matches"] = sum(
                (seg, p[5], p[4], p[6:8], p[9:11]) in request_keys for seg, t, p in rows)
        if identifier == 0x106 and cmdset == 4 and command == 5 and length == 45:
            vectors = [struct.unpack("<4f", p[24:40]) for p in payloads]
            norms = [sum(v*v for v in q) for q in vectors if all(math.isfinite(v) for v in q)]
            increments = [(int.from_bytes(b[2][23:27], "little") - int.from_bytes(a[2][23:27], "little")) % 2**32
                          for a, b in zip(rows, rows[1:]) if a[0] == b[0]]
            item["four_float32_le_payload_offset24"] = {"finite_packets": len(norms), "norm_squared_min": min(norms) if norms else None,
                                                       "norm_squared_max": max(norms) if norms else None,
                                                       "maximum_error_from_one": max(abs(n-1) for n in norms) if norms else None}
            item["uint32_le_payload_offset12_increment"] = {"median": median(increments) if increments else None,
                                                           "minimum": min(increments) if increments else None, "maximum": max(increments) if increments else None}
        details[name] = item
    def select(identifier, source):
        return sorted([(seg, stamp, p) for ident, ext, seg, stamp, p in packets
                       if not ext and ident == identifier and p[4] == source and p[5] == 0x52 and p[8:11] == bytes.fromhex("00042f")], key=lambda row: row[1])
    return {"raw_log": str(path.resolve()), "sha256": sha256(raw).hexdigest(), "integrity": integrity,
            "crc_valid_packets": len(packets), "groups": details,
            "copy_0x504_BF_to_0x232_BF": copies(select(0x504, 0xBF), select(0x232, 0xBF)),
            "copy_0x232_BF_to_0x201_DF": copies(select(0x232, 0xBF), select(0x201, 0xDF))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--output", type=Path, default=Path("reports/state_evidence.json"))
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        packet = bytearray(16);packet[9:11] = bytes.fromhex("2420");packet[11] = 0x5A
        other = packet.copy();other[11] = 0x5D
        assert profile(packet) != profile(other)
        assert period([(0, 0, packet), (0, 100000, packet)]) == 100
        assert period([(0, 0, packet), (1, 100000, packet)]) is None
        test = copies([(0, 1, bytes(packet))], [(1, 2, bytes(packet))])
        assert test["pairs"] == 0 and test["unpaired_left"] == 1
        rewritten = packet.copy();rewritten[4] = 0xDF;rewritten[-1] ^= 1
        test = copies([(0, 1, bytes(packet))], [(0, 2, bytes(rewritten))])
        assert test["pairs"] == test["same_payload"] == test["same_packet_except_sender_and_crc"] == 1
        rewritten[11] ^= 1
        assert copies([(0, 1, bytes(packet))], [(0, 2, bytes(rewritten))])["same_payload"] == 0
        q = struct.unpack("<4f", struct.pack("<4f", 1, 0, 0, 0))
        assert sum(v*v for v in q) == 1
        print("state analyzer self-check passed")
        return
    if not args.logs:
        parser.error("provide saved capture logs")
    captures = [analyze(path) for path in args.logs]
    report = {"scope": "Passive observations, no confirmed motor feedback or startup/enable sequence.", "captures": captures,
              "external_reference": {"checked_on": "2026-10-07", "provenance": "Original community DUML dissector source, not DJI RS3/FM03 documentation.",
                                     "header_source": "https://raw.githubusercontent.com/o-gs/dji-firmware-tools/master/comm_dissector/wireshark/dji-dumlv1-proto.lua",
                                     "gimbal_source": "https://raw.githubusercontent.com/o-gs/dji-firmware-tools/master/comm_dissector/wireshark/dji-dumlv1-gimbal.lua",
                                     "scope": "Generic request/response flag bit 0x80 only; inspected gimbal definitions do not define 0x2F or 0x69. No device-specific units imported."},
              "offset_convention": "zero-based application payload, excluding 11-byte DUML header and CRC",
              "limitations": ["CAN IDs and logical addresses do not establish physical direction or relay topology.",
                              "Normalized four-float vector is a quaternion candidate only; axes/device/frame are unknown.",
                              "Periodic state and copying do not prove a required heartbeat or startup handshake.",
                              "04:69 context is a preceding field sample within 50ms, not a measured motor/dial motion state."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    for capture in captures:
        print(Path(capture["raw_log"]).name, "CRC", capture["crc_valid_packets"],
              "copy pairs", capture["copy_0x504_BF_to_0x232_BF"]["pairs"], capture["copy_0x232_BF_to_0x201_DF"]["pairs"])


if __name__ == "__main__":
    main()
