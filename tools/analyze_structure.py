"""Offline 04:69 structure and sequence evidence. Never opens a serial port."""
import argparse
from collections import Counter, defaultdict, deque
from hashlib import sha256
import json
from pathlib import Path
from statistics import median

from analyze_protocol import reconstruct
from capture import parse_frame, summarize


def sequence(packet):
    return int.from_bytes(packet[6:8], "little")


def match_pairs(left, right):
    by_sequence = defaultdict(deque)
    for row in right:
        by_sequence[(row[0], sequence(row[2]))].append(row)
    paired, matched_right = [], 0
    for row in left:
        candidates = by_sequence[(row[0], (sequence(row[2]) + 1) % 65536)]
        if candidates:
            paired.append((row, candidates.popleft()))
            matched_right += 1
    return paired, len(left) - len(paired), len(right) - matched_right


def analyze_structure(path):
    raw = path.read_bytes()
    integrity = summarize(raw)
    packets = list(reconstruct(raw))
    channels = {identifier: [] for identifier in (0x503, 0x201)}
    can_bytes = Counter()
    frame_counts = Counter()
    dlcs = defaultdict(Counter)
    for line in raw.split(b"\n")[:-1]:
        frame = parse_frame(line.decode("ascii", errors="replace"))
        if frame is not None and not frame[3] and not frame[4]:
            can_bytes[frame[2]] += len(frame[-1])
            frame_counts[frame[2]] += 1
            dlcs[frame[2]][frame[5]] += 1
    crc_bytes = Counter()
    for identifier, extended, segment, stamp, packet in packets:
        if not extended:
            crc_bytes[identifier] += len(packet)
        if (not extended and identifier in channels and len(packet) == 19
                and packet[4] == 0xE5 and packet[5] == {0x503: 0xBF, 0x201: 0x52}[identifier]
                and packet[8:11] == bytes.fromhex("000469")
                and packet[11:15] == bytes.fromhex("0F040819")):
            channels[identifier].append((segment, stamp, packet))
    details = {}
    for identifier, rows in channels.items():
        steps = Counter((sequence(b[2]) - sequence(a[2])) % 65536
                        for a, b in zip(rows, rows[1:]) if a[0] == b[0])
        unusual = [{"sequence_step": (sequence(b[2]) - sequence(a[2])) % 65536,
                    "relative_s": round((b[1] - integrity["first_t_us"]) / 1e6, 6),
                    "interval_ms": round((b[1] - a[1]) / 1000, 3)}
                   for a, b in zip(rows, rows[1:]) if a[0] == b[0]
                   and (sequence(b[2]) - sequence(a[2])) % 65536 != 2]
        details[f"0x{identifier:03X}"] = {
            "candidate_packets": len(rows), "sequence_step_counts": dict(steps),
            "non_two_sequence_steps": unusual,
            "variable_packet_offsets": [index for index in range(19)
                                        if len({row[2][index] for row in rows}) > 1],
            "can_frames": frame_counts[identifier], "dlc_counts": dict(dlcs[identifier]),
            "can_data_bytes": can_bytes[identifier], "crc_valid_bytes_all_commands": crc_bytes[identifier],
            "first_packet_hex": rows[0][2].hex(" ").upper() if rows else None}
    pairs, unmatched_left, unmatched_right = match_pairs(channels[0x503], channels[0x201])
    delays = [(right[1] - left[1]) / 1000 for left, right in pairs]
    gui = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
    return {"raw_log": str(path.resolve()), "sha256": sha256(raw).hexdigest(),
            "integrity": integrity, "elapsed_s": gui.get("elapsed"),
            "counter_deltas": gui.get("counter_deltas"), "channels": details,
            "pairing_rule": "Same reconstruction segment; 0x201 DUML sequence = 0x503 sequence + 1 modulo 65536.",
            "paired_packets": len(pairs),
            "equal_payload_pairs": sum(a[2][11:-2] == b[2][11:-2] for a, b in pairs),
            "unpaired_0x503": unmatched_left, "unpaired_0x201": unmatched_right,
            "driver_timestamp_delta_ms": {"minimum": min(delays), "median": median(delays), "maximum": max(delays)} if delays else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--output", type=Path, default=Path("reports/structure.json"))
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        packet = lambda n: bytes(6) + n.to_bytes(2, "little") + bytes(11)
        left = [(0, 10, packet(65535)), (1, 20, packet(3))]
        right = [(0, 5, packet(99)), (0, 11, packet(0)), (2, 21, packet(4))]
        pairs, missing_left, missing_right = match_pairs(left, right)
        assert len(pairs) == 1 and sequence(pairs[0][1][2]) == 0
        assert missing_left == 1 and missing_right == 2
        print("structure analyzer self-check passed")
        return
    if not args.logs:
        parser.error("provide saved capture logs")
    captures = [analyze_structure(path) for path in args.logs]
    report = {"scope": "Passive structure evidence; this is not a motor-control implementation.",
              "captures": captures,
              "total_paired_packets": sum(c["paired_packets"] for c in captures),
              "total_equal_payload_pairs": sum(c["equal_payload_pairs"] for c in captures),
              "packet_template_0x503": "55 13 04 03 E5 BF SS SS 00 04 69 0F 04 08 19 VV VV CC CC",
              "packet_template_0x201": "55 13 04 03 E5 52 SS SS 00 04 69 0F 04 08 19 VV VV CC CC",
              "offsets": {"header_crc8": 3, "logical_sender": 4, "logical_receiver": 5,
                          "sequence_uint16_le": [6, 7], "flags": 8, "command": [9, 10],
                          "payload_prefix": [11, 12, 13, 14], "candidate_int16_le": [15, 16],
                          "packet_crc16_le": [17, 18]},
              "limitations": ["Logical addresses and CAN IDs do not establish physical transmission direction.",
                              "Sequence jumps are observations, not proof of missing CAN frames.",
                              "Pairing and fixed structure do not establish handshake, acknowledgements, required cadence or independent control.",
                              "Candidate units and rate/increment semantics remain unknown; user cannot perform fixed-angle experiment."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    for capture in captures:
        print(Path(capture["raw_log"]).name, "pairs", capture["paired_packets"],
              "payload matches", capture["equal_payload_pairs"], "unpaired", capture["unpaired_0x503"], capture["unpaired_0x201"])
    print("total pairs:", report["total_paired_packets"])


if __name__ == "__main__":
    main()
