"""Build an observed DUML catalog from saved logs; no serial access or transmission."""

import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

from analyze_protocol import reconstruct
from analyze_state import profile
from capture import summarize


def identity(row):
    identifier, extended, segment, stamp, packet = row
    fields = profile(packet)
    if packet[9:11] == b"\x04\x69":
        fields = (*fields[:-1], packet[11:15].hex(" "))
    return (identifier, extended, *fields)


def describe(rows):
    rows = sorted(rows, key=lambda row: row[3])
    packets = [row[4] for row in rows]
    payloads = [packet[11:-2] for packet in packets]
    intervals, steps = [], Counter()
    for left, right in zip(rows, rows[1:]):
        if left[2] == right[2] and right[3] >= left[3]:
            intervals.append((right[3]-left[3])/1000)
            steps[(int.from_bytes(right[4][6:8], "little")
                   - int.from_bytes(left[4][6:8], "little")) % 65536] += 1
    values = [sorted({payload[i] for payload in payloads}) for i in range(len(payloads[0]))]
    return {"packets": len(rows), "unique_payloads": len(set(payloads)),
            "first_device_t_us": rows[0][3], "last_device_t_us": rows[-1][3],
            "median_interval_ms": median(intervals) if intervals else None,
            "interval_min_ms": min(intervals) if intervals else None,
            "interval_max_ms": max(intervals) if intervals else None,
            "sequence_steps_mod_65536": dict(sorted(steps.items())),
            "payload_byte_values": values,
            "variable_payload_offsets": [i for i, v in enumerate(values) if len(v)>1],
            "common_payloads": Counter(payload.hex(" ") for payload in payloads).most_common(3),
            "first_packet_hex": packets[0].hex(" "), "last_packet_hex": packets[-1].hex(" ")}


def packet_mask(packets):
    """Observed constants only; sequence and CRC remain variable even in one sample."""
    length = len(packets[0])
    return " ".join("??" if i in (6, 7, length-2, length-1)
                    or len({packet[i] for packet in packets})>1 else f"{packets[0][i]:02X}"
                    for i in range(length))


def bf_cycles(rows):
    """Observed BF burst order, anchored by 24:20/5A4101; not a send schedule."""
    rows = sorted(rows, key=lambda row: row[3])
    anchors = [i for i, row in enumerate(rows) if row[0] == 0x232 and not row[1]
               and row[4][4:6] == b"\xbf\x32" and row[4][8:11] == b"\x20\x24\x20"
               and row[4][11:-2] == b"\x5a\x41\x01"]
    patterns, base_steps, intervals = Counter(), Counter(), []
    example = None
    for start, end in zip(anchors, anchors[1:]):
        left, right = rows[start], rows[end]
        if left[2] != right[2] or any(row[2] != left[2] for row in rows[start:end+1]):
            continue
        base = int.from_bytes(left[4][6:8], "little")
        base_steps[(int.from_bytes(right[4][6:8], "little")-base) % 65536] += 1
        intervals.append((right[3]-left[3])/1000)
        seen, slots = set(), []
        for row in rows[start:end]:
            packet = row[4]
            if row[1] or packet[4] != 0xBF or packet in seen or packet[9:11] == b"\x00\x01":
                continue
            seen.add(packet)
            payload = packet[11:-2]
            slots.append(((int.from_bytes(packet[6:8], "little")-base) % 65536,
                          f"{packet[5]:02X}", f"{packet[8]:02X}", packet[9:11].hex(":"),
                          payload[:2].hex(" ")))
        pattern = tuple(slots)
        patterns[pattern] += 1
        if example is None:
            example = {"segment": left[2], "anchor_device_t_us": left[3],
                       "first_cycle_slots": slots}
    return {"complete_anchor_intervals": len(intervals),
            "median_anchor_interval_ms": median(intervals) if intervals else None,
            "anchor_sequence_steps": dict(sorted(base_steps.items())),
            "patterns": [{"observed_cycles": count, "slots": pattern}
                         for pattern, count in patterns.most_common()], "example": example,
            "scope": "BF standard-ID packets only, exact copies within each cycle deduplicated; 00:01 excluded; no cross-segment intervals"}


def build(paths):
    all_groups = defaultdict(dict)
    captures = []
    for path in paths:
        raw = path.read_bytes()
        summary = summarize(raw)
        rows = list(reconstruct(raw))
        groups = defaultdict(list)
        for row in rows:
            groups[identity(row)].append(row)
        capture = {"raw_log": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest(),
                   "integrity": summary, "crc_valid_packets": len(rows),
                   "trailing_bytes": len(raw.rsplit(b"\n", 1)[-1]), "bf_cycle_observation": bf_cycles(rows)}
        captures.append(capture)
        for key, records in groups.items():
            all_groups[key][path.name] = records
    result = []
    for key, per_capture in sorted(all_groups.items()):
        identifier, extended, source, dest, flags, cmdset, command, length, subtype = key
        packets = [row[4] for rows in per_capture.values() for row in rows]
        payloads = [packet[11:-2] for packet in packets]
        values = [sorted({payload[i] for payload in payloads}) for i in range(length)]
        result.append({"can_id": f"0x{identifier:03X}", "extended": extended,
                       "logical_source": f"{source:02X}", "logical_destination": f"{dest:02X}",
                       "flags": f"{flags:02X}", "command": f"{cmdset:02X}:{command:02X}",
                       "payload_length": length, "subtype": subtype,
                       "captures_present": len(per_capture), "total_packets": len(packets),
                       "observed_packet_mask": packet_mask(packets),
                       "payload_byte_values": values,
                       "variable_payload_offsets": [i for i, v in enumerate(values) if len(v)>1],
                       "required_for_independent_control": "untested",
                       "by_capture": {name: describe(rows) for name, rows in per_capture.items()}})
    return {"captures": captures, "groups": result,
            "offset_convention": "zero-based payload offsets; masks include header and CRC",
            "header_layout": {"0": "55 sync", "1..2": "10-bit packet length, high version bits",
                              "3": "header CRC8", "4..5": "logical addresses",
                              "6..7": "uint16 little-endian sequence", "8": "flags",
                              "9..10": "command set and command", "11..length-3": "payload",
                              "length-2..length-1": "uint16 little-endian packet CRC16"},
            "crc": {"header": {"initial": "0x77", "reflected_polynomial": "0x8C", "covered_bytes": "0..2"},
                    "packet": {"initial": "0x3692", "reflected_polynomial": "0x8408", "covered_bytes": "all except final two"}},
            "limits": ["Observed constants are not guaranteed protocol constants.",
                       "Sequences and CRC bytes are always masked, even for a single packet.",
                       "Intervals and sequence steps are computed per capture and reliable segment, never across captures.",
                       "04:69 prefix layouts and 04:2F / 24:20 subtypes are separated.",
                       "Raw action labels, physical senders and physical edge times are not inferred.",
                       "No observed message is proven necessary for independent control."]}


def self_check():
    def row(segment, stamp, seq, prefix=b"\x0f\x04\x08\x19"):
        packet = b"\x55\x13\x04\x03\xe5\xbf" + seq.to_bytes(2, "little") + b"\x00\x04\x69" + prefix + b"\x00\x00\x11\x22"
        return (0x503, 0, segment, stamp, packet)
    first, second, boundary = row(0, 0, 65535), row(0, 10000, 1), row(1, 20000, 7)
    stats = describe([boundary, second, first])
    assert stats["median_interval_ms"] == 10 and stats["sequence_steps_mod_65536"] == {2: 1}
    assert identity(first) != identity(row(0, 0, 1, b"\x0f\x04\x00\x00"))
    mask = packet_mask([first[4]]).split()
    assert [mask[i] for i in (6, 7, 17, 18)] == ["??"]*4
    assert not stats["variable_payload_offsets"]
    def anchor(segment, stamp, seq):
        packet = b"\x55\x10\x04\x56\xbf\x32" + seq.to_bytes(2, "little") + b"\x20\x24\x20\x5a\x41\x01\x00\x00"
        return (0x232, 0, segment, stamp, packet)
    start, end = anchor(0, 0, 65535), anchor(0, 100000, 17)
    duplicate = (0x504, *start[1:])
    cycle = bf_cycles([end, duplicate, start])
    assert cycle["anchor_sequence_steps"] == {18: 1}
    assert len(cycle["patterns"][0]["slots"]) == 1
    assert bf_cycles([start, anchor(1, 100000, 17)])["complete_anchor_intervals"] == 0
    print("protocol catalog self-check passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if not args.logs or not args.output:
        parser.error("provide saved logs and --output")
    if len(set(path.resolve() for path in args.logs)) != len(args.logs):
        parser.error("duplicate input logs would count the same evidence twice")
    if len({path.name for path in args.logs}) != len(args.logs):
        parser.error("input logs must have distinct file names")
    report = build(args.logs)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(len(report["captures"]), "captures,", len(report["groups"]), "route/layout groups,",
          sum(capture["crc_valid_packets"] for capture in report["captures"]), "CRC-valid packets")


if __name__ == "__main__":
    main()
