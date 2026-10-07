"""Offline DUML-compatible packet reconstruction; never opens a serial port.

CRC parameters and header fields reference:
https://github.com/o-gs/dji-firmware-tools/blob/master/comm_dat2pcap.py
Payload offsets below are zero-based, excluding the 11-byte DUML header.
"""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import json
from pathlib import Path
import re
from statistics import median

from capture import parse_frame, summarize


def crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def scan_packets(data):
    offset = 0
    while offset + 4 <= len(data):
        if data[offset] != 0x55 or crc(data[offset:offset + 3], 0x77, 0x8C) != data[offset + 3]:
            offset += 1
            continue
        length = data[offset + 1] | ((data[offset + 2] & 3) << 8)
        if not 13 <= length <= 1023 or offset + length > len(data):
            offset += 1
            continue
        packet = bytes(data[offset:offset + length])
        if crc(packet[:-2], 0x3692, 0x8408) != int.from_bytes(packet[-2:], "little"):
            offset += 1
            continue
        yield offset, packet
        offset += length


def reconstruct(raw):
    streams = {}
    previous_sequence = None
    previous_status = None
    segment = 0
    for line in raw.split(b"\n")[:-1]:
        text = line.decode("ascii", errors="replace")
        if text.startswith("# STATUS "):
            status = {key: int(value) for key, value in re.findall(r"(\w+)=(\d+)", text)}
            if previous_status and (status.get("t_us", 0) < previous_status.get("t_us", 0)
                                    or any(status.get(key, 0) != previous_status.get(key, 0)
                                           for key in ("rx_missed", "rx_overrun", "logger_dropped"))):
                segment += 1
            previous_status = status
            continue
        frame = parse_frame(text)
        if frame is None:
            continue
        stamp, sequence, identifier, extended, rtr, _, data = frame
        if previous_sequence is not None and sequence != previous_sequence + 1:
            segment += 1
        previous_sequence = sequence
        if rtr or not data:
            continue
        key = (identifier, extended, segment)
        if key not in streams:
            streams[key] = (bytearray(), [], [])
        stream, offsets, stamps = streams[key]
        offsets.append(len(stream))
        stamps.append(stamp)
        stream.extend(data)
    for (identifier, extended, segment), (data, offsets, stamps) in streams.items():
        for offset, packet in scan_packets(data):
            stamp = stamps[bisect_right(offsets, offset) - 1]
            yield identifier, extended, segment, stamp, packet


def analyze(path):
    raw = path.read_bytes()
    summary = summarize(raw)
    groups = defaultdict(list)
    for identifier, extended, segment, stamp, packet in reconstruct(raw):
        payload = packet[11:-2]
        key = (f"{'E' if extended else 'S'}:0x{identifier:03X} "
               f"{packet[4]:02X}>{packet[5]:02X} flags={packet[8]:02X} "
               f"cmd={packet[9]:02X}:{packet[10]:02X} payload_len={len(payload)}")
        groups[key].append((stamp, segment, payload, packet))
    details = {}
    for key, records in sorted(groups.items()):
        payload_counts = Counter(payload.hex(" ") for _, _, payload, _ in records)
        byte_counts = [Counter(payload[index] for _, _, payload, _ in records)
                       for index in range(len(records[0][2]))]
        intervals = [(b[0] - a[0]) / 1000 for a, b in zip(records, records[1:])
                     if a[1] == b[1] and b[0] > a[0]]
        details[key] = {"packets": len(records), "unique_payloads": len(payload_counts),
                        "variable_payload_offsets": [i for i, values in enumerate(byte_counts) if len(values) > 1],
                        "byte_values": [sorted(values) for values in byte_counts],
                        "most_common_payloads": payload_counts.most_common(5),
                        "median_interval_ms": round(median(intervals), 3) if intervals else None,
                        "sample_packet": records[0][3].hex(" ")}
    return {"raw_log": str(path.resolve()), "crc_valid_packets": sum(x["packets"] for x in details.values()),
            "integrity": summary, "groups": details,
            "notes": ["CAN payload streams are separate by ID and extended flag.",
                      "Reassembly boundaries reset at observed sequence gaps, reboots or changed firmware drop counters.",
                      "Timestamps are driver-dequeue timestamps, not hardware frame-edge times.",
                      "Variable payload bytes are candidates, not identified motor control fields.",
                      "Incomplete final UART line is excluded from packet reconstruction."]}


def compare(reference, other):
    rows = []
    for key in sorted(set(reference["groups"]) & set(other["groups"])):
        left, right = reference["groups"][key], other["groups"][key]
        rows.append({"group": key,
                     "idle_unique_payloads": left["unique_payloads"],
                     "other_unique_payloads": right["unique_payloads"],
                     "variable_only_in_other": sorted(set(right["variable_payload_offsets"]) - set(left["variable_payload_offsets"])),
                     "idle_variable_offsets": left["variable_payload_offsets"],
                     "other_variable_offsets": right["variable_payload_offsets"]})
    return {"reference": reference["raw_log"], "other": other["raw_log"],
            "offset_convention": "zero-based DUML payload offset, excluding header and CRC", "groups": rows}


def self_check():
    packet = bytes.fromhex("55 13 04 03 E5 BF C2 97 00 04 69 0F 04 08 19 00 00 06 16")
    assert crc(packet[:3], 0x77, 0x8C) == packet[3]
    assert crc(packet[:-2], 0x3692, 0x8408) == int.from_bytes(packet[-2:], "little")
    assert list(scan_packets(b"noise" + packet + packet[:-3])) == [(5, packet)]
    bad = bytearray(packet)
    bad[11] ^= 1
    assert list(scan_packets(bad)) == []
    lines = [f"{100 + i},{i + 1},00000503,0,0,{len(data)},{data.hex()}\n"
             for i, data in enumerate((packet[:8], packet[8:16], packet[16:]))]
    assert [p[-1] for p in reconstruct("".join(lines).encode())] == [packet]
    lines[1] = lines[1].replace(",2,", ",4,", 1)
    assert list(reconstruct("".join(lines).encode())) == []
    print("protocol analyzer self-check passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if not args.logs:
        parser.error("provide one or more .log files; the first is the reference/idle capture")
    reports = []
    for path in args.logs:
        report = analyze(path)
        target = path.with_suffix(".analysis.json")
        target.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        reports.append(report)
        print(f"{path.name}: {report['integrity']['frames']} CAN frames, {report['crc_valid_packets']} CRC-valid packets")
    for index, report in enumerate(reports[1:], 1):
        comparison = compare(reports[0], report)
        target = args.logs[index].with_suffix(".vs-idle.json")
        target.write_text(json.dumps(comparison, indent=2, ensure_ascii=False), encoding="utf-8")
        for row in comparison["groups"]:
            if row["variable_only_in_other"]:
                print(json.dumps(row, ensure_ascii=False))


if __name__ == "__main__":
    main()
