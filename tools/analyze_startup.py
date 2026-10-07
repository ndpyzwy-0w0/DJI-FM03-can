"""Offline startup timelines; timestamps describe recorded frames, not physical actions."""

import argparse
import hashlib
import json
from collections import Counter, defaultdict, deque
from pathlib import Path

from analyze_protocol import reconstruct
from capture import parse_frame, summarize


def match_queries(rows, max_age_us=2_000_000):
    """Pair 00:01 packets once, by segment, reversed address and sequence."""
    pending = defaultdict(deque)
    pairs, unmatched = [], []
    for row in sorted(rows, key=lambda r: r[3]):
        identifier, extended, segment, stamp, packet = row
        if packet[9:11] != b"\x00\x01":
            continue
        key = (segment, packet[4], packet[5], packet[6:8])
        if not packet[8] & 0x80:
            pending[key].append(row)
            continue
        requests = pending[(segment, packet[5], packet[4], packet[6:8])]
        while requests and stamp - requests[0][3] > max_age_us:
            unmatched.append(requests.popleft())
        if requests:
            pairs.append((requests.popleft(), row))
        else:
            unmatched.append(row)
    unmatched.extend(row for requests in pending.values() for row in requests)
    return pairs, unmatched


def analyze(path):
    raw = path.read_bytes()
    integrity = summarize(raw)
    frames = [frame for line in raw.split(b"\n")[:-1]
              if (frame := parse_frame(line.decode(errors="replace")))]
    rows = sorted(reconstruct(raw), key=lambda r: r[3])
    origin = frames[0][0] if frames else None

    def record(row):
        identifier, extended, segment, stamp, packet = row
        return {"can_id": f"0x{identifier:03X}", "extended": extended, "segment": segment,
                "device_t_us": stamp, "relative_s": round((stamp - origin) / 1e6, 6),
                "sequence": int.from_bytes(packet[6:8], "little"),
                "packet_hex": packet.hex(" ")}

    groups = defaultdict(list)
    for row in rows:
        identifier, extended, segment, stamp, packet = row
        key = (f"0x{identifier:03X} {packet[4]:02X}>{packet[5]:02X} "
               f"flags={packet[8]:02X} cmd={packet[9]:02X}:{packet[10]:02X} "
               f"len={len(packet)-13}")
        groups[key].append(row)
    profiles = {key: {"packets": len(records), "first": record(records[0]),
                      "last": record(records[-1]),
                      "payload_counts": Counter(r[4][11:-2].hex(" ") for r in records).most_common()}
                for key, records in sorted(groups.items())}
    transitions, previous = [], {}
    for row in rows:
        identifier, extended, segment, stamp, packet = row
        payload = packet[11:-2]
        wanted = packet[9:11] == b"\x04\x69" or (
            identifier == 0x504 and packet[4:6] == b"\xbf\xe5"
            and packet[9:11] == b"\x04\x2f" and payload[:1] in (b"\x09", b"\x12", b"\x18"))
        if wanted:
            key = (identifier, extended, segment, packet[4:6], packet[8:11], payload[:1])
            if previous.get(key) != payload:
                transitions.append(record(row))
                previous[key] = payload
    pairs, unmatched = match_queries(rows)
    repeated = Counter((f[2], f[3], f[4], f[5], f[6]) for f in frames)
    frequent = []
    for (identifier, extended, rtr, dlc, data), count in repeated.most_common(3):
        matching = [f for f in frames if f[2:] == (identifier, extended, rtr, dlc, data)]
        frequent.append({"can_id": f"0x{identifier:03X}", "extended": extended,
                         "rtr": rtr, "dlc": dlc, "data_hex": data.hex(" "), "frames": count,
                         "first_s": round((matching[0][0]-origin)/1e6, 6),
                         "last_s": round((matching[-1][0]-origin)/1e6, 6)})
    status_first, status_last = integrity["status_first"], integrity["status_last"]
    counters = {key: status_last[key] - status_first[key]
                for key in ("logger_dropped", "rx_missed", "rx_overrun", "bus_errors")
                if status_first and status_last and key in status_first and key in status_last}
    return {"raw_log": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest(),
            "integrity": integrity, "counter_deltas": counters,
            "trailing_bytes": len(raw.rsplit(b"\n", 1)[-1]), "complete_frames": len(frames),
            "origin_first_can_t_us": origin,
            "first_can_minus_first_status_s": round((origin-status_first["t_us"])/1e6, 6)
            if origin is not None and status_first else None,
            "crc_valid_packets": len(rows), "profiles": profiles, "transitions": transitions,
            "version_query_pairs": [{"request": record(a), "response": record(b),
                                     "recorded_start_delay_ms": (b[3]-a[3])/1000}
                                    for a, b in pairs],
            "unmatched_version_packets": [record(row) for row in unmatched],
            "frequent_frames": frequent,
            "payload_byte_coverage": {f"0x{i:03X}": {
                "recorded_complete_frame_bytes": sum(f[5] for f in frames if f[2] == i and not f[4]),
                "crc_packet_bytes": sum(len(r[4]) for r in rows if r[0] == i)}
                for i in sorted({f[2] for f in frames})}}


def self_check():
    def row(segment, stamp, src, dst, seq, flags):
        packet = bytes([0x55, 13, 4, 0, src, dst]) + seq.to_bytes(2, "little") + bytes([flags, 0, 1, 0, 0])
        return (0x503, 0, segment, stamp, packet)
    request = row(0, 10, 0xE5, 0xBF, 65535, 0x40)
    response = row(0, 20, 0xBF, 0xE5, 65535, 0xC0)
    assert match_queries([response, request]) == ([(request, response)], [])
    assert len(match_queries([request, row(1, 20, 0xBF, 0xE5, 65535, 0xC0)])[1]) == 2
    assert len(match_queries([request, response], max_age_us=5)[1]) == 2
    duplicate = row(0, 11, 0xE5, 0xBF, 65535, 0x40)
    pairs, unmatched = match_queries([request, duplicate, response])
    assert pairs == [(request, response)] and unmatched == [duplicate]
    print("startup analyzer self-check passed")


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
        parser.error("provide logs and --output")
    report = {"captures": [analyze(path) for path in args.logs],
              "limits": ["Action labels must be confirmed separately by the user.",
                         "Time zero is the first recorded CAN frame, not physical power-on.",
                         "Version query classification follows community DUML mapping; hardware identity is unconfirmed.",
                         "Pairing repeated same-sequence queries does not prove causal one-to-one replies.",
                         "Repeated frames alone do not establish missing ACK or physical sender.",
                         "Observed startup exchanges are not proven requirements for independent motor control."]}
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    for capture in report["captures"]:
        print(Path(capture["raw_log"]).name, capture["crc_valid_packets"], "CRC packets,",
              len(capture["version_query_pairs"]), "query pairs")


if __name__ == "__main__":
    main()
