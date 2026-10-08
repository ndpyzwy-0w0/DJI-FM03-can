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


def plausible_header(data):
    """Could this frame begin a DUML packet? Returns the declared length or None.

    Used to tell a genuine packet start from a continuation frame whose payload
    happens to begin with 0x55 (the sync byte appears inside real bodies).
    A start needs a valid header CRC8; continuation ownership uses the full CRC16.
    """
    if len(data) < 4 or data[0] != 0x55 or crc(data[:3], 0x77, 0x8C) != data[3]:
        return None
    length = data[1] | ((data[2] & 3) << 8)
    return length if 13 <= length <= 1023 else None


def reconstruct_frames(raw):
    """Packet reconstruction that respects CAN frame boundaries.

    Frame order on the bus is self-contained per CAN ID: measured fragment
    patterns are e.g. 0x201>0x201>0x201 for a 19-byte 04:69 packet and
    0x232>0x232 for a 16-byte 24:20 packet, so bytes are grouped per
    (CAN ID, extended, reliable segment). A packet is closed by consuming whole
    frames (8, 8, ... bytes) until the declared length is reached, then verifying
    both CRCs. Bytes are never scanned for 0x55, which is what previously let the
    scanner emit a few packets that were never on the bus.

    Returns the frames used per packet, so callers can distinguish a complete
    packet from a truncated one instead of silently mixing them.
    """
    streams = {}
    order = []
    previous_sequence = None
    previous_status = None
    segment = 0
    stats = Counter()
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
            streams[key] = []
            order.append(key)
        streams[key].append((sequence, stamp, data))

    packets = []
    for key in order:
        identifier, extended, segment = key
        frames = streams[key]
        index = 0
        while index < len(frames):
            _seq, _stamp, data = frames[index]
            length = plausible_header(data)
            if length is None:
                stats["not_a_packet_start"] += 1
                index += 1
                continue
            buffer = bytearray()
            first_stamp = frames[index][1]
            cursor = index
            # A continuation can itself contain a valid-looking header. Only the
            # complete packet CRC decides whether it belongs to the current packet.
            while cursor < len(frames) and len(buffer) < length:
                buffer += frames[cursor][2]
                cursor += 1
            candidate = bytes(buffer[:length])
            accepted = (len(buffer) == length
                        and crc(candidate[:-2], 0x3692, 0x8408)
                        == int.from_bytes(candidate[-2:], "little"))
            if not accepted:
                # Reconsider every following frame; a truncated or corrupt start
                # must not consume the next healthy packet while seeking its tail.
                stats["truncated" if len(buffer) < length else "crc_fail"] += 1
                index += 1
                continue
            stats["complete"] += 1
            packets.append({"can_id": identifier, "extended": extended,
                            "segment": segment, "stamp": first_stamp,
                            "frames_used": cursor - index, "packet": candidate})
            index = cursor
    packets.sort(key=lambda item: item["stamp"])
    return {"packets": packets, "stats": stats, "segment_count": segment + 1,
            "note": ("Frames are grouped per (CAN ID, extended, reliable segment) and "
                     "consumed whole; packet boundaries are never found by scanning "
                     "for 0x55 inside the byte stream.")}


def reconstruct(raw):
    """Backwards-compatible generator over complete packets.

    Delegates to reconstruct_frames(), so it no longer emits packets assembled
    from misaligned bytes.
    """
    for entry in reconstruct_frames(raw)["packets"]:
        yield (entry["can_id"], entry["extended"], entry["segment"],
               entry["stamp"], entry["packet"])


def reconstruct_counting(raw):
    """Same as reconstruct() but also yields the number of frames per packet."""
    for entry in reconstruct_frames(raw)["packets"]:
        yield (entry["can_id"], entry["extended"], entry["segment"],
               entry["stamp"], entry["packet"], entry["frames_used"])


def _legacy_reconstruct(raw):
    """Previous byte-scanning version, kept only for regression comparison."""
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

    # Frame-boundary behaviour: the same 3-frame packet must reconstruct once.
    report = reconstruct_frames("".join(lines[:1] + [
        f"{102},{2},00000503,0,0,{len(packet[8:16])},{packet[8:16].hex()}\n",
        f"{103},{3},00000503,0,0,{len(packet[16:])},{packet[16:].hex()}\n"]).encode())
    assert len(report["packets"]) == 1 and report["stats"]["complete"] == 1
    assert report["packets"][0]["frames_used"] == 3
    assert report["packets"][0]["packet"] == packet

    # A frame that starts a packet but never completes is reported as truncated,
    # not silently accepted, and does not corrupt the following packet.
    truncated = (f"{100},1,00000503,0,0,8,{packet[:8].hex()}\n"
                 f"{101},2,00000503,0,0,3,{packet[8:11].hex()}\n")
    report = reconstruct_frames(truncated.encode())
    assert not report["packets"] and report["stats"]["truncated"] == 1

    # A packet that starts but never completes must be reported as truncated and
    # must not stop the following packet from being recovered.
    head = bytearray([0x55, 13, 0x04, 0x00, 0xE5, 0xBF, 0xC2, 0x97, 0x00, 0x0D, 0x00])
    head[3] = crc(head[:3], 0x77, 0x8C)
    good = bytes(head) + crc(bytes(head), 0x3692, 0x8408).to_bytes(2, "little")
    assert len(good) == 13 and plausible_header(good) == 13
    truncated = (f"{100},1,00000503,0,0,8,{good[:8].hex()}\n"          # declares 13 bytes
                 f"{101},2,00000503,0,0,4,{good[8:12].hex()}\n")       # only 12 delivered
    report = reconstruct_frames(truncated.encode())
    assert report["stats"]["truncated"] >= 1, report["stats"]
    assert not report["packets"], "an incomplete packet must not be reported as complete"

    # Truncated packet followed by a healthy one: the healthy one must survive.
    stream = truncated + (f"{200},3,00000503,0,0,8,{good[:8].hex()}\n"
                          f"{201},4,00000503,0,0,{len(good[8:])},{good[8:].hex()}\n")
    report = reconstruct_frames(stream.encode())
    assert any(entry["packet"] == good for entry in report["packets"]), report["stats"]

    # Real capture continuations can start with 55, with or without a valid
    # header CRC8. Neither case may interrupt the outer packet.
    for embedded_header in (bytes.fromhex("55 0D 04 00"), good[:8]):
        payload = b"\x00" * 5 + embedded_header
        outer = bytearray([0x55, 13 + len(payload), 0x04, 0, 0xE5, 0xBF,
                           0, 0, 0, 0x04, 0x05])
        outer[3] = crc(outer[:3], 0x77, 0x8C)
        outer += payload
        outer += crc(outer, 0x3692, 0x8408).to_bytes(2, "little")
        rows = [f"{200+i},{i+1},00000106,0,0,{len(outer[i*8:i*8+8])},"
                f"{outer[i*8:i*8+8].hex()}\n"
                for i in range((len(outer)+7)//8)]
        report = reconstruct_frames("".join(rows).encode())
        assert [e["packet"] for e in report["packets"]] == [bytes(outer)]

    # A bad CRC must not hide the next complete packet.
    corrupt = bytearray(good); corrupt[-1] ^= 1
    raw = b""; seq = 0
    for p in (corrupt, good):
        for offset in range(0, len(p), 8):
            seq += 1; chunk = p[offset:offset+8]
            raw += f"{300+seq},{seq},00000503,0,0,{len(chunk)},{chunk.hex()}\n".encode()
    report = reconstruct_frames(raw)
    assert [e["packet"] for e in report["packets"]] == [good]
    assert report["stats"]["crc_fail"] == 1
    # CRC16 alone cannot validate a damaged header CRC8.
    bad_header = bytearray(good)
    bad_header[3] ^= 1
    bad_header[-2:] = crc(bad_header[:-2], 0x3692, 0x8408).to_bytes(2, "little")
    rows = [f"{400+i},{i+1},00000503,0,0,{len(bad_header[i*8:i*8+8])},"
            f"{bad_header[i*8:i*8+8].hex()}\n" for i in range(2)]
    assert not reconstruct_frames("".join(rows).encode())["packets"]

    # Firmware drop-counter changes must separate otherwise contiguous fragments.
    fragments = [f"{500+i},{i+1},00000503,0,0,{len(packet[i*8:i*8+8])},"
                 f"{packet[i*8:i*8+8].hex()}\n" for i in range(3)]
    raw = ("# STATUS t_us=499 logger_dropped=0 rx_missed=0 rx_overrun=0\n"
           + fragments[0]
           + "# STATUS t_us=501 logger_dropped=1 rx_missed=0 rx_overrun=0\n"
           + "".join(fragments[1:]))
    assert not reconstruct_frames(raw.encode())["packets"]
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
