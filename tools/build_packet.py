"""Offline DUML packet construction and double-CRC re-verification.

Read-only: this module never opens a serial port, never imports pyserial and
contains no CAN transmit path. It builds packets into memory and compares saved
evidence against them, so the encoder used in a future active stage can be
validated against the real captures first.

CRC parameters and header layout reference:
https://github.com/o-gs/dji-firmware-tools/blob/master/comm_dat2pcap.py
Payload offsets below are zero-based, excluding the 11-byte DUML header.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

from analyze_protocol import reconstruct

HEADER_LENGTH = 11
TRAILER_LENGTH = 2
MAX_LENGTH = 1023
HEADER_CRC_INITIAL, HEADER_CRC_POLYNOMIAL = 0x77, 0x8C
PACKET_CRC_INITIAL, PACKET_CRC_POLYNOMIAL = 0x3692, 0x8408


def crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def packet_field(packet, name):
    """Extract one field from an already CRC-valid packet."""
    if len(packet) < HEADER_LENGTH + TRAILER_LENGTH:
        raise ValueError("packet is shorter than an empty DUML packet")
    length = packet[1] | ((packet[2] & 3) << 8)
    if length != len(packet):
        raise ValueError(f"packet length field {length} does not match {len(packet)} bytes")
    fields = {"source": packet[4], "destination": packet[5],
              "sequence": int.from_bytes(packet[6:8], "little"),
              "flags": packet[8], "command_set": packet[9], "command": packet[10],
              "version": packet[2] >> 2, "length": length,
              "payload": packet[HEADER_LENGTH:-TRAILER_LENGTH]}
    if name not in fields:
        raise ValueError(f"unknown field {name!r}; expected one of {sorted(fields)}")
    return fields[name]


def build_packet(source, destination, sequence, command_set, command,
                 payload=b"", flags=0, version=0x04):
    """Build one DUML packet with both CRCs computed over the final bytes.

    version is the 6-bit packet version value stored in the high bits of the byte
    at offset 2 (its low 2 bits hold the high bits of the 10-bit length). The
    value must fit 6 bits after shifting left by 2, so the observed evidence byte
    0x04 corresponds to version=1.
    """
    for name, value in (("source", source), ("destination", destination),
                        ("flags", flags), ("command_set", command_set), ("command", command)):
        if not 0 <= value <= 0xFF:
            raise ValueError(f"{name} must be one byte, got {value}")
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError(f"sequence must be uint16, got {sequence}")
    if not 0 <= version <= 0x3F:
        raise ValueError(f"version must fit 6 bits after shifting left by 2, got {version}")
    payload = bytes(payload)
    length = HEADER_LENGTH + len(payload) + TRAILER_LENGTH
    if length > MAX_LENGTH:
        raise ValueError(f"packet length {length} exceeds the 10-bit length field")
    header = bytes([0x55, length & 0xFF, version << 2 | (length >> 8) & 0x03])
    body = header[:3] + bytes([crc(header[:3], HEADER_CRC_INITIAL, HEADER_CRC_POLYNOMIAL)])
    body += bytes([source, destination]) + sequence.to_bytes(2, "little")
    body += bytes([flags, command_set, command]) + payload
    return body + crc(body, PACKET_CRC_INITIAL, PACKET_CRC_POLYNOMIAL).to_bytes(2, "little")


def fragment(packet, prefix=b"\x55\x10\x04\x56"):
    """Split a packet into CAN data frames, for checking against saved frames."""
    return [packet[index:index + 8] for index in range(0, len(packet), 8)]


def layout(source, destination, command_set, command, payload):
    return (f"S? {source:02X}>{destination:02X} cmd={command_set:02X}:{command:02X} "
            f"payload_len={len(payload)} payload={payload.hex(' ')}")


def verify(path):
    """Re-encode every CRC-valid packet of a saved log and compare byte for byte."""
    result = {"raw_log": str(path.resolve()), "packets": 0, "mismatches": [],
              "layout_counts": Counter()}
    for identifier, extended, segment, stamp, packet in reconstruct(path.read_bytes()):
        rebuilt = build_packet(
            packet[4], packet[5], int.from_bytes(packet[6:8], "little"),
            packet[9], packet[10], packet[HEADER_LENGTH:-TRAILER_LENGTH], packet[8],
            version=packet[2] >> 2)
        result["packets"] += 1
        result["layout_counts"][layout(packet[4], packet[5], packet[9], packet[10],
                                       packet[HEADER_LENGTH:-TRAILER_LENGTH])] += 1
        if rebuilt != packet:
            result["mismatches"].append({
                "can_id": f"{'E' if extended else 'S'}:0x{identifier:03X}", "segment": segment,
                "t_us": stamp, "stored": packet.hex(" "), "rebuilt": rebuilt.hex(" ")})
    result["layout_counts"] = dict(result["layout_counts"])
    result["distinct_layouts"] = len(result["layout_counts"])
    return result


def describe_packet(packet):
    """Human-readable summary of a built packet, with its CAN fragmentation."""
    return {"hex": packet.hex(" "),
            "length": len(packet),
            "source": packet[4], "destination": packet[5],
            "sequence": int.from_bytes(packet[6:8], "little"),
            "flags": packet[8], "command": f"{packet[9]:02X}:{packet[10]:02X}",
            "version": packet[2] >> 2,
            "header_crc8": packet[3],
            "packet_crc16": int.from_bytes(packet[-2:], "little"),
            "frames": [frame.hex(" ").upper() for frame in fragment(packet)]}


def build_from_spec(spec):
    """Turn CLI FIELD=VALUE strings into one packet.

    Required: source, destination, sequence, command. Optional: payload (hex),
    flags, version. command accepts either 04:69 or 0x0469/0x469.
    """
    values = {}
    for item in spec:
        if "=" not in item:
            raise ValueError(f"expected FIELD=VALUE, got {item!r}")
        key, text = item.split("=", 1)
        values[key.strip().lower()] = text.strip()
    try:
        source, destination = int(values.pop("source"), 0), int(values.pop("destination"), 0)
        sequence = int(values.pop("sequence"), 0)
        command = values.pop("command")
        payload = bytes.fromhex(values.pop("payload", "").replace(" ", ""))
        flags = int(values.pop("flags", "0"), 0)
        version = int(values.pop("version", "1"), 0)
    except KeyError as exc:
        raise ValueError(f"missing required field {exc.args[0]!r}") from None
    except ValueError as exc:
        raise ValueError(f"could not parse a field value: {exc}") from None
    if values:
        raise ValueError(f"unknown field(s): {sorted(values)}")
    if ":" in command:
        left, right = command.split(":", 1)
        command_set, command = int(left, 16), int(right, 16)
    else:
        combined = int(command, 0)
        command_set, command = (combined >> 8) & 0xFF, combined & 0xFF
    return build_packet(source, destination, sequence, command_set, command,
                        payload, flags, version)


def self_check():
    sample = bytes.fromhex("55 13 04 03 E5 BF C2 97 00 04 69 0F 04 08 19 00 00 06 16")
    assert build_packet(0xE5, 0xBF, 0x97C2, 0x04, 0x69,
                        bytes.fromhex("0F 04 08 19 00 00"), version=1) == sample
    # Second real packet, taken from the saved idle capture summary.
    second = bytes.fromhex("55 10 04 56 04 C9 D9 0D 00 04 10 7A 01 01 43 93")
    assert build_packet(0x04, 0xC9, 0x0DD9, 0x04, 0x10, bytes.fromhex("7A 01 01"), version=1) == second
    assert packet_field(sample, "version") == 1 and packet_field(sample, "length") == 0x13
    assert packet_field(sample, "sequence") == 0x97C2
    assert packet_field(sample, "payload") == bytes.fromhex("0F 04 08 19 00 00")
    assert packet_field(sample, "command_set") == 4 and packet_field(sample, "command") == 0x69
    # Both CRCs must be recomputed: changing any covered byte invalidates the stored trailer.
    mutated = bytearray(sample)
    mutated[4], mutated[6] = 0x00, 0x01
    assert bytes(mutated) != sample and mutated[3] == sample[3]
    changed = build_packet(0x00, 0xBF, 0x0001, 0x04, 0x69, bytes.fromhex("0F 04 08 19 00 00"))
    assert changed[3] != sample[3] and changed[-2:] != sample[-2:]
    assert changed[:4] == build_packet(0x00, 0xBF, 0x0001, 0x04, 0x69,
                                       bytes.fromhex("0F 04 08 19 00 00"))[:4]
    # Length, version and empty payload handling.
    empty = build_packet(0x04, 0xC9, 0, 0x04, 0x10)
    assert len(empty) == 13 and empty[1] == 13 and packet_field(empty, "payload") == b""
    # Length field: 10 bits, so 1023 (payload 1010) is the largest legal packet.
    biggest = build_packet(0xE5, 0xBF, 1, 4, 0x69, b"\x00" * 1010)
    assert len(biggest) == 1023 and packet_field(biggest, "length") == 1023
    assert biggest[1] == 0xFF and biggest[2] & 0x03 == 3
    base = {"source": 1, "destination": 1, "sequence": 0, "command_set": 1, "command": 1}
    for bad in ({"source": 256}, {"destination": -1}, {"sequence": 0x10000},
                {"version": 64}, {"command": 0x100}, {"flags": 0x100}):
        try:
            build_packet(**{**base, **bad})
        except ValueError:
            pass
        else:
            raise AssertionError(f"{bad} should be rejected")
    try:
        build_packet(1, 1, 0, 1, 1, b"\x00" * 1011)
    except ValueError:
        pass
    else:
        raise AssertionError("oversized payload should be rejected")
    # Fragmentation matches the frame-level view used by the captures.
    assert [len(x) for x in fragment(sample)] == [8, 8, 3]
    assert b"".join(fragment(sample)) == sample
    try:
        packet_field(sample, "missing")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown field should be rejected")
    print("packet builder self-check passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--build", nargs="+", metavar="FIELD=VALUE",
                        help="build one packet instead of verifying logs, e.g. "
                             "--build source=0xE5 destination=0xBF sequence=0x1234 "
                             "command=04:69 payload=0F0408190000")
    parser.add_argument("--output", type=Path,
                        help="write the verification report here; defaults to stdout only")
    parser.add_argument("--limit", type=int, default=20,
                        help="maximum number of mismatch samples to keep (default 20)")
    parser.add_argument("--show", type=int, default=0,
                        help="print the N most common reconstructed layouts")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if args.build:
        if args.logs:
            parser.error("--build builds a single packet and cannot be combined with log files")
        print(json.dumps(describe_packet(build_from_spec(args.build)), ensure_ascii=False, indent=2))
        return
    if not args.logs:
        parser.error("provide one or more saved .log files")
    reports = []
    failed = False
    for path in args.logs:
        report = verify(path)
        report["mismatches"] = report["mismatches"][:args.limit]
        reports.append(report)
        print(f"{path.name}: {report['packets']} packets re-encoded, "
              f"{len(report['mismatches'])} mismatches, "
              f"{report['distinct_layouts']} distinct layouts")
        if args.show:
            for name, count in Counter(report["layout_counts"]).most_common(args.show):
                print(f"  {count:6d}  {name}")
        # A mismatch means the rebuild changed bytes a saved log already proved valid.
        failed = failed or bool(report["mismatches"])
    # Coverage guard: every packet a summary reports must be re-encoded here, so a
    # silenty skipped packet family cannot look like a clean pass.
    covered = sum(report["packets"] for report in reports)
    reported = 0
    for path in args.logs:
        summary = path.with_suffix(".analysis.json")
        if summary.is_file():
            reported += json.loads(summary.read_text(encoding="utf-8"))["crc_valid_packets"]
    if args.logs and any(path.with_suffix(".analysis.json").is_file() for path in args.logs):
        print(f"coverage: {covered} re-encoded vs {reported} packets in saved analysis summaries")
        if covered != reported:
            print("ERROR: re-encoded packet count does not match the saved summaries")
            failed = True
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        document = {"tool": "tools/build_packet.py", "no_serial_io": True,
                    "no_transmit_path": True,
                    "notes": ["Each packet is rebuilt only from its own fields; sequence and both CRCs are recomputed.",
                              "A byte-for-byte match proves the encoder reproduces this evidence, not that any packet is safe or necessary to send.",
                              "This tool never opens a serial port and never transmits CAN."],
                    "reports": reports}
        args.output.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"wrote {args.output}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
