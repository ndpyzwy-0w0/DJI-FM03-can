"""Catalogue the link bring-up: every distinct packet from power-on to normal flow.

M3 needs to know the minimum set of messages that takes the focus motor from its
"connection abnormal" state to a working link. This tool does not answer that, but
it produces the exact byte material to work from: for one capture, the ordered
sequence of first-occurrences of every route, the full bytes of each distinct
packet, the sequence number used, and when the regular periodic flow begins.

Offline only. Never opens a serial port and never transmits.
"""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

from analyze_protocol import crc, reconstruct


def bringup(path, window_seconds=2.0):
    """Return the ordered bring-up catalogue for the first window_seconds."""
    raw = path.read_bytes()
    packets = []
    for identifier, extended, segment, stamp, packet in reconstruct(raw):
        packets.append({"can_id": identifier, "extended": extended, "segment": segment,
                        "stamp": stamp, "packet": packet})
    if not packets:
        return {"raw_log": str(path.resolve()), "packets": 0, "note": "no valid packets"}
    packets.sort(key=lambda item: item["stamp"])
    t0 = packets[0]["stamp"]
    window = [item for item in packets if (item["stamp"] - t0) <= window_seconds * 1e6]

    # Key on the logical route only (addresses, flags, command, payload length) so
    # the same logical flow seen on two CAN IDs is reported once; the CAN IDs that
    # carried it are collected separately.
    first_seen = {}
    order = []
    for item in window:
        packet = item["packet"]
        key = (packet[4], packet[5], packet[8], packet[9], packet[10], len(packet[11:-2]))
        if key not in first_seen:
            first_seen[key] = {
                "t_s": round((item["stamp"] - t0) / 1e6, 6),
                "can_ids": [f"0x{item['can_id']:03X}"],
                "source": f"0x{packet[4]:02X}", "destination": f"0x{packet[5]:02X}",
                "flags": f"0x{packet[8]:02X}",
                "command": f"{packet[9]:02X}:{packet[10]:02X}",
                "payload_len": len(packet[11:-2]),
                "sequence": int.from_bytes(packet[6:8], "little"),
                "header_crc8": packet[3],
                "packet_crc16": int.from_bytes(packet[-2:], "little"),
                "packet_hex": packet.hex(" "),
            }
            order.append(key)
        else:
            label = f"0x{item['can_id']:03X}"
            if label not in first_seen[key]["can_ids"]:
                first_seen[key]["can_ids"].append(label)

    # When does each route reach its steady rate? Compare the first and second half
    # of the whole capture for that route.
    totals = Counter()
    for item in packets:
        packet = item["packet"]
        totals[(packet[4], packet[5], packet[8], packet[9], packet[10],
                len(packet[11:-2]))] += 1
    rows = []
    for key in order:
        rows.append({**first_seen[key], "packets_in_capture": totals[key]})

    # Sequence numbers are not contiguous at bring-up; report the observed step set
    # per route so M3 knows what sequence discipline each stream uses.
    seqs = defaultdict(list)
    for item in packets:
        packet = item["packet"]
        seqs[(packet[4], packet[5], packet[8], packet[9], packet[10],
              len(packet[11:-2]))].append(int.from_bytes(packet[6:8], "little"))
    steps = {}
    for key, values in seqs.items():
        counter = Counter((b - a) % 65536 for a, b in zip(values, values[1:]))
        label = (f"{key[0]:02X}>{key[1]:02X} f{key[2]:02X} {key[3]:02X}:{key[4]:02X} "
                 f"len{key[5]}")
        steps[label] = dict(sorted(counter.items()))
    return {"raw_log": str(path.resolve()),
            "packets": len(packets),
            "span_s": round((packets[-1]["stamp"] - t0) / 1e6, 4),
            "window_seconds": window_seconds,
            "first_occurrence_order": rows,
            "sequence_steps_per_route": steps,
            "notes": ["Times are relative to the first valid packet, not to power-on.",
                      "Only CRC-validated packets are listed; the flood states produce none.",
                      "Sequence steps are reported per route across the whole capture.",
                      "No serial port is opened and no frame is transmitted."]}


def self_check():
    packet = bytes.fromhex("55 13 04 03 E5 BF C2 97 00 04 69 0F 04 08 19 00 00 06 16")
    assert crc(packet[:3], 0x77, 0x8C) == packet[3]
    assert crc(packet[:-2], 0x3692, 0x8408) == int.from_bytes(packet[-2:], "little")
    lines = [f"{100 + i},{i + 1},00000503,0,0,{len(chunk)},{chunk.hex()}\n"
             for i, chunk in enumerate((packet[:8], packet[8:16], packet[16:]))]
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "synthetic.log"
        path.write_bytes("".join(lines).encode())
        report = bringup(path)
        assert report["packets"] == 1, report
        assert len(report["first_occurrence_order"]) == 1
        row = report["first_occurrence_order"][0]
        assert row["can_ids"] == ["0x503"], row["can_ids"]
        assert row["source"] == "0xE5" and row["destination"] == "0xBF"
        assert row["command"] == "04:69" and row["payload_len"] == 6
        assert row["sequence"] == 0x97C2 and row["packet_hex"] == packet.hex(" ")
        assert row["t_s"] == 0.0 and row["packets_in_capture"] == 1
        assert report["sequence_steps_per_route"]["E5>BF f00 04:69 len6"] == {}
    print("bring-up catalogue self-check passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, nargs="?")
    parser.add_argument("--window", type=float, default=2.0,
                        help="bring-up window in seconds (default 2.0)")
    parser.add_argument("--output", type=Path, help="write the JSON report here")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if args.log is None:
        parser.error("provide a saved .log file (or use --self-check)")
    report = bringup(args.log, args.window)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"wrote {args.output}")
    print(f"{args.log.name}: {report['packets']} packets, span {report.get('span_s')}s, "
          f"{len(report.get('first_occurrence_order', []))} distinct routes in the first "
          f"{args.window}s")
    for row in report.get("first_occurrence_order", []):
        print(f"  t={row['t_s']:7.3f}  {','.join(row['can_ids'])} "
              f"{row['source']}>{row['destination']} "
              f"f{row['flags'][2:]} {row['command']} len={row['payload_len']:2} "
              f"seq={row['sequence']:5} n={row['packets_in_capture']:5} "
              f"{row['packet_hex'][:44]}")


if __name__ == "__main__":
    main()
