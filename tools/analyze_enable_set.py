"""M3 groundwork: which messages exist only to bring the link up?

A message that runs at a steady rate for the whole capture is business traffic, not
an enable step. A message that appears only during bring-up and then never again (or
only a handful of times) is an enable-handshake candidate. This separates the two,
offline, from a power-on capture.

For every route it reports: first appearance, how many packets fall in the first
second vs the rest, the steady-state rate in the last two seconds, and the verdict.

Offline only; no serial port, no transmission.
"""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

from analyze_protocol import reconstruct


def classify(path, bringup_seconds=1.0, steady_window=2.0):
    raw = path.read_bytes()
    packets = []
    for identifier, extended, segment, stamp, packet in reconstruct(raw):
        packets.append((stamp, identifier, packet))
    if not packets:
        return {"raw_log": str(path.resolve()), "packets": 0}
    packets.sort(key=lambda row: row[0])
    t0, t_end = packets[0][0], packets[-1][0]
    span = (t_end - t0) / 1e6

    routes = defaultdict(lambda: {"first": None, "bringup": 0, "rest": 0,
                                  "steady": 0, "total": 0, "can_ids": Counter(),
                                  "payloads": Counter()})
    cutoff = t0 + bringup_seconds * 1e6
    steady_start = t_end - steady_window * 1e6
    for stamp, identifier, packet in packets:
        key = (packet[4], packet[5], packet[8], packet[9], packet[10], len(packet[11:-2]))
        entry = routes[key]
        if entry["first"] is None:
            entry["first"] = stamp
        entry["total"] += 1
        entry["can_ids"][f"0x{identifier:03X}"] += 1
        entry["payloads"][packet[11:-2].hex(" ")] += 1
        if stamp <= cutoff:
            entry["bringup"] += 1
        else:
            entry["rest"] += 1
        if stamp >= steady_start:
            entry["steady"] += 1

    rows = []
    for key, entry in routes.items():
        src, dst, flags, cs, cd, plen = key
        steady_rate = entry["steady"] / max(steady_window, 1e-9)
        label = (f"{src:02X}>{dst:02X} f{flags:02X} {cs:02X}:{cd:02X} len{plen}")
        if entry["steady"] == 0:
            verdict = "BRING-UP ONLY (never in the last window)"
        elif entry["steady"] <= 3:
            verdict = "rare at steady state"
        else:
            verdict = "steady traffic"
        rows.append({
            "route": label,
            "first_s": round((entry["first"] - t0) / 1e6, 6),
            "can_ids": sorted(entry["can_ids"]),
            "packets_total": entry["total"],
            "packets_first_second": entry["bringup"],
            "packets_after": entry["rest"],
            "packets_last_window": entry["steady"],
            "steady_rate_per_s": round(steady_rate, 3),
            "distinct_payloads": len(entry["payloads"]),
            "top_payloads": [p for p, _c in entry["payloads"].most_common(3)],
            "verdict": verdict,
        })
    rows.sort(key=lambda row: (row["first_s"], row["route"]))
    return {"raw_log": str(path.resolve()), "packets": len(packets),
            "span_s": round(span, 4), "bringup_seconds": bringup_seconds,
            "steady_window_s": steady_window, "routes": rows,
            "notes": ["Times are relative to the first valid packet, not power-on.",
                      "'BRING-UP ONLY' means the route produced no packet in the final window.",
                      "A steady periodic route is business traffic; a one-shot route is an "
                      "enable/handshake candidate.",
                      "No serial port is opened and no frame is transmitted."]}


def self_check():
    from analyze_protocol import crc
    # Two routes: one periodic throughout, one that appears once at the start.
    head = bytearray([0x55, 13, 0x04, 0x00, 0xE5, 0xBF, 0x00, 0x00, 0x00, 0x0D, 0x00])
    head[3] = crc(head[:3], 0x77, 0x8C)
    periodic = bytes(head) + crc(bytes(head), 0x3692, 0x8408).to_bytes(2, "little")
    head2 = bytearray(head)
    head2[9], head2[10] = 0x00, 0x01
    head2[3] = crc(head2[:3], 0x77, 0x8C)
    oneshot = bytes(head2) + crc(bytes(head2), 0x3692, 0x8408).to_bytes(2, "little")

    lines = []
    stamp = 1_000_000
    sequence = 0
    # One-shot first, then the periodic route repeated with 100 ms gaps.
    sequence += 1
    lines.append(f"{stamp},{sequence},00000503,0,0,8,{oneshot[:8].hex()}\n")
    lines.append(f"{stamp + 1},{sequence + 1},00000503,0,0,{len(oneshot[8:])},{oneshot[8:].hex()}\n")
    for index in range(20):
        stamp += 100_000
        sequence += 1
        lines.append(f"{stamp},{sequence},00000503,0,0,8,{periodic[:8].hex()}\n")
        sequence += 1
        lines.append(f"{stamp + 1},{sequence},00000503,0,0,{len(periodic[8:])},{periodic[8:].hex()}\n")
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "synthetic.log"
        path.write_text("".join(lines), encoding="ascii")
        report = classify(path, bringup_seconds=0.5, steady_window=1.0)
        assert report["packets"] == 21, report["packets"]
        by_route = {row["route"]: row for row in report["routes"]}
        one = by_route["E5>BF f00 00:01 len0"]
        periodic_row = by_route["E5>BF f00 0D:00 len0"]
        assert one["verdict"].startswith("BRING-UP ONLY"), one
        assert one["distinct_payloads"] == 1 and one["packets_total"] == 1
        assert periodic_row["verdict"] == "steady traffic", periodic_row
        assert periodic_row["packets_total"] == 20
        assert periodic_row["steady_rate_per_s"] > 1
        # The one-shot must be reported before the periodic route by first appearance.
        assert report["routes"][0]["route"] == "E5>BF f00 00:01 len0"
    print("bring-up classifier self-check passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, nargs="?")
    parser.add_argument("--bringup-seconds", type=float, default=1.0)
    parser.add_argument("--steady-window", type=float, default=2.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if args.log is None:
        parser.error("provide a saved .log file (or use --self-check)")
    report = classify(args.log, args.bringup_seconds, args.steady_window)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"wrote {args.output}")
    print(f"{args.log.name}: {report['packets']} packets over {report['span_s']}s")
    print(f"  {'route':30} {'first':>8} {'1st-s':>7} {'after':>7} {'last':>6} {'rate/s':>9}  verdict")
    for row in report["routes"]:
        print(f"  {row['route']:30} {row['first_s']:8.3f} {row['packets_first_second']:7} "
              f"{row['packets_after']:7} {row['packets_last_window']:6} "
              f"{row['steady_rate_per_s']:9.2f}  {row['verdict']}")


if __name__ == "__main__":
    main()
