"""Export offline direction evidence for the 04:69 candidate; no serial access."""
import argparse
import csv
from hashlib import sha256
from html import escape
import json
from pathlib import Path

from analyze_protocol import reconstruct
from capture import summarize


def candidate_value(payload):
    if len(payload) != 6 or payload[:4] != bytes.fromhex("0F040819"):
        raise ValueError("Unexpected candidate payload layout")
    return int.from_bytes(payload[4:6], "little", signed=True)


def extract(path, label):
    raw = path.read_bytes()
    integrity = summarize(raw)
    first = integrity["first_t_us"]
    rows = []
    by_id = {0x503: [], 0x201: []}
    for identifier, extended, segment, stamp, packet in reconstruct(raw):
        if (extended or identifier not in by_id or packet[4] != 0xE5
                or packet[5] != {0x503: 0xBF, 0x201: 0x52}[identifier]
                or packet[8:11] != bytes.fromhex("000469") or len(packet) != 19):
            continue
        payload = packet[11:-2]
        value = candidate_value(payload)
        row = {"capture": label, "can_id": f"0x{identifier:03X}",
               "segment": segment, "device_t_us": stamp,
               "relative_s": round((stamp - first) / 1e6, 6),
               "duml_sequence": int.from_bytes(packet[6:8], "little"),
               "value_int16_le": value, "payload_hex": payload.hex(" ").upper()}
        rows.append(row)
        by_id[identifier].append(row)
    if not by_id[0x503]:
        raise ValueError(f"No matching CRC-valid 04:69 packets in {path}")
    values = [row["value_int16_le"] for row in by_id[0x503]]
    nonzero = [row for row in by_id[0x503] if row["value_int16_le"]]
    gui = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
    stats = {"label": label, "raw_log": str(path.resolve()), "sha256": sha256(raw).hexdigest(),
             "elapsed_s": gui.get("elapsed"), "can_frames": integrity["frames"],
             "packets_0x503": len(values), "packets_0x201": len(by_id[0x201]),
             "minimum": min(values), "maximum": max(values),
             "negative_packets": sum(value < 0 for value in values),
             "positive_packets": sum(value > 0 for value in values),
             "zero_packets": sum(value == 0 for value in values),
             "first_value": values[0], "last_value": values[-1],
             "nonzero_first_s": nonzero[0]["relative_s"] if nonzero else None,
             "nonzero_last_s": nonzero[-1]["relative_s"] if nonzero else None,
             "two_id_payload_sequences_equal": [row["payload_hex"] for row in by_id[0x503]] == [row["payload_hex"] for row in by_id[0x201]],
             "sequence_gaps": integrity["sequence_gaps"], "sequence_resets": integrity["sequence_resets"],
             "trailing_bytes": gui.get("trailing_bytes"), "status_first": integrity["status_first"],
             "status_last": integrity["status_last"], "counter_deltas": gui.get("counter_deltas"),
             "capture_error": gui.get("error", "")}
    return stats, rows, by_id[0x503]


def svg_plot(captures):
    width, height = 1080, 780
    left, right, panel_height = 95, 1020, 145
    max_time = max(float(stats["elapsed_s"]) for stats, _, _ in captures)
    bound = max(350, max(abs(row["value_int16_le"]) for _, _, series in captures for row in series))
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="#ffffff"/>',
             '<g font-family="Segoe UI,Arial,sans-serif" fill="#243530">',
             '<text x="40" y="38" font-size="22" font-weight="600">FM03: direction-associated field</text>',
             '<text x="40" y="63" font-size="13">CAN 0x503 / DUML 04:69 / payload bytes 4-5 (zero-based) / signed little-endian 16-bit</text>']
    colors = ("#8c9892", "#13745a", "#386da5")
    titles = ("Idle", "Slow forward (user label)", "Slow reverse (user label)")
    for index, (stats, _, series) in enumerate(captures):
        top = 125 + index * 207
        bottom = top + panel_height
        x = lambda t: left + t / max_time * (right - left)
        y = lambda value: top + (bound - value) / (2 * bound) * panel_height
        parts.append(f'<text x="{left}" y="{top-21}" font-size="15" font-weight="600">{titles[index]}: {stats["minimum"]} to {stats["maximum"]}</text>')
        parts.append(f'<text x="{left}" y="{top-5}" font-size="10" fill="#6e7d76">{escape(Path(stats["raw_log"]).name)}</text>')
        for value in (-bound, 0, bound):
            py = y(value)
            parts.append(f'<line x1="{left}" y1="{py:.2f}" x2="{right}" y2="{py:.2f}" stroke="#d9e2dc"/>')
            parts.append(f'<text x="{left-12}" y="{py+4:.2f}" font-size="11" text-anchor="end">{value}</text>')
        for tick in range(0, int(max_time) + 1, 5):
            px = x(tick)
            parts.append(f'<line x1="{px:.2f}" y1="{top}" x2="{px:.2f}" y2="{bottom}" stroke="#edf1ee"/>')
            parts.append(f'<text x="{px:.2f}" y="{bottom+17}" font-size="11" text-anchor="middle">{tick}</text>')
        # Break the trace at reconstruction boundaries rather than joining gaps.
        segment_rows = {}
        for row in series:
            segment_rows.setdefault(row["segment"], []).append(row)
        for rows in segment_rows.values():
            points = " ".join(f'{x(row["relative_s"]):.2f},{y(row["value_int16_le"]):.2f}' for row in rows)
            parts.append(f'<polyline points="{points}" fill="none" stroke="{colors[index]}" stroke-width="1.4"/>')
    parts.extend(['<text x="22" y="410" font-size="12" text-anchor="middle" transform="rotate(-90 22 410)">Field value (raw units; physical unit unknown)</text>',
                  '<text x="555" y="737" font-size="12" text-anchor="middle">Seconds since first captured CAN frame (driver-dequeue time)</text>',
                  '<text x="40" y="768" font-size="11" fill="#6e7d76">CRC-valid packets only. Sign association is observed; speed/increment semantics and physical endpoints remain unverified.</text>', '</g></svg>'])
    return "\n".join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path, help="idle, slow_forward, slow_reverse, in this order")
    parser.add_argument("--output", type=Path, default=Path("reports/direction_validation"))
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        assert candidate_value(bytes.fromhex("0F0408190000")) == 0
        assert candidate_value(bytes.fromhex("0F0408193DFF")) == -195
        assert candidate_value(bytes.fromhex("0F0408193301")) == 307
        try:
            candidate_value(bytes.fromhex("0F04081900"))
        except ValueError:
            pass
        else:
            raise AssertionError("truncated payload accepted")
        print("direction analyzer self-check passed")
        return
    if len(args.logs) != 3:
        parser.error("provide exactly three logs: idle, slow_forward, slow_reverse")
    captures = [extract(path, label) for path, label in zip(args.logs, ("idle", "slow_forward", "slow_reverse"))]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {"field": "DUML 04:69 payload[4:6], int16 little-endian",
              "payload_layout": "0F 04 08 19 LL HH", "captures": [item[0] for item in captures],
              "limitations": ["Direction labels are supplied by the user; physical clockwise/counterclockwise mapping is not recorded.",
                              "Nonzero times identify this field, not independently marked physical motion times.",
                              "Speed, increment, scaling, actual sender/receiver and active-control behavior are not established."]}
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    with args.output.with_suffix(".csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(captures[0][1][0]))
        writer.writeheader()
        for _, rows, _ in captures:
            writer.writerows(rows)
    args.output.with_suffix(".svg").write_text(svg_plot(captures), encoding="utf-8")
    for stats, _, _ in captures:
        print(json.dumps({key: stats[key] for key in ("label", "can_frames", "minimum", "maximum", "negative_packets", "positive_packets", "zero_packets", "two_id_payload_sequences_equal")}))


if __name__ == "__main__":
    main()
