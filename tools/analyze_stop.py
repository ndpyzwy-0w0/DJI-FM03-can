"""Offline single-flick timeline. Field timestamps are not physical stop times."""
import argparse
import csv
import json
from collections import defaultdict
from html import escape
from pathlib import Path
from statistics import median

from analyze_direction import extract
from analyze_protocol import analyze, compare, reconstruct
from capture import summarize


def runs(series):
    result = []
    for row in series:
        kind = "nonzero" if row["value_int16_le"] else "zero"
        if not result or (result[-1]["kind"], result[-1]["segment"]) != (kind, row["segment"]):
            result.append({"kind": kind, "segment": row["segment"], "first_s": row["relative_s"],
                           "last_s": row["relative_s"], "packets": 0})
        result[-1]["last_s"] = row["relative_s"]
        result[-1]["packets"] += 1
    return result


def timing(series):
    intervals = [round(b["relative_s"] - a["relative_s"], 6)
                 for a, b in zip(series, series[1:]) if a["segment"] == b["segment"]]
    blocks = runs(series)
    # A final zero run is an observed field state, not a motor-stop detector.
    final = blocks[-1] if blocks and blocks[-1]["kind"] == "zero" else None
    return {"runs": blocks, "final_zero_first_s": final["first_s"] if final else None,
            "final_zero_observed_s": round(final["last_s"] - final["first_s"], 6) if final else None,
            "median_interval_s": median(intervals) if intervals else None,
            "maximum_interval_s": max(intervals) if intervals else None}


def plot(series, source="Captured CAN log"):
    end = max(1, max(row["relative_s"] for row in series))
    bound = max(100, max(abs(row["value_int16_le"]) for row in series))
    x = lambda t: 90 + t / end * 900
    y = lambda v: 115 + (bound - v) / (2 * bound) * 225
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="450" viewBox="0 0 1080 450">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<g font-family="Segoe UI,Arial,sans-serif" fill="#243530">',
             '<text x="40" y="35" font-size="22">FM03: 04:69 candidate field timeline</text>',
             f'<text x="40" y="60" font-size="12">Source: {escape(source)} / CAN 0x503 / CRC-valid packets</text>',
             '<text x="40" y="81" font-size="12">Physical dial and motor stop times: not independently marked</text>']
    for value in (-bound, 0, bound):
        parts.append(f'<path d="M90 {y(value):.2f} H990" stroke="#dddddd"/><text x="80" y="{y(value)+4:.2f}" text-anchor="end" font-size="12">{value}</text>')
    for index in range(7):
        t = end * index / 6
        parts.append(f'<text x="{x(t):.2f}" y="365" text-anchor="middle" font-size="12">{t:.1f}</text>')
    segments = defaultdict(list)
    for row in series:
        segments[row["segment"]].append(row)
    for rows in segments.values():
        points = " ".join(f'{x(r["relative_s"]):.2f},{y(r["value_int16_le"]):.2f}' for r in rows)
        parts.append(f'<polyline points="{points}" fill="none" stroke="#13745a" stroke-width="1.6"/>')
    final = timing(series)["final_zero_first_s"]
    if final is not None:
        parts.append(f'<path d="M{x(final):.2f} 100 V345" stroke="#666666" stroke-dasharray="4 4"/><text x="{x(final)+8:.2f}" y="105" font-size="12">Final zero: {final:.6f} s</text>')
    parts += ['<text x="540" y="395" text-anchor="middle" font-size="12">Seconds since first captured CAN frame (driver-dequeue time)</text>',
              '<text x="22" y="225" transform="rotate(-90 22 225)" text-anchor="middle" font-size="12">Signed int16 LE value (physical unit unknown)</text>',
              '<text x="40" y="425" font-size="11">Payload bytes 4-5, zero-based. Field zero does not establish physical motor stop.</text>', '</g></svg>']
    return "\n".join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    parser.add_argument("--output", type=Path, default=Path("reports/stop_timeline"))
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        sample = [{"relative_s": t, "segment": s, "value_int16_le": v}
                  for t, s, v in [(0, 0, 0), (1, 0, -1), (2, 0, 0), (3, 1, 0), (4, 1, 0)]]
        assert [r["packets"] for r in runs(sample)] == [1, 1, 1, 2]
        assert timing(sample)["final_zero_first_s"] == 3
        assert timing(sample)["final_zero_observed_s"] == 1
        assert timing(sample[:2])["final_zero_first_s"] is None
        assert timing([])["final_zero_first_s"] is None
        chart = plot([{"relative_s": 0, "segment": 0, "value_int16_le": 0},
                      {"relative_s": 20, "segment": 0, "value_int16_le": 3000}], "test.log")
        assert '990.00,115.00' in chart and '>20.0</text>' in chart and 'test.log' in chart
        print("stop timeline self-check passed")
        return
    if args.log is None:
        parser.error("provide one saved capture log")
    stats, rows, series = extract(args.log, "single_flick_stop")
    by_id = {identifier: timing([r for r in rows if r["can_id"] == identifier])
             for identifier in ("0x503", "0x201")}
    zero = by_id["0x503"]["final_zero_first_s"]
    raw = args.log.read_bytes()
    first = summarize(raw)["first_t_us"]
    groups = defaultdict(list)
    for ident, ext, seg, stamp, packet in reconstruct(raw):
        payload = packet[11:-2]
        key = f'{"E" if ext else "S"}:0x{ident:03X} {packet[4]:02X}>{packet[5]:02X} flags={packet[8]:02X} cmd={packet[9]:02X}:{packet[10]:02X} payload_len={len(payload)}'
        groups[key].append(((stamp-first)/1e6, seg, payload))
    after = {}
    if zero is not None:
        for key, records in sorted(groups.items()):
            selected = [r for r in records if r[0] >= zero]
            if selected:
                after[key] = {"packets": len(selected), "unique_payloads": len({r[2] for r in selected}),
                              "last_s": round(selected[-1][0], 6),
                              "changed_payload_offsets": [i for i in range(len(selected[0][2]))
                                                          if len({r[2][i] for r in selected}) > 1]}
    report = {"capture": stats, "timing_by_id": by_id, "crc_valid_packets": sum(map(len, groups.values())),
              "groups_after_field_zero": after,
              "physical_observation": {"user_confirmation": "User replied that the requested experiment was completed in the latest Other capture; physical action times were not supplied.",
                                       "direction": None, "dial_stop_s": None, "motor_stop_s": None},
              "limitations": ["Relative times begin at first captured CAN frame, not the GUI start click.",
                              "Packet timestamps belong to the first constituent CAN frame, at driver dequeue.",
                              "Final zero identifies a field, not independently timed physical motion.",
                              "Other changing packets can also change at idle; they do not establish motor feedback.",
                              "Counters cover first-to-last recorded status only; partial capture boundaries remain."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    with args.output.with_suffix(".csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    args.output.with_suffix(".svg").write_text(plot(series, args.log.name), encoding="utf-8")
    analysis = analyze(args.log)
    args.log.with_suffix(".analysis.json").write_text(json.dumps(analysis, ensure_ascii=False, indent=2), encoding="utf-8")
    reference = args.log.parent / "20261007_154729_002678_idle.log"
    if reference.exists():
        args.log.with_suffix(".vs-idle.json").write_text(json.dumps(compare(analyze(reference), analysis), ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"capture": stats, "timing_by_id": by_id}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
