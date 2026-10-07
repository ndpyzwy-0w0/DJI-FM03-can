"""Compare user-labelled dial speeds offline; values are not motor speed units."""
import argparse
import csv
import json
from pathlib import Path
from statistics import median, quantiles

from analyze_direction import extract


def amplitude(values):
    active = [abs(value) for value in values if value]
    return {"median_nonzero_abs": median(active) if active else 0,
            "p95_nonzero_abs": quantiles(active, n=20, method="inclusive")[18]
            if len(active) > 1 else (active[0] if active else 0)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path, help="idle, slow forward/reverse, fast forward/reverse")
    parser.add_argument("--output", type=Path, default=Path("reports/speed_comparison"))
    parser.add_argument("--observation", help="User-supplied physical observation, if available")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        assert amplitude([0, 0]) == {"median_nonzero_abs": 0, "p95_nonzero_abs": 0}
        assert amplitude([-100, 0, 200]) == {"median_nonzero_abs": 150.0, "p95_nonzero_abs": 195.0}
        assert amplitude([-42])["p95_nonzero_abs"] == 42
        print("speed comparison self-check passed")
        return
    if len(args.logs) != 5:
        parser.error("provide five logs: idle, slow_forward, slow_reverse, fast_forward, fast_reverse")
    captures, all_rows = [], []
    labels = ("idle", "slow_forward", "slow_reverse", "fast_forward", "fast_reverse")
    for path, label in zip(args.logs, labels):
        stats, rows, series = extract(path, label)
        stats.update(amplitude([row["value_int16_le"] for row in series]))
        left = [row["payload_hex"] for row in rows if row["can_id"] == "0x503"]
        right = [row["payload_hex"] for row in rows if row["can_id"] == "0x201"]
        stats["one_extra_leading_zero_on_0x201"] = (
            len(right) == len(left) + 1 and right[0] == "0F 04 08 19 00 00" and left == right[1:])
        captures.append(stats)
        all_rows.extend(rows)
    ratios = {direction: captures[index + 3]["median_nonzero_abs"] / captures[index + 1]["median_nonzero_abs"]
              if captures[index + 1]["median_nonzero_abs"] else None
              for index, direction in enumerate(("forward", "reverse"))}
    report = {"field": "DUML 04:69 payload[4:6], signed little-endian 16-bit",
              "captures": captures, "fast_to_slow_nonzero_median_ratio": ratios,
              "scope": "Magnitude ratios describe this field only, not motor speed or dial angular velocity.",
              "user_observation": args.observation,
              "interpretation": "Direction and amplitude correlate with dial input. Input-rate or increment interpretation is supported; direct motor-speed meaning is not established. Target accumulation with speed limiting and deceleration control are hypotheses.",
              "limitations": ["No independently timed motor/dial motion or measured angle/speed.",
                              "Fast captures contain minority opposite-sign values; do not erase or explain them as confirmed physical reversal.",
                              "Observed peaks of +/-2048 alone do not prove a protocol clamp.",
                              "Per-capture counters are relative to first recorded firmware status; capture boundaries remain partial.",
                              "No CAN frames were transmitted and no independent motor control was verified."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    with args.output.with_suffix(".csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    for stats in captures:
        print(json.dumps({key: stats[key] for key in ("label", "minimum", "maximum", "median_nonzero_abs", "negative_packets", "positive_packets", "zero_packets")}))
    print("field amplitude ratios:", ratios)


if __name__ == "__main__":
    main()
