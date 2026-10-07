"""Leave-one-out stability check over saved captures; offline and read-only.

Purpose: separate "observed constant" from "protocol constant". A byte that is
fixed inside one capture is only an observation; it becomes stable evidence only
if it stays fixed when every capture is removed in turn. This tool never opens a
serial port and never transmits.

Evidence class is decided from the saved summary, not from file names:
healthy = recorded STATUS interval adds no logger_dropped / rx_missed /
rx_overrun / bus_errors and shows no CAN sequence gap or reset.
"""
import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

from analyze_protocol import reconstruct
from capture import summarize

DROP_KEYS = ("logger_dropped", "rx_missed", "rx_overrun", "bus_errors")


def classify(summary):
    """Decide from the summary alone whether a capture is usable as evidence."""
    reasons = []
    first, last = summary.get("status_first"), summary.get("status_last")
    if first and last:
        for key in DROP_KEYS:
            if key in first and key in last and last[key] != first[key]:
                reasons.append(f"{key} +{last[key] - first[key]}")
    if summary["sequence_gaps"]:
        reasons.append(f"sequence_gaps={summary['sequence_gaps']}")
    if summary["sequence_resets"]:
        reasons.append(f"sequence_resets={summary['sequence_resets']}")
    if not summary["frames"]:
        reasons.append("no CAN frames")
    return {"healthy": not reasons, "rejected_because": reasons}


def route(packet):
    """Observation key: CAN routing plus command and payload length."""
    return (f"{packet[4]:02X}>{packet[5]:02X} flags={packet[8]:02X} "
            f"cmd={packet[9]:02X}:{packet[10]:02X} len={len(packet) - 13}")


def collect(path):
    """Per-capture statistics for every observation route."""
    raw = path.read_bytes()
    summary = summarize(raw)
    routes = {}
    for identifier, extended, segment, stamp, packet in reconstruct(raw):
        key = route(packet)
        entry = routes.setdefault(key, {"packets": 0, "can_ids": Counter(), "segments": set(),
                                        "byte_values": defaultdict(set), "payloads": Counter(),
                                        "stamps": [], "sequences": [], "payload_length": len(packet) - 13})
        payload = packet[11:-2]
        entry["packets"] += 1
        entry["can_ids"][f"{'E' if extended else 'S'}:0x{identifier:03X}"] += 1
        entry["segments"].add(segment)
        entry["payloads"][payload.hex(" ")] += 1
        entry["stamps"].append(stamp)
        entry["sequences"].append(int.from_bytes(packet[6:8], "little"))
        # Index payload bytes as a fixed-width header slice so all packets of the
        # same route compare position by position.
        for index, byte in enumerate(packet):
            entry["byte_values"][index].add(byte)
    for entry in routes.values():
        steps = Counter()
        for left, right in zip(entry["sequences"], entry["sequences"][1:]):
            steps[(right - left) % 65536] += 1
        intervals = [b - a for a, b in zip(entry["stamps"], entry["stamps"][1:]) if b >= a]
        entry["sequence_steps"] = dict(sorted(steps.items()))
        entry["median_interval_ms"] = round(median(intervals) / 1000, 4) if intervals else None
        entry["distinct_payloads"] = len(entry["payloads"])
        entry["top_payload"] = entry["payloads"].most_common(1)[0][0]
        entry["segments"] = sorted(entry["segments"])
        entry["byte_values"] = {index: sorted(values) for index, values in entry["byte_values"].items()}
        entry["payloads"] = dict(entry["payloads"])
        entry["can_ids"] = dict(entry["can_ids"])
        del entry["stamps"], entry["sequences"]
    return {"raw_log": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest(),
            "integrity": summary, "evidence": classify(summary), "routes": routes}


def compare(captures):
    """Cross-capture stability per route, including leave-one-out behaviour."""
    names = [Path(capture["raw_log"]).name for capture in captures]
    identifiers = [capture["raw_log"] for capture in captures]
    routes = sorted({key for capture in captures for key in capture["routes"]})
    rows = []
    for key in routes:
        present = [capture for capture in captures if key in capture["routes"]]
        absent = [name for name, capture in zip(names, captures) if key not in capture["routes"]]
        # A byte is stable only if its value set has one element in every capture
        # that carries the route; otherwise record exactly which captures vary.
        offsets = sorted({index for capture in present for index in capture["routes"][key]["byte_values"]})
        # Payload offsets are zero-based excluding the 11-byte DUML header and the
        # trailing CRC16, matching the rest of the project. Packet offsets keep the
        # header and trailer so those bytes can still be reported separately; the
        # payload length comes from the route itself, never from an assumed layout.
        payload_length = next(iter(present))["routes"][key]["payload_length"]

        def payload_offset(packet_offset):
            return packet_offset - 11 if 11 <= packet_offset < 11 + payload_length else None

        stable, varying = [], {}
        for index in offsets:
            values = {capture["routes"][key]["byte_values"][index][0]
                      for capture in present}
            if all(len(capture["routes"][key]["byte_values"][index]) == 1 for capture in present) and len(values) == 1:
                stable.append(index)
            else:
                varying[index] = {name: [f"{value:02X}" for value in capture["routes"][key]["byte_values"][index]]
                                  for name, capture in zip(names, present)}
        # Keep every capture's own step distribution: one majority signature would
        # hide a capture that genuinely disagrees.
        steps_by_capture = {name: capture["routes"][key]["sequence_steps"]
                            for name, capture in zip(names, present)}
        signatures = {json.dumps(value, sort_keys=True) for value in steps_by_capture.values()}
        intervals = [capture["routes"][key]["median_interval_ms"] for capture in present]
        # Leave-one-out: with capture k removed, does any other capture disagree
        # on a byte that looked constant inside capture k alone?
        leave_one_out = []
        for index in stable:
            for dropped in range(len(present)):
                others = [capture["routes"][key]["byte_values"][index][0]
                          for position, capture in enumerate(present) if position != dropped]
                if len(set(others)) > 1:
                    leave_one_out.append({"offset": index, "dropped": Path(present[dropped]["raw_log"]).name,
                                          "remaining_values": sorted(f"{value:02X}" for value in set(others))})
        rows.append({
            "route": key, "captures_present": len(present), "captures_absent": absent,
            "packets_total": sum(capture["routes"][key]["packets"] for capture in present),
            "packets_per_capture": {name: capture["routes"][key]["packets"]
                                    for name, capture in zip(names, present)},
            "can_ids": sorted({can_id for capture in present for can_id in capture["routes"][key]["can_ids"]}),
            "stable_packet_offsets": stable,
            "stable_payload_offsets": [offset for offset in (payload_offset(i) for i in stable)
                                       if offset is not None],
            "varying_offsets": sorted(varying),
            "varying_payload_offsets": [offset for offset in (payload_offset(i) for i in sorted(varying))
                                        if offset is not None],
            "varying_offset_values": {str(index): values for index, values in sorted(varying.items())},
            "cross_capture_disagreements": leave_one_out,
            "sequence_steps_identical_across_captures": len(signatures) == 1,
            "sequence_steps_per_capture": steps_by_capture,
            "sequence_steps_signatures": len(signatures),
            "median_interval_ms_range": [min(intervals), max(intervals)] if intervals else None,
            "distinct_payload_counts": sorted(capture["routes"][key]["distinct_payloads"] for capture in present),
        })
    return {"captures": names, "routes": rows,
            "leave_one_out_convention": ("A byte is reported stable only when it holds one value in "
                                         "every capture carrying the route; leave-one-out re-checks each "
                                         "stable offset with one capture removed at a time."),
            "limits": ["Stability across these captures is repetition evidence, not proof that a byte is a protocol constant.",
                       "Records are one session on one bench; a stable byte may still be a session constant.",
                       "Byte offsets are packet offsets, so the 11-byte header is included in the ranges.",
                       "No route or byte is shown to be necessary for independent control.",
                       "This tool never opens a serial port and never transmits CAN."]}


def self_check():
    crafted = []
    # Three synthetic captures, one route, one deliberately unstable byte.
    for index, value in enumerate((0x00, 0x00, 0x11)):
        packet = bytes([0x55, 0x13, 0x04, 0x03, 0xE5, 0xBF,
                        (index + 1) & 0xFF, 0x00, 0x00, 0x04, 0x69,
                        0x0F, 0x04, 0x08, 0x19, value, 0x00, 0x00, 0x00])
        # Recompute both CRCs so the packet survives reconstruction.
        from build_packet import build_packet as rebuild
        packet = rebuild(0xE5, 0xBF, index + 1, 0x04, 0x69, packet[11:-2], 0x00, 1)
        crafted.append(packet)
    assert route(crafted[0]) == route(crafted[1]) == "E5>BF flags=00 cmd=04:69 len=6"
    assert classify({"status_first": {"logger_dropped": 1}, "status_last": {"logger_dropped": 1},
                     "sequence_gaps": 0, "sequence_resets": 0, "frames": 5})["healthy"]
    assert not classify({"status_first": {"logger_dropped": 1}, "status_last": {"logger_dropped": 2},
                         "sequence_gaps": 0, "sequence_resets": 0, "frames": 5})["healthy"]
    assert not classify({"status_first": None, "status_last": None,
                         "sequence_gaps": 3, "sequence_resets": 0, "frames": 5})["healthy"]
    assert not classify({"status_first": None, "status_last": None,
                         "sequence_gaps": 0, "sequence_resets": 0, "frames": 0})["healthy"]
    # The differing payload byte must be reported as varying, and the fixed bytes
    # must survive a leave-one-out check when a capture is removed.
    def fake(name, payload_byte, steps=None):
        from build_packet import build_packet as rebuild
        packet = rebuild(0xE5, 0xBF, 1, 0x04, 0x69, bytes([0x0F, 0x04, 0x08, 0x19, payload_byte, 0x00]), 0, 1)
        entry = {"packets": 1, "can_ids": {"S:0x503": 1}, "segments": [0],
                 "byte_values": {index: [byte] for index, byte in enumerate(packet)},
                 "payloads": {packet[11:-2].hex(" "): 1}, "distinct_payloads": 1, "payload_length": 6,
                 "top_payload": packet[11:-2].hex(" "), "sequence_steps": steps or {0: 1},
                 "median_interval_ms": 10.0}
        return {"raw_log": name, "sha256": "0" * 64, "integrity": {}, "evidence": {"healthy": True},
                "routes": {"E5>BF flags=00 cmd=04:69 len=6": entry}}
    result = compare([fake("a.log", 0x00), fake("b.log", 0x0A)])
    row = result["routes"][0]
    assert row["captures_present"] == 2 and not row["captures_absent"]
    assert 15 in row["varying_offsets"] and 15 not in row["stable_packet_offsets"]
    assert 4 not in row["varying_offsets"] and 4 in row["stable_packet_offsets"]
    # Payload offsets drop the 11-byte DUML header: packet offset 15 is payload 4,
    # packet offsets 11..13 are payload 0..2, and header bytes are not reported.
    assert 4 in row["varying_payload_offsets"] and 4 not in row["stable_payload_offsets"]
    assert 3 in row["stable_payload_offsets"] and 0 not in row["varying_payload_offsets"]
    # The trailing CRC16 sits at packet offsets 17 and 18 for a 6-byte payload.
    # Those two bytes legitimately change when the payload changes, so they must be
    # reported as packet offsets and excluded from the payload offset lists.
    assert row["varying_payload_offsets"] == [4]
    assert 17 in row["varying_offsets"] and 18 in row["varying_offsets"]
    assert max(row["varying_payload_offsets"]) == 4 and 5 not in row["varying_payload_offsets"]
    assert row["stable_payload_offsets"] == [0, 1, 2, 3, 5]
    assert not row["cross_capture_disagreements"]
    assert row["sequence_steps_identical_across_captures"] and row["sequence_steps_signatures"] == 1
    # A capture whose step distribution differs must be reported, not averaged into
    # a majority signature: the flag turns false and every capture keeps its own.
    mixed = compare([fake("a.log", 0x00, {2: 10, 4: 1}), fake("b.log", 0x05, {2: 11})])
    mixed_row = mixed["routes"][0]
    assert not mixed_row["sequence_steps_identical_across_captures"]
    assert mixed_row["sequence_steps_signatures"] == 2
    assert mixed_row["sequence_steps_per_capture"]["a.log"] == {2: 10, 4: 1}
    assert mixed_row["sequence_steps_per_capture"]["b.log"] == {2: 11}
    # Routes present in only part of the set must be reported with the absent names.
    partial = compare([fake("a.log", 0x00), {"raw_log": "c.log", "sha256": "0" * 64,
                                             "integrity": {}, "evidence": {"healthy": True}, "routes": {}}])
    assert partial["routes"][0]["captures_absent"] == ["c.log"]
    print("stability checker self-check passed")


def render(report):
    lines = ["# 跨记录稳定性复核（留一法）", "",
             f"参与记录：{len(report['captures'])} 份。口径：{report['leave_one_out_convention']}", "",
             "## 逐路由稳定性", "",
             "| 路由（逻辑地址/flags/命令/长度） | 记录数 | 包数 | 稳定payload偏移 | 变化payload偏移 | 序号步进签名数 | 周期中位范围(ms) |",
             "|---|---|---|---|---|---|---|"]
    for row in report["routes"]:
        interval = row["median_interval_ms_range"]
        interval_text = f"{interval[0]}～{interval[1]}" if interval else "n/a"
        lines.append(f"| `{row['route']}` | {row['captures_present']}/{len(report['captures'])} | "
                     f"{row['packets_total']} | {row['stable_payload_offsets']} | "
                     f"{row['varying_payload_offsets']} | {row['sequence_steps_signatures']} | {interval_text} |")
    lines += ["", "## 序号步进逐记录对照", "",
              "各记录自己的步进分布，不做多数合并；签名数 >1 表示记录之间确实不一致。", ""]
    for row in report["routes"]:
        if row["sequence_steps_signatures"] <= 1:
            continue
        lines.append(f"### `{row['route']}`")
        lines.append("")
        for name, steps in row["sequence_steps_per_capture"].items():
            lines.append(f"- {name}：{steps}")
        lines.append("")
    lines += ["## 未在所有记录出现的路由", ""]
    absent = [row for row in report["routes"] if row["captures_absent"]]
    lines += [f"- `{row['route']}`：缺 {', '.join(row['captures_absent'])}" for row in absent] or ["- 无"]
    lines += ["", "## 限制", ""] + [f"- {note}" for note in report["limits"]] + [""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--output", type=Path, help="write the JSON report here")
    parser.add_argument("--markdown", type=Path, help="write a short markdown summary here")
    parser.add_argument("--include-unhealthy", action="store_true",
                        help="also analyse captures whose summary shows new errors or gaps")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if not args.logs:
        parser.error("provide two or more saved .log files")
    if len(set(path.resolve() for path in args.logs)) != len(args.logs):
        parser.error("duplicate input logs would count the same evidence twice")
    captures, skipped = [], []
    for path in args.logs:
        capture = collect(path)
        (captures if capture["evidence"]["healthy"] or args.include_unhealthy else skipped).append(capture)
    if len(captures) < 2:
        parser.error(f"need at least two usable captures, got {len(captures)}")
    for capture in skipped:
        print(f"skipped {Path(capture['raw_log']).name}: "
              f"{', '.join(capture['evidence']['rejected_because'])}")
    report = compare(captures)
    report["skipped"] = [{"raw_log": capture["raw_log"], "sha256": capture["sha256"],
                          "rejected_because": capture["evidence"]["rejected_because"]} for capture in skipped]
    report["capture_sha256"] = {Path(capture["raw_log"]).name: capture["sha256"] for capture in captures}
    print(f"{len(captures)} captures, {len(report['routes'])} routes, "
          f"{sum(row['packets_total'] for row in report['routes'])} packets")
    for row in report["routes"]:
        print(f"  {row['route']}: {row['packets_total']} packets, "
              f"{len(row['stable_payload_offsets'])} stable payload offsets, "
              f"{len(row['varying_payload_offsets'])} varying, absent in {len(row['captures_absent'])} captures, "
              f"disagreements={len(row['cross_capture_disagreements'])}, "
              f"sequence-step signatures={row['sequence_steps_signatures']}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"wrote {args.output}")
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(render(report), encoding="utf-8")
        print(f"wrote {args.markdown}")


if __name__ == "__main__":
    main()
