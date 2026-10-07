"""Save the UART bytes unchanged, plus a small capture summary."""
import argparse
from collections import Counter
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import re
import time


def parse_frame(line):
    fields = line.strip().split(",")
    if len(fields) != 7 or line.startswith("#"):
        return None
    try:
        stamp, sequence = int(fields[0]), int(fields[1])
        identifier = int(fields[2], 16)
        extended, rtr, dlc = map(int, fields[3:6])
        data = bytes.fromhex(fields[6])
        if stamp < 0 or sequence < 1 or extended not in (0, 1) or rtr not in (0, 1):
            return None
        if not 0 <= identifier <= (0x1FFFFFFF if extended else 0x7FF):
            return None
        if not 0 <= dlc <= 15 or len(data) != (0 if rtr else min(dlc, 8)):
            return None
        return stamp, sequence, identifier, extended, rtr, dlc, data
    except ValueError:
        return None


def summarize(raw):
    ids = Counter()
    frames = malformed = gaps = resets = 0
    previous = None
    first_stamp = last_stamp = None
    statuses = []
    errors = []
    for raw_line in raw.splitlines():
        line = raw_line.decode("ascii", errors="replace")
        if line.startswith("# STATUS "):
            statuses.append({key: int(value) for key, value in
                             re.findall(r"(\w+)=(\d+)", line)})
        elif line.startswith("# ERROR "):
            errors.append(line)
        elif (frame := parse_frame(line)) is not None:
            stamp, sequence, identifier, extended, *_ = frame
            frames += 1
            ids[f"{'E' if extended else 'S'}:{identifier:08X}"] += 1
            if previous is not None:
                if sequence > previous:
                    gaps += sequence - previous - 1
                else:
                    resets += 1
            previous = sequence
            if first_stamp is None:
                first_stamp = stamp
            last_stamp = stamp
        elif line and not line.startswith("#"):
            malformed += 1
    return {"frames": frames, "ids": dict(ids), "sequence_gaps": gaps,
            "sequence_resets": resets, "non_frame_lines": malformed,
            "first_t_us": first_stamp, "last_t_us": last_stamp,
            "status_first": statuses[0] if statuses else None,
            "status_last": statuses[-1] if statuses else None,
            "firmware_errors": errors}


def self_check():
    raw = (b"# READY listen_only\n"
           b"100,1,00000123,0,0,2,AABB\n"
           b"110,3,001ABCDE,1,1,8,\n"
           b"# STATUS rx=3 rx_missed=0 logger_dropped=1\n")
    result = summarize(raw)
    assert result["frames"] == 2 and result["sequence_gaps"] == 1
    assert result["ids"] == {"S:00000123": 1, "E:001ABCDE": 1}
    assert result["status_last"]["logger_dropped"] == 1
    assert parse_frame("1,1,00000800,0,0,0,") is None
    assert parse_frame("1,1,00000001,0,0,2,AA") is None
    assert parse_frame("1,1,00000001,0,1,2,AA") is None
    assert parse_frame("1,1,00000001,2,0,0,") is None
    assert summarize(b"1,9,00000001,0,0,0,\n2,1,00000001,0,0,0,\n")["sequence_resets"] == 1
    print("capture self-check passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--label", default="probe")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if not args.port or args.seconds <= 0 or not re.fullmatch(r"[A-Za-z0-9_-]+", args.label):
        parser.error("provide --port, positive --seconds and an ASCII filename label")

    import serial
    root = Path(__file__).resolve().parents[1] / "captures"
    root.mkdir(exist_ok=True)
    started_at = datetime.now(timezone(timedelta(hours=8)))
    path = root / f"{started_at:%Y%m%d_%H%M%S_%f}_{args.label}.log"
    started = time.monotonic()
    interrupted = False
    # Set control lines before opening, so capture doesn't intentionally reset ESP32.
    connection = serial.Serial(port=None, baudrate=921600, timeout=0.2)
    connection.dtr = False
    connection.rts = False
    connection.port = args.port
    try:
        connection.open()
        with path.open("xb") as output:
            connection.write(b"s")  # UART status request; never a CAN message.
            print(f"Recording {args.seconds:g}s on {args.port}: {path}", flush=True)
            try:
                while time.monotonic() - started < args.seconds:
                    chunk = connection.read(4096)
                    output.write(chunk)
                    output.flush()
            except KeyboardInterrupt:
                interrupted = True
    finally:
        connection.close()
    result = {"port": args.port, "serial_baud": 921600, "label": args.label,
              "started_at": started_at.isoformat(),
              "elapsed_seconds": round(time.monotonic() - started, 3),
              "interrupted": interrupted, "raw_log": str(path),
              **summarize(path.read_bytes())}
    path.with_suffix(".json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
