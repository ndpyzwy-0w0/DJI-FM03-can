"""Local FM03 serial capture UI. Run with --open to open the browser."""
import argparse
from collections import deque
from datetime import datetime, timedelta, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import unquote, urlparse
import webbrowser

import serial
from serial.tools import list_ports
from capture import parse_frame

ROOT = Path(__file__).resolve().parents[1]
DROP_KEYS = ("logger_dropped", "rx_missed", "rx_overrun", "bus_errors")
BEIJING = timezone(timedelta(hours=8))


def available_ports():
    return [{"port": p.device, "description": p.description,
             "usb": p.vid is not None} for p in list_ports.comports()]


class CaptureSession:
    def __init__(self, directory=ROOT / "captures", serial_factory=serial.Serial):
        self.directory = Path(directory)
        self.serial_factory = serial_factory
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self._reset("", "")

    def _reset(self, port, label):
        self.phase = "idle"
        self.port, self.label = port, label
        self.started_at = None
        self.started = None
        self.elapsed = 0
        self.raw_path = None
        self.summary_path = None
        self.error = ""
        self.frames = self.bytes_saved = self.gaps = self.resets = self.non_frame_lines = 0
        self.previous_sequence = None
        self.buffer = b""
        self.latest = deque(maxlen=150)
        self.messages = deque(maxlen=12)
        self.rate_window = deque()
        self.ids = {}
        self.status = None
        self.status_first = None
        self.counter_deltas = {key: 0 for key in DROP_KEYS}
        self.last_serial = None
        self.last_status = None

    def start(self, port, label):
        if not isinstance(port, str) or not isinstance(label, str):
            raise ValueError("串口和场景标签必须是文本。")
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,48}", label):
            raise ValueError("场景标签只支持英文字母、数字、下划线和短横线。")
        if port not in {item["port"] for item in available_ports()}:
            raise ValueError("找不到所选串口，请连接开发板后刷新串口。")
        with self.operation_lock, self.lock:
            if self.thread and self.thread.is_alive():
                raise ValueError("当前抓包尚未结束，请先停止。")
            self._reset(port, label)
            self.phase = "starting"
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._record, daemon=True)
            self.thread.start()

    def stop(self):
        with self.operation_lock:
            with self.lock:
                thread = self.thread
                if thread and thread.is_alive():
                    self.phase = "stopping"
                    self.stop_event.set()
            if thread:
                thread.join(timeout=3)

    def _consume(self, chunk):
        now = time.monotonic()
        with self.lock:
            self.bytes_saved += len(chunk)
            self.last_serial = now
            self.buffer += chunk
            lines = self.buffer.split(b"\n")
            self.buffer = lines.pop()
            # Keep display parsing bounded; every original byte is already on disk.
            if len(self.buffer) > 4096:
                self.buffer = b""
                self.non_frame_lines += 1
            batch_count = 0
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                if line.startswith("# STATUS "):
                    current = {key: int(value) for key, value in
                               re.findall(r"(\w+)=(\d+)", line)}
                    for key in DROP_KEYS:
                        if self.status is not None and key in current and key in self.status:
                            before, after = self.status[key], current[key]
                            reboot = current.get("t_us", 0) < self.status.get("t_us", 0)
                            self.counter_deltas[key] += after if reboot or after < before else after - before
                    if self.status_first is None:
                        self.status_first = current.copy()
                    self.status = current
                    self.last_status = now
                elif line.startswith("#"):
                    self.messages.append(line)
                elif (frame := parse_frame(line)) is not None:
                    stamp, sequence, identifier, extended, rtr, dlc, data = frame
                    self.frames += 1
                    batch_count += 1
                    if self.previous_sequence is not None:
                        if sequence > self.previous_sequence:
                            self.gaps += sequence - self.previous_sequence - 1
                        else:
                            self.resets += 1
                    self.previous_sequence = sequence
                    key = f"{'E' if extended else 'S'}:{identifier:08X}"
                    row = {"t_us": stamp, "sequence": sequence,
                           "id": f"0x{identifier:08X}" if extended else f"0x{identifier:03X}",
                           "extended": bool(extended), "rtr": bool(rtr), "dlc": dlc,
                           "data": data.hex(" ").upper()}
                    self.latest.append(row)
                    if key not in self.ids:
                        self.ids[key] = {"id": row["id"], "extended": bool(extended), "count": 0}
                    self.ids[key].update(count=self.ids[key]["count"] + 1,
                                         data=row["data"], rtr=bool(rtr), dlc=dlc)
                elif line:
                    self.non_frame_lines += 1
            if batch_count:
                self.rate_window.append((now, batch_count))
            while self.rate_window and now - self.rate_window[0][0] > 2:
                self.rate_window.popleft()

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            active = self.phase in ("starting", "capturing", "stopping")
            elapsed = now - self.started if active and self.started else self.elapsed
            return {"product": "fm03_capture", "phase": self.phase,
                    "port": self.port, "label": self.label,
                    "started_at": self.started_at, "elapsed": round(elapsed, 2),
                    "frames": self.frames, "fps": sum(n for t, n in self.rate_window if now - t <= 1) if active else 0,
                    "bytes_saved": self.bytes_saved, "unique_ids": len(self.ids),
                    "sequence_gaps": self.gaps, "sequence_resets": self.resets,
                    "non_frame_lines": self.non_frame_lines,
                    "trailing_bytes": len(self.buffer),
                    "status_first": self.status_first, "status": self.status,
                    "counter_deltas": self.counter_deltas.copy(),
                    "status_age": round(now - self.last_status, 1) if self.last_status else None,
                    "serial_age": round(now - self.last_serial, 1) if self.last_serial else None,
                    "latest": list(reversed(self.latest)),
                    "ids": sorted((dict(item) for item in self.ids.values()), key=lambda x: -x["count"]),
                    "messages": list(self.messages), "error": self.error,
                    "raw_file": self.raw_path.name if self.raw_path else None,
                    "summary_file": self.summary_path.name if self.summary_path else None,
                    "saved_directory": str(self.directory)}

    def _record(self):
        connection = None
        try:
            connection = self.serial_factory(port=None, baudrate=921600, timeout=0.15, write_timeout=1)
            connection.dtr = False
            connection.rts = False
            connection.port = self.port
            connection.open()
            self.directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(BEIJING)
            path = self.directory / f"{stamp:%Y%m%d_%H%M%S_%f}_{self.label}.log"
            with path.open("xb") as output:
                with self.lock:
                    self.raw_path = path
                    self.started_at = stamp.isoformat()
                    self.started = time.monotonic()
                    self.phase = "stopping" if self.stop_event.is_set() else "capturing"
                connection.write(b"s")  # Only requests UART status, never transmits CAN.
                while not self.stop_event.is_set():
                    chunk = connection.read(min(max(connection.in_waiting, 1), 65536))
                    if chunk:
                        output.write(chunk)
                        output.flush()
                        self._consume(chunk)
        except Exception as exc:
            with self.lock:
                self.error = f"抓包失败：{exc}"
            logging.exception("Capture failed")
        finally:
            if connection:
                try:
                    connection.close()
                except Exception as exc:
                    with self.lock:
                        self.error = self.error or f"串口关闭失败：{exc}"
            with self.lock:
                self.elapsed = time.monotonic() - self.started if self.started else 0
                self.phase = "stopping"
            if self.raw_path:
                try:
                    summary = self.snapshot()
                    summary.update(phase="error" if self.error else "stopped",
                                   serial_baud=921600,
                                   stopped_at=datetime.now(BEIJING).isoformat())
                    target = self.raw_path.with_suffix(".json")
                    temporary = target.with_suffix(".json.tmp")
                    temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
                    temporary.replace(target)
                    with self.lock:
                        self.summary_path = target
                except Exception as exc:
                    with self.lock:
                        self.error = f"原始日志已保留，但摘要保存失败：{exc}"
                    logging.exception("Summary save failed")
            with self.lock:
                self.phase = "error" if self.error else "stopped"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def _reply(self, code, body, content_type="application/json; charset=utf-8", download=None):
        if not isinstance(body, bytes):
            body = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        if download:
            self.send_header("Content-Disposition", f'attachment; filename="{download}"')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _local_request(self):
        port = self.server.server_port
        allowed = {f"127.0.0.1:{port}", f"localhost:{port}"}
        if self.headers.get("Host") not in allowed:
            self._reply(403, {"error": "仅允许本机页面访问。"})
            return False
        origin = self.headers.get("Origin")
        if origin and origin not in {f"http://{host}" for host in allowed}:
            self._reply(403, {"error": "请求来源不匹配。"})
            return False
        return True

    def do_GET(self):
        if not self._local_request():
            return
        route = urlparse(self.path).path
        if route == "/api/state":
            self._reply(200, {**self.server.capture.snapshot(), "token": self.server.token})
        elif route == "/api/ports":
            self._reply(200, available_ports())
        elif route in ("/", "/app.js", "/style.css"):
            name, mime = {"/": ("index.html", "text/html"),
                          "/app.js": ("app.js", "text/javascript"),
                          "/style.css": ("style.css", "text/css")}[route]
            self._reply(200, (ROOT / "tools" / "ui" / name).read_bytes(), f"{mime}; charset=utf-8")
        elif route.startswith("/captures/"):
            name = unquote(route.removeprefix("/captures/"))
            if not re.fullmatch(r"\d{8}_\d{6}_\d{6}_[A-Za-z0-9_-]+\.(log|json)", name):
                self._reply(404, {"error": "文件不存在。"})
                return
            path = self.server.capture.directory / name
            if not path.is_file():
                self._reply(404, {"error": "文件不存在。"})
                return
            self._reply(200, path.read_bytes(), "application/octet-stream", download=name)
        else:
            self._reply(404, {"error": "页面不存在。"})

    def do_POST(self):
        if not self._local_request():
            return
        if not secrets.compare_digest(self.headers.get("X-FM03-Token", ""), self.server.token):
            self._reply(403, {"error": "请刷新上位机页面后重试。"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 <= length <= 2048:
                raise ValueError("请求长度无效。")
            body = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(body, dict):
                raise ValueError("请求格式无效。")
            route = urlparse(self.path).path
            if route == "/api/start":
                self.server.capture.start(body.get("port"), body.get("label", "capture"))
            elif route == "/api/stop":
                self.server.capture.stop()
            else:
                self._reply(404, {"error": "操作不存在。"})
                return
            self._reply(200, self.server.capture.snapshot())
        except (ValueError, json.JSONDecodeError) as exc:
            self._reply(400, {"error": str(exc)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    (ROOT / ".pio").mkdir(exist_ok=True)
    logging.basicConfig(filename=ROOT / ".pio" / "gui-server.log", level=logging.WARNING,
                        format="%(asctime)s %(levelname)s %(message)s", encoding="utf-8")
    url = f"http://127.0.0.1:{args.port}"
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    except OSError:
        from urllib.request import urlopen
        try:
            with urlopen(f"{url}/api/state", timeout=2) as response:
                existing = json.load(response)
            if existing.get("product") != "fm03_capture":
                raise RuntimeError("端口被其他程序占用。")
            if args.open:
                webbrowser.open(url)
            return
        except Exception:
            logging.exception("Cannot start GUI server")
            raise
    server.capture = CaptureSession()
    server.token = secrets.token_urlsafe(32)
    if args.open:
        webbrowser.open(url)
    print(f"FM03 GUI: {url}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.capture.stop()
        server.server_close()


if __name__ == "__main__":
    main()
