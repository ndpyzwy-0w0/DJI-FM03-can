"""FM03 focus-motor control host.

Drives the `motor_control` firmware live over the UART: pick a value, watch the motor
react, change it, stop it. No reflashing between experiments.

    .venv/Scripts/python.exe tools/motor_control.py            # pick the port in the UI
    .venv/Scripts/python.exe tools/motor_control.py --port COM17 --connect

Safety built into the pair (firmware and host):
  * The firmware boots listening only. Nothing moves until `arm`, and arming does not
    itself command motion.
  * The host sends `status` as a heartbeat. If the host stops (this window closes, the
    cable is pulled, the machine sleeps), the firmware zeroes the value after its
    watchdog window and prints why.
  * `stop` commands zero immediately and keeps control; the big red button and the Esc
    key both send it.
  * The device's own BOOT button, held for two seconds, also stops and disarms.
  * The magnitude is clamped by both the slider range and the firmware limit.

The motor must be the only other node on the bus: the RS3 must be physically
disconnected while this program transmits.
"""
from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - reported to the operator below
    serial = None
    list_ports = None

BAUD = 921600
DEFAULT_LIMIT = 2048
HEARTBEAT_MS = 500
STATE_KEYS = (
    "mode", "armed", "value", "target", "run", "run_left_ms", "sent", "nonzero", "zero",
    "rejected", "tx_failed", "twai_state", "msgs_to_tx", "bus_errors", "tx_errors",
    "rx_missed", "limit", "interval_ms", "watchdog_ms", "watchdog_stops",
    "run_complete", "button", "version",
)


def parse_state(line: str) -> dict:
    """Parse one '# STATE ...' telemetry line into a dict of typed values."""
    body = line.split("STATE", 1)[1]
    state: dict = {}
    for match in re.finditer(r'([A-Za-z_]+)=("[^"]*"|\S+)', body):
        key, raw = match.group(1), match.group(2)
        if key not in STATE_KEYS:
            continue
        if raw.startswith('"'):
            state[key] = raw.strip('"')
            continue
        try:
            state[key] = int(raw)
        except ValueError:
            state[key] = raw
    return state


def describe_direction(value: int) -> str:
    if value > 0:
        return f"+{value}  (one direction)"
    if value < 0:
        return f"{value}  (the opposite direction)"
    return "0  (stopped)"


class MotorLink:
    """Owns the serial port and the device conversation. UI-agnostic and testable."""

    def __init__(self, events: queue.Queue):
        self.events = events
        self.serial = None
        self.port = None
        self.lock = threading.Lock()
        self.reader = None
        self.heartbeat = None
        self.stopping = threading.Event()
        self.lines_written = 0
        self.lines_read = 0
        self.bad_lines = 0
        self.last_state: dict = {}
        self.last_state_at = 0.0
        self.last_rx_at = 0.0
        self.connect_error = None
        # Kept as an attribute so a test can shorten it without touching production
        # code; the shipped cadence is HEARTBEAT_MS.
        self.heartbeat_ms = HEARTBEAT_MS

    # -- lifecycle ---------------------------------------------------------
    def open(self, port: str) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed in this interpreter")
        self.close()
        self.stopping.clear()
        connection = serial.Serial(port=None, baudrate=BAUD, timeout=0.1, write_timeout=1)
        # Set control lines before opening: this must never reset the ESP32.
        connection.dtr = False
        connection.rts = False
        connection.port = port
        connection.open()
        self.serial = connection
        self.port = port
        self.connect_error = None
        self.reader = threading.Thread(target=self._read_loop, name="motor-reader", daemon=True)
        self.reader.start()
        self.heartbeat = threading.Thread(target=self._heartbeat_loop, name="motor-heartbeat",
                                          daemon=True)
        self.heartbeat.start()
        self.send("status")

    def close(self) -> None:
        self.stopping.set()
        if self.heartbeat and self.heartbeat.is_alive():
            self.heartbeat.join(timeout=1)
        if self.reader and self.reader.is_alive():
            self.reader.join(timeout=1)
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.reader = None
        self.heartbeat = None

    def is_open(self) -> bool:
        return self.serial is not None and getattr(self.serial, "is_open", False)

    # -- conversation ------------------------------------------------------
    def send(self, command: str) -> bool:
        if not self.is_open():
            return False
        payload = (command + "\n").encode("ascii", errors="replace")
        try:
            with self.lock:
                self.serial.write(payload)
                self.lines_written += 1
            return True
        except Exception as exc:  # cable pulled, port closed underneath us
            self.events.put(("error", f"write failed: {exc}"))
            return False

    def _read_loop(self) -> None:
        buffer = b""
        while not self.stopping.is_set():
            try:
                chunk = self.serial.read(4096) if self.serial is not None else b""
            except Exception as exc:
                if not self.stopping.is_set():
                    self.events.put(("error", f"read failed: {exc}"))
                return
            if not chunk:
                continue
            self.last_rx_at = time.monotonic()
            buffer += chunk
            lines = buffer.split(b"\n")
            buffer = lines.pop()
            if len(buffer) > 8192:
                buffer = b""
                self.bad_lines += 1
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                self.lines_read += 1
                if line.startswith("# STATE "):
                    state = parse_state(line)
                    self.last_state = state
                    self.last_state_at = time.monotonic()
                    self.events.put(("state", state))
                elif line.startswith("#"):
                    self.events.put(("line", line))
                else:
                    self.bad_lines += 1

    def _heartbeat_loop(self) -> None:
        while not self.stopping.is_set():
            self.send("status")
            self.stopping.wait(self.heartbeat_ms / 1000.0)


class MotorControlApp:
    def __init__(self, root: tk.Tk, port: str | None, autoconnect: bool):
        self.root = root
        self.events: queue.Queue = queue.Queue()
        self.link = MotorLink(self.events)
        self.armed = False
        self.value = 0
        self.limit = DEFAULT_LIMIT
        self.build_ui()
        if port:
            self.port_var.set(port)
        self.refresh_ports()
        if autoconnect and self.port_var.get():
            self.root.after(200, self.connect)
        self.root.after(50, self.pump)

    # -- UI ----------------------------------------------------------------
    def build_ui(self) -> None:
        self.root.title("FM03 跟焦电机 运动控制")
        self.root.minsize(760, 620)
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        # Connection row
        conn = ttk.LabelFrame(outer, text="连接", padding=8)
        conn.pack(fill="x")
        ttk.Label(conn, text="串口").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, width=28, state="readonly")
        self.port_box.grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(conn, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=3)
        self.connect_button = ttk.Button(conn, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=3)
        self.conn_label = ttk.Label(conn, text="未连接")
        self.conn_label.grid(row=0, column=4, padx=10, sticky="w")
        self.heartbeat_label = ttk.Label(conn, text="")
        self.heartbeat_label.grid(row=0, column=5, padx=10, sticky="w")

        # Enable row
        enable = ttk.LabelFrame(outer, text="使能（必须先 arm，板子才会开始发送）", padding=8)
        enable.pack(fill="x", pady=(8, 0))
        self.arm_button = ttk.Button(enable, text="ARM 使能", command=self.arm)
        self.arm_button.grid(row=0, column=0, padx=3)
        self.disarm_button = ttk.Button(enable, text="DISARM 释放", command=self.disarm)
        self.disarm_button.grid(row=0, column=1, padx=3)
        self.stop_button = tk.Button(enable, text="STOP 归零 (Esc)", command=self.stop,
                                     bg="#c62828", fg="white", font=("", 11, "bold"),
                                     padx=16, pady=4)
        self.stop_button.grid(row=0, column=2, padx=10)
        self.arm_label = ttk.Label(enable, text="disarmed", font=("", 10, "bold"))
        self.arm_label.grid(row=0, column=3, padx=10)

        # Value control
        control = ttk.LabelFrame(outer, text="数值控制（正负代表方向，绝对值越大转越多）", padding=8)
        control.pack(fill="x", pady=(8, 0))
        self.value_scale = ttk.Scale(control, from_=-DEFAULT_LIMIT, to=DEFAULT_LIMIT,
                                     orient="horizontal", command=self.on_scale, length=520)
        self.value_scale.grid(row=0, column=0, columnspan=6, sticky="we", pady=(0, 6))
        ttk.Label(control, text="数值").grid(row=1, column=0, sticky="w")
        self.value_var = tk.StringVar(value="0")
        entry = ttk.Entry(control, textvariable=self.value_var, width=10, font=("Consolas", 11))
        entry.grid(row=1, column=1, sticky="w", padx=4)
        entry.bind("<Return>", lambda _event: self.send_value())
        ttk.Button(control, text="发送 (Enter)", command=self.send_value).grid(row=1, column=2, padx=4)
        ttk.Button(control, text="归零", command=self.zero).grid(row=1, column=3, padx=4)

        steps = ttk.Frame(control)
        steps.grid(row=2, column=0, columnspan=6, sticky="w", pady=(8, 0))
        ttk.Label(steps, text="步进 / 快捷").pack(side="left")
        for amount in (-256, -16, -1, 1, 16, 256):
            ttk.Button(steps, text=f"{amount:+d}", width=5,
                       command=lambda a=amount: self.nudge(a)).pack(side="left", padx=2)
        ttk.Button(steps, text="清除", command=lambda: self.set_value(0)).pack(side="left", padx=8)

        run = ttk.Frame(control)
        run.grid(row=3, column=0, columnspan=6, sticky="w", pady=(8, 0))
        ttk.Label(run, text="定时运行：保持").pack(side="left")
        self.run_ms_var = tk.StringVar(value="2000")
        ttk.Entry(run, textvariable=self.run_ms_var, width=7).pack(side="left", padx=3)
        ttk.Label(run, text="ms 然后归零").pack(side="left")
        ttk.Button(run, text="运行当前数值", command=self.run_timed).pack(side="left", padx=8)
        ttk.Label(run, text="拨轮周期 ms").pack(side="left", padx=(16, 0))
        self.interval_var = tk.StringVar(value="50")
        ttk.Entry(run, textvariable=self.interval_var, width=6).pack(side="left", padx=3)
        ttk.Button(run, text="应用", command=self.apply_interval).pack(side="left", padx=3)

        # Big readout
        readout = ttk.LabelFrame(outer, text="当前下发值", padding=8)
        readout.pack(fill="x", pady=(8, 0))
        self.value_display = tk.Label(readout, text=describe_direction(0),
                                      font=("", 22, "bold"), anchor="w")
        self.value_display.pack(fill="x")
        self.detail_label = ttk.Label(readout, text="", justify="left")
        self.detail_label.pack(fill="x")

        # Telemetry + log
        bottom = ttk.Frame(outer)
        bottom.pack(fill="both", expand=True, pady=(8, 0))
        telemetry = ttk.LabelFrame(bottom, text="设备状态", padding=8)
        telemetry.pack(side="left", fill="both", expand=True)
        self.telemetry_text = tk.Text(telemetry, width=34, height=9, font=("Consolas", 9))
        self.telemetry_text.pack(fill="both", expand=True)
        self.telemetry_text.configure(state="disabled")

        logging_frame = ttk.LabelFrame(bottom, text="事件", padding=8)
        logging_frame.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.log_text = tk.Text(logging_frame, height=9, font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

        self.root.bind("<Escape>", lambda _event: self.stop())
        self.root.bind("<Left>", lambda _event: self.nudge(-1))
        self.root.bind("<Right>", lambda _event: self.nudge(1))
        self.root.bind("<Up>", lambda _event: self.nudge(16))
        self.root.bind("<Down>", lambda _event: self.nudge(-16))

    def log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{stamp}  {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # -- helpers -----------------------------------------------------------
    def refresh_ports(self) -> None:
        if list_ports is None:
            self.log("pyserial 不可用，无法枚举串口")
            return
        ports = [p.device for p in list_ports.comports()]
        usb = [p.device for p in list_ports.comports() if p.vid is not None]
        self.port_box["values"] = usb + [p for p in ports if p not in usb]
        if not self.port_var.get() and usb:
            self.port_var.set(usb[0])

    def require_link(self) -> bool:
        if self.link.is_open():
            return True
        messagebox.showwarning("未连接", "请先连接串口。")
        return False

    def connect(self) -> None:
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("未选择串口", "请选择一个串口。")
            return
        try:
            self.link.open(port)
        except Exception as exc:
            self.link.connect_error = str(exc)
            self.log(f"连接失败：{exc}")
            messagebox.showerror("连接失败", str(exc))
            return
        self.log(f"已连接 {port} @ {BAUD}")
        self.connect_button.configure(text="断开", command=self.disconnect)

    def disconnect(self) -> None:
        self.link.close()
        self.armed = False
        self.connect_button.configure(text="连接", command=self.connect)
        self.conn_label.configure(text="未连接")
        self.log("已断开")

    def toggle_connection(self) -> None:
        if self.link.is_open():
            self.disconnect()
        else:
            self.connect()

    # -- commands ----------------------------------------------------------
    def arm(self) -> None:
        if not self.require_link():
            return
        self.link.send("arm")
        self.armed = True
        self.value = 0
        self.set_value(0)
        self.log("arm：已使能，数值仍为 0，先用滑块或输入选一个值")

    def disarm(self) -> None:
        if not self.require_link():
            return
        self.link.send("disarm")
        self.armed = False
        self.value = 0
        self.set_value(0)
        self.log("disarm：已释放控制，数值归零")

    def stop(self) -> None:
        if self.link.is_open():
            self.link.send("stop")
        self.value = 0
        self.set_value(0)
        self.log("stop：数值归零")

    def zero(self) -> None:
        self.set_value(0)
        self.send_value()

    def set_value(self, value: int) -> None:
        value = max(-self.limit, min(self.limit, int(value)))
        self.value = value
        self.value_var.set(str(value))
        self.value_scale.set(value)
        self.value_display.configure(text=describe_direction(value))

    def on_scale(self, raw: str) -> None:
        try:
            value = int(float(raw))
        except ValueError:
            return
        self.value = value
        self.value_var.set(str(value))
        self.value_display.configure(text=describe_direction(value))

    def nudge(self, amount: int) -> None:
        self.set_value(self.value + amount)
        self.send_value()

    def current_value(self) -> int | None:
        try:
            return max(-self.limit, min(self.limit, int(self.value_var.get())))
        except ValueError:
            messagebox.showwarning("数值无效", "请输入一个整数。")
            return None

    def send_value(self) -> None:
        if not self.require_link():
            return
        value = self.current_value()
        if value is None:
            return
        self.set_value(value)
        self.link.send(f"go {value}")
        self.log(f"go {value}")

    def run_timed(self) -> None:
        if not self.require_link():
            return
        value = self.current_value()
        if value is None:
            return
        try:
            duration = int(self.run_ms_var.get())
        except ValueError:
            messagebox.showwarning("时长无效", "请输入毫秒数（1..60000）。")
            return
        if not 1 <= duration <= 60000:
            messagebox.showwarning("时长无效", "时长需在 1..60000 ms。")
            return
        self.set_value(value)
        self.link.send(f"run {value} {duration}")
        self.log(f"run {value} {duration}ms")

    def apply_interval(self) -> None:
        if not self.require_link():
            return
        try:
            interval = int(self.interval_var.get())
        except ValueError:
            messagebox.showwarning("周期无效", "请输入毫秒数（10..1000）。")
            return
        if not 10 <= interval <= 1000:
            messagebox.showwarning("周期无效", "周期需在 10..1000 ms。")
            return
        self.link.send(f"interval {interval}")
        self.log(f"interval {interval}ms")

    # -- event pump --------------------------------------------------------
    def pump(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "state":
                    self.on_state(payload)
                elif kind == "line":
                    self.on_device_line(payload)
                elif kind == "error":
                    self.log(payload)
        except queue.Empty:
            pass
        self.update_connection_labels()
        self.root.after(50, self.pump)

    def on_device_line(self, line: str) -> None:
        if line.startswith("# STATE"):
            return
        if line.startswith("# ACK arm"):
            self.armed = True
        elif line.startswith("# ACK disarm"):
            self.armed = False
        if line.startswith(("# ACK", "# ERR", "# WARN", "# VERSION")):
            self.log(line)

    def on_state(self, state: dict) -> None:
        if "limit" in state and isinstance(state["limit"], int):
            self.limit = state["limit"]
            self.value_scale.configure(from_=-self.limit, to=self.limit)
        if "armed" in state:
            self.armed = bool(state["armed"])
        if "interval_ms" in state and isinstance(state["interval_ms"], int):
            self.interval_var.set(str(state["interval_ms"]))
        applied = state.get("value")
        if isinstance(applied, int):
            self.value_display.configure(text=describe_direction(applied))
        lines = [
            f"mode        {state.get('mode', '?')}",
            f"armed       {state.get('armed', '?')}",
            f"value       {state.get('value', '?')}",
            f"sent        {state.get('sent', '?')}",
            f"nonzero/zero {state.get('nonzero', '?')} / {state.get('zero', '?')}",
            f"rejected    {state.get('rejected', '?')}",
            f"tx_failed   {state.get('tx_failed', '?')}",
            f"twai_state  {state.get('twai_state', '?')}  pending {state.get('msgs_to_tx', '?')}",
            f"bus_errors  {state.get('bus_errors', '?')}",
            f"tx_errors   {state.get('tx_errors', '?')}",
            f"rx_missed   {state.get('rx_missed', '?')}",
            f"watchdog    {state.get('watchdog_ms', '?')} ms, stops {state.get('watchdog_stops', '?')}",
            f"interval    {state.get('interval_ms', '?')} ms",
            f"limit       {state.get('limit', '?')}",
            f"button      {state.get('button', '?')}",
            f"version     {state.get('version', '?')}",
        ]
        if state.get("run"):
            lines.insert(3, f"run_left_ms {state.get('run_left_ms', '?')}")
        self.telemetry_text.configure(state="normal")
        self.telemetry_text.delete("1.0", "end")
        self.telemetry_text.insert("end", "\n".join(lines) + "\n")
        self.telemetry_text.configure(state="disabled")

    def update_connection_labels(self) -> None:
        if self.link.is_open():
            self.conn_label.configure(text=f"已连接 {self.link.port}")
        elif self.armed:
            self.conn_label.configure(text="连接已断开")
        state_age = time.monotonic() - self.link.last_state_at if self.link.last_state_at else None
        if self.link.is_open() and state_age is not None:
            self.heartbeat_label.configure(text=f"心跳 {state_age:0.1f}s 前")
            self.heartbeat_label.configure(foreground="red" if state_age > 1.5 else "black")
        else:
            self.heartbeat_label.configure(text="")
        self.arm_label.configure(text="ARMED" if self.armed else "disarmed",
                                 foreground="red" if self.armed else "black")


def self_check() -> None:
    """Offline protocol checks. Opens no serial port."""
    state = parse_state('# STATE mode=replay armed=1 value=-16 target=-16 run=0 run_left_ms=0 '
                        'sent=399 nonzero=60 zero=339 rejected=0 tx_failed=0 twai_state=0 '
                        'msgs_to_tx=0 bus_errors=0 tx_errors=0 rx_missed=0 limit=2048 '
                        'interval_ms=50 watchdog_ms=2000 watchdog_stops=0 run_complete=1 '
                        'button=0 version="motor_control 1.0"')
    assert state["mode"] == "replay", state
    assert state["value"] == -16, state
    assert state["limit"] == 2048, state
    assert state["version"] == "motor_control 1.0", state
    assert "armed" in state and state["armed"] == 1
    assert describe_direction(-16).startswith("-16")
    assert describe_direction(0).startswith("0")
    assert describe_direction(16).startswith("+16")
    # A malformed or unrelated line must not raise.
    assert parse_state("# STATE ") == {}
    print("motor_control self-check passed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port, e.g. COM17")
    parser.add_argument("--connect", action="store_true", help="connect on start")
    parser.add_argument("--check", action="store_true", help="run offline self-checks and exit")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = parser.parse_args()

    if args.check:
        self_check()
        return 0
    if args.list:
        if list_ports is None:
            print("pyserial is not available")
            return 1
        for info in list_ports.comports():
            print(f"{info.device:8} {info.description} {info.hwid}")
        return 0
    if serial is None:
        print("pyserial is required (use the project .venv)", file=sys.stderr)
        return 1

    root = tk.Tk()
    app = MotorControlApp(root, args.port, args.connect)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.link.close(), root.destroy()))
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
