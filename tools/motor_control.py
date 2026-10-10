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
  * Experimental internal calibration has no verified cancel command. Ordinary
    STOP/DISARM/watchdog do not guarantee stopping it; use motor power removal.

The motor must be the only other node on the bus: the RS3 must be physically
disconnected while this program transmits.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from motor_status import LiveMotor, parse_motor_line
from control_session import HOST_VERSION, SessionLog, default_log_root
from position_control import PositionController, unavailable as position_unavailable, reason_text

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
    "run_complete", "pulse", "pulse_done", "button", "version",
    "cal_pending", "cal_active_seen", "cal_attempts", "cal_sent", "cal_observed", "cal_wait_ms",
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

    def __init__(self, events: queue.Queue, log_root=None):
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
        self.log_root = Path(log_root) if log_root is not None else default_log_root()
        self.session = None
        self.last_session_dir = None
        self.logging_error = None
        self.tx_id = 0
        self.calibration_unconfirmed = False
        self.calibration_attempt_before = 0
        self.position_active = False
        self.position_stop_requested = False
        # Kept as an attribute so a test can shorten it without touching production
        # code; the shipped cadence is HEARTBEAT_MS.
        self.heartbeat_ms = HEARTBEAT_MS

    # -- lifecycle ---------------------------------------------------------
    def open(self, port: str) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed in this interpreter")
        self.close()
        self.stopping.clear()
        self.logging_error = None
        self.lines_written = self.lines_read = self.bad_lines = self.tx_id = 0
        self.last_state = {}
        self.last_state_at = self.last_rx_at = 0.0
        device = {}
        try:
            info = next((p for p in list_ports.comports() if p.device == port), None)
            if info:
                device = {key: getattr(info, key, None) for key in ('description', 'hwid', 'vid', 'pid', 'serial_number')}
        except Exception as exc:
            device = {'enumeration_error': str(exc)}
        # Refuse to start an unrecorded experiment if the folder cannot be created.
        self.session = SessionLog(self.log_root, port, BAUD, device)
        self.last_session_dir = self.session.path
        connection = None
        try:
            connection = serial.Serial(port=None, baudrate=BAUD, timeout=0.1, write_timeout=1)
            # Set control lines before opening: this must never reset the ESP32.
            connection.dtr = False
            connection.rts = False
            connection.port = port
            connection.open()
        except Exception as exc:
            if connection is not None:
                connection.close()
            self.record_event('io_error', operation='open', error=str(exc))
            self.close('open_failed')
            raise
        self.serial = connection
        self.port = port
        self.connect_error = None
        self.record_event('serial_open', port=port, dtr=False, rts=False)
        self.reader = threading.Thread(target=self._read_loop, name="motor-reader", daemon=True)
        self.reader.start()
        self.heartbeat = threading.Thread(target=self._heartbeat_loop, name="motor-heartbeat",
                                          daemon=True)
        self.heartbeat.start()
        self.send("status")

    def close(self, reason='disconnect') -> None:
        self.stopping.set()
        if self.heartbeat and self.heartbeat.is_alive():
            self.heartbeat.join(timeout=2)
        if self.reader and self.reader.is_alive():
            self.reader.join(timeout=2)
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.reader = None
        self.heartbeat = None
        if self.session and not self.session.closed:
            try:
                self.session.close(reason)
            except Exception as exc:
                self._log_error(exc)

    def _log_error(self, exc):
        if self.logging_error is None:
            self.logging_error = str(exc)
            self.events.put(('logging_error', f'实验日志保存失败：{exc}；停止本次实验并检查磁盘。'))

    def record_event(self, kind, **fields):
        if self.session is None or self.session.closed:
            return None
        try:
            return self.session.event(kind, **fields)
        except Exception as exc:
            self._log_error(exc)
            return None

    def is_open(self) -> bool:
        return self.serial is not None and getattr(self.serial, "is_open", False)

    # -- conversation ------------------------------------------------------
    def send(self, command: str, *, position_control=False) -> bool:
        if not self.is_open():
            return False
        payload = (command + "\n").encode("ascii", errors="replace")
        allowed = command.strip().lower() in ('stop', 'zero', 'disarm', 'go 0', 'set 0', 'status', 'ping', 'version', 'help')
        if (self.logging_error or self.stopping.is_set()) and not allowed:
            self.events.put(('error', f'未发送 {command}：记录或连接已中断；STOP/DISARM仍可用。'))
            self.record_event('command_blocked', command=command)
            return False
        tx_id = None
        try:
            with self.lock:
                verb = command.strip().lower().split()[0] if command.strip() else ''
                if self.position_active and not position_control and not allowed:
                    self.record_event('command_blocked', command=command, reason='position_control_active')
                    self.events.put(('command_blocked', '自动到位中，普通控制和校准已锁定；先 STOP/取消到位。'))
                    return False
                if self.position_active and not position_control and command.strip().lower() in ('stop','zero','disarm','go 0','set 0'):
                    self.position_stop_requested = True
                motion = verb in ('run', 'pulse', 'sweep', 'calibrate')
                if verb in ('go', 'set'):
                    motion = command.strip().split()[1:] != ['0']
                if motion and (self.calibration_unconfirmed or self.last_state.get('cal_pending')):
                    self.record_event('command_blocked', command=command, reason='calibration_observation_lock')
                    self.events.put(('error', '校准观察中：禁止普通非零控制和重复触发；STOP 不保证取消内部校准。'))
                    return False
                self.tx_id += 1
                tx_id = self.tx_id
                self.record_event('tx_attempt', tx_id=tx_id, command=command, payload_hex=payload.hex())
                if (self.logging_error or self.stopping.is_set()) and not allowed:
                    return False
                if verb == 'calibrate':
                    # A short write or missing reply is ambiguous; retain the guard.
                    self.calibration_attempt_before = self.last_state.get('cal_attempts', 0)
                    self.calibration_unconfirmed = True
                written = self.serial.write(payload)
                ok = written == len(payload)
                self.record_event('tx_result', tx_id=tx_id, command=command, bytes_written=written, ok=ok)
                if not ok:
                    self.events.put(('error', f'串口命令只写入 {written}/{len(payload)} 字节：{command}'))
                    return False
                self.lines_written += 1
            return True
        except Exception as exc:  # cable pulled, port closed underneath us
            self.record_event('tx_result', tx_id=tx_id, command=command, bytes_written=None, ok=False, error=str(exc))
            self.events.put(("error", f"write failed: {exc}"))
            return False

    def _read_loop(self) -> None:
        buffer = b""
        while not self.stopping.is_set():
            try:
                # Drain bytes already available. Some USB drivers batch a fixed-size
                # read despite the configured timeout; that made 100 ms telemetry
                # arrive in 4096-byte/~2 s batches and trip the 500 ms freshness gate.
                connection = self.serial
                chunk = connection.read(min(max(connection.in_waiting, 1), 4096)) if connection is not None else b""
            except Exception as exc:
                if not self.stopping.is_set():
                    self.record_event('io_error', operation='read', error=str(exc))
                    self.events.put(("error", f"read failed: {exc}"))
                    # A failed reader must not keep feeding a blind motion watchdog.
                    self.stopping.set()
                return
            if not chunk:
                continue
            self.last_rx_at = time.monotonic()
            chunk_record = None
            if self.session and not self.session.closed:
                try:
                    chunk_record = self.session.receive(chunk, self.last_rx_at)
                except Exception as exc:
                    self._log_error(exc)
            buffer += chunk
            lines = buffer.split(b"\n")
            buffer = lines.pop()
            if len(buffer) > 8192:
                self.record_event('parser_gap', discarded_partial_bytes=len(buffer))
                buffer = b""
                self.bad_lines += 1
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                fields = parse_state(line) if line.startswith('# STATE ') else parse_motor_line(line)
                category = 'state' if line.startswith('# STATE ') else 'motor' if fields is not None else 'other'
                self.record_event('rx_line', line=raw.decode('ascii', errors='replace'), category=category, fields=fields or {},
                    completed_in_chunk_seq=chunk_record['seq'] if chunk_record else None,
                    received_elapsed_s=chunk_record['received_elapsed_s'] if chunk_record else None)
                if not line:
                    continue
                self.lines_read += 1
                if line.startswith("# STATE "):
                    state = parse_state(line)
                    if state.get('cal_pending') or state.get('cal_attempts', 0) > self.calibration_attempt_before:
                        self.calibration_unconfirmed = False
                    self.last_state = state
                    self.last_state_at = time.monotonic()
                    self.events.put(("state", state))
                elif (motor := parse_motor_line(line)) is not None:
                    self.events.put(("motor", (motor,time.monotonic())))
                elif line.startswith("#"):
                    if line.startswith(('# ERR calibrate requires_arm_and_zero', '# ERR calibrate stop_ordinary_motion_first',
                                        '# ACK calibration_clear ')):
                        self.calibration_unconfirmed = False
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
        self.motor = LiveMotor()
        self.position = PositionController(lambda command: self.link.send(command, position_control=True), self.position_event)
        self.armed = False
        self.value = 0
        self.limit = DEFAULT_LIMIT
        self.last_coordinate_log = None
        self.build_ui()
        if port:
            self.port_var.set(port)
        self.refresh_ports()
        if autoconnect and self.port_var.get():
            self.root.after(200, self.connect)
        self.root.after(50, self.pump)

    # -- UI ----------------------------------------------------------------
    def build_ui(self) -> None:
        self.root.title("FM03 跟焦电机 运动控制（坐标自动到位/日志版）")
        self.root.minsize(860, 760)
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
        self.session_label = ttk.Label(conn, text='连接后自动保存完整实验日志', wraplength=800)
        self.session_label.grid(row=1, column=0, columnspan=6, sticky='w', pady=3)
        self.observation_var = tk.StringVar()
        ttk.Entry(conn, textvariable=self.observation_var, width=58).grid(row=2, column=0, columnspan=3, sticky='we')
        ttk.Button(conn, text='记录观察', command=self.record_observation).grid(row=2, column=3, padx=3)
        ttk.Button(conn, text='打开日志目录', command=self.open_log_folder).grid(row=2, column=4, padx=3)
        ttk.Label(conn, text='可填实际动作、灯语、电源、接线；未记录的现场信息保持未知。').grid(row=3, column=0, columnspan=6, sticky='w')

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
        self.calibrate_button = ttk.Button(enable, text='自动寻限位（试验）', command=self.calibrate, state='disabled')
        self.calibrate_button.grid(row=1, column=0, columnspan=2, pady=5)
        self.calibration_clear_button = ttk.Button(enable, text='确认已停止，解除观察锁', command=self.clear_calibration, state='disabled')
        self.calibration_clear_button.grid(row=1, column=2, columnspan=2)
        self.calibration_label = ttk.Label(enable, text='需要控制固件 1.2；先 ARM，再 STOP 并等待读数归零。', wraplength=800)
        self.calibration_label.grid(row=2, column=0, columnspan=4, sticky='w')
        ttk.Label(enable, text='内部自动校准可能撞机械挡块；STOP / DISARM / 看门狗不保证取消。需要中止时请切断电机电源。', wraplength=800, foreground='#a00000').grid(row=3, column=0, columnspan=4, sticky='w')

        # Value control
        control = ttk.LabelFrame(outer, text="数值控制（正负代表方向，绝对值越大转得越快）", padding=8)
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
        # Re-sends the value that is already set, without changing it. This is what
        # tests "does repeating the same value change anything"; the +/- buttons below
        # change the value instead, so the two questions stay separate.
        ttk.Button(control, text="重发当前值", command=self.resend_value).grid(row=1, column=4, padx=4)

        steps = ttk.Frame(control)
        steps.grid(row=2, column=0, columnspan=6, sticky="w", pady=(8, 0))
        ttk.Label(steps, text="直接设为").pack(side="left")
        for amount in (-256, -16, -1, 1, 16, 256):
            ttk.Button(steps, text=f"{amount:+d}", width=5,
                       command=lambda a=amount: self.set_and_send(a)).pack(side="left", padx=2)
        ttk.Label(steps, text="连续点这些按钮不会累加；要递增请用方向键或输入框。").pack(side="left", padx=8)

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

        # Pulse mode: hold the value briefly, then zero, repeated. This is how we test
        # whether one short pulse moves the motor by a repeatable amount, which is what
        # would make open-loop positioning possible without a position stream.
        pulses = ttk.LabelFrame(outer, text="脉冲模式（测「一个脉冲 = 固定位移」是否成立）", padding=8)
        pulses.pack(fill="x", pady=(8, 0))
        ttk.Label(pulses, text="保持").pack(side="left")
        self.pulse_hold_var = tk.StringVar(value="100")
        ttk.Entry(pulses, textvariable=self.pulse_hold_var, width=6).pack(side="left", padx=3)
        ttk.Label(pulses, text="ms，间隔").pack(side="left")
        self.pulse_gap_var = tk.StringVar(value="400")
        ttk.Entry(pulses, textvariable=self.pulse_gap_var, width=6).pack(side="left", padx=3)
        ttk.Label(pulses, text="ms，次数").pack(side="left")
        self.pulse_repeat_var = tk.StringVar(value="10")
        ttk.Entry(pulses, textvariable=self.pulse_repeat_var, width=6).pack(side="left", padx=3)
        ttk.Button(pulses, text="发脉冲", command=self.send_pulse).pack(side="left", padx=8)
        ttk.Button(pulses, text="低速撞限位（速度试验）", command=self.send_sweep).pack(side="left", padx=3)
        self.pulse_label = ttk.Label(pulses, text="")
        self.pulse_label.pack(side="left", padx=10)

        # Big readout
        readout = ttk.LabelFrame(outer, text="当前下发值", padding=8)
        readout.pack(fill="x", pady=(8, 0))
        self.value_display = tk.Label(readout, text=describe_direction(0),
                                      font=("", 22, "bold"), anchor="w")
        self.value_display.pack(fill="x")
        self.detail_label = ttk.Label(readout, text="", justify="left")
        self.detail_label.pack(fill="x")

        status = ttk.LabelFrame(outer,text="校准阶段与原始坐标（仅观察/记录）",padding=8)
        status.pack(fill="x",pady=(8,0))
        self.motor_label = ttk.Label(status,text="尚未收到可识别的电机状态",font=("",13,"bold"),wraplength=800)
        self.motor_label.grid(row=0,column=0,columnspan=5,sticky="w")
        self.record_a = ttk.Button(status,text="记录 A",command=lambda:self.record_coordinate('A'),state='disabled')
        self.record_a.grid(row=1,column=0,padx=3,pady=5)
        self.record_b = ttk.Button(status,text="记录 B",command=lambda:self.record_coordinate('B'),state='disabled')
        self.record_b.grid(row=1,column=1,padx=3)
        ttk.Button(status,text="导出坐标记录",command=self.export_coordinates).grid(row=1,column=2,padx=3)
        self.marks_label = ttk.Label(status,text="A：未记录    B：未记录")
        self.marks_label.grid(row=1,column=3,columnspan=2,padx=8,sticky="w")
        ttk.Label(status,text="原始坐标可能是实际位置或内部参考值；A/B 不触发运动，重新校准/中断/过期后失效。",wraplength=800).grid(row=2,column=0,columnspan=5,sticky='w')

        positioning = ttk.LabelFrame(outer, text='指定坐标自动到位（需先 ARM、普通归零和新鲜阶段 3）', padding=8)
        positioning.pack(fill='x', pady=(8,0))
        self.position_target_var = tk.StringVar(value='2048')
        self.position_tolerance_var = tk.StringVar(value='4')
        self.position_speed_var = tk.StringVar(value='64')
        self.position_timeout_var = tk.StringVar(value='60')
        for column, (label, variable) in enumerate((('目标0～4096',self.position_target_var),
                ('允许误差',self.position_tolerance_var),('最大速度',self.position_speed_var),('超时秒',self.position_timeout_var))):
            ttk.Label(positioning,text=label).grid(row=0,column=2*column,padx=3)
            ttk.Entry(positioning,textvariable=variable,width=7).grid(row=0,column=2*column+1,padx=3)
        self.position_button = ttk.Button(positioning,text='开始到位',command=self.start_position,state='disabled')
        self.position_button.grid(row=1,column=0,columnspan=2,pady=4)
        self.goto_a_button = ttk.Button(positioning,text='到 A',command=lambda:self.goto_mark('A'),state='disabled')
        self.goto_a_button.grid(row=1,column=2)
        self.goto_b_button = ttk.Button(positioning,text='到 B',command=lambda:self.goto_mark('B'),state='disabled')
        self.goto_b_button.grid(row=1,column=3)
        ttk.Button(positioning,text='取消到位并归零',command=self.stop).grid(row=1,column=4,columnspan=3)
        self.position_label = ttk.Label(positioning,text='尚未启动；允许误差是原始坐标单位，实际机械误差需复测。',wraplength=800)
        self.position_label.grid(row=2,column=0,columnspan=8,sticky='w')

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
        self.link.record_event('ui_event', message=message)
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
            self.motor.reset('重新连接')
            while not self.events.empty():
                self.events.get_nowait()
            self.link.open(port)
        except Exception as exc:
            self.link.connect_error = str(exc)
            self.log(f"连接失败：{exc}")
            messagebox.showerror("连接失败", str(exc))
            return
        self.log(f"已连接 {port} @ {BAUD}")
        self.last_coordinate_log = None
        self.session_label.configure(text=f'自动日志：{self.link.last_session_dir}', foreground='black')
        if self.observation_var.get().strip():
            self.record_observation()
        self.connect_button.configure(text="断开", command=self.disconnect)

    def disconnect(self, reason='disconnect') -> None:
        self.position.cancel(reason,self.position_context(),time.monotonic())
        self.link.position_active = False
        self.log('结束本次串口实验')
        self.motor.reset('连接已断开')
        self.update_motor_status(active=False)
        self.link.close(reason)
        self.armed = False
        self.connect_button.configure(text="连接", command=self.connect)
        self.conn_label.configure(text="未连接")
        self.log("已断开")
        if self.link.last_session_dir and not self.link.logging_error:
            self.session_label.configure(text=f'日志已保存（含自动摘要）：{self.link.last_session_dir}')

    def shutdown(self):
        self.disconnect('window_closed')
        self.root.destroy()

    def record_observation(self):
        text = self.observation_var.get().strip()
        if not text:
            messagebox.showinfo('观察备注', '填写实际动作、灯语、电源或接线，再点记录。')
            return
        if not self.link.is_open():
            messagebox.showinfo('尚未开始实验', '连接后会保存输入的工况；实验中也可以继续记录观察。')
            return
        self.link.record_event('observation', text=text, source='user', time_semantics='note click time; actual action time only if user specifies it')
        self.log(f'观察备注：{text}')
        self.observation_var.set('')

    def open_log_folder(self):
        path = self.link.last_session_dir or self.link.log_root
        try:
            path.mkdir(parents=True, exist_ok=True)
            os.startfile(str(path))
        except OSError as exc:
            messagebox.showerror('打开日志目录失败', str(exc))

    def toggle_connection(self) -> None:
        if self.link.is_open():
            self.disconnect()
        else:
            self.connect()

    # -- commands ----------------------------------------------------------
    def arm(self) -> None:
        if not self.require_link():
            return
        if not self.link.send("arm"):
            return
        self.armed = True
        self.value = 0
        self.set_value(0)
        self.log("arm：已使能，数值仍为 0，先用滑块或输入选一个值")

    def disarm(self) -> None:
        if not self.require_link():
            return
        self.position.cancel('operator_disarm',self.position_context(),time.monotonic())
        self.link.position_active = False
        self.link.send("disarm")
        self.armed = False
        self.value = 0
        self.set_value(0)
        self.log("disarm：已释放控制，数值归零")

    def stop(self) -> None:
        moving = self.position.active
        self.position.cancel('operator_stop',self.position_context(),time.monotonic())
        self.link.position_active = False
        if self.link.is_open() and not moving:
            self.link.send("stop")
        self.value = 0
        self.set_value(0)
        self.log("stop：数值归零")

    def zero(self) -> None:
        if self.position.active:
            self.stop()
            return
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
        """Adjust the value relative to the current one and send it."""
        self.set_value(self.value + amount)
        self.send_value()

    def set_and_send(self, value: int) -> None:
        """Set an absolute value and send it. Pressing this repeatedly sends the same
        value, so it cannot be mistaken for the value climbing."""
        self.set_value(value)
        self.send_value()

    def resend_value(self) -> None:
        """Send the current value again without changing it."""
        if not self.require_link():
            return
        value = self.current_value()
        if value is None:
            return
        self.link.send(f"go {value}")
        self.log(f"resend {value}")

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
        if self.position.active and value == 0:
            self.stop()
            return
        self.set_value(value)
        if value != 0 and not self.armed:
            # The firmware refuses this as well; saying so here is friendlier than
            # letting it look like the motor should be turning.
            if not messagebox.askyesno("尚未 ARM", "还没有 ARM，固件不会发送。现在 ARM 并下发吗？"):
                self.log("未发送：需要先 ARM")
                return
            self.arm()
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

    def send_pulse(self) -> None:
        """Send n identical short pulses, so the motor's resting position can be
        compared before and after: equal steps means time-based positioning works."""
        if not self.require_link():
            return
        value = self.current_value()
        if value is None:
            return
        if value == 0:
            messagebox.showwarning("数值为 0", "脉冲需要一个非零数值。")
            return
        try:
            hold = int(self.pulse_hold_var.get())
            gap = int(self.pulse_gap_var.get())
            repeat = int(self.pulse_repeat_var.get())
        except ValueError:
            messagebox.showwarning("脉冲参数无效", "保持/间隔/次数都必须是整数。")
            return
        if not 20 <= hold <= 60000 or not 20 <= gap <= 60000 or not 1 <= repeat <= 2000:
            messagebox.showwarning("脉冲参数无效",
                                   "保持与间隔需在 20..60000 ms，次数 1..2000。")
            return
        if not self.armed:
            if not messagebox.askyesno("尚未 ARM", "还没有 ARM，固件不会发送。现在 ARM 并下发吗？"):
                return
            self.arm()
        self.set_value(value)
        self.link.send(f"pulse {value} {hold} {gap} {repeat}")
        total = (hold + gap) * repeat / 1000.0
        self.log(f"pulse {value} hold={hold}ms gap={gap}ms repeat={repeat} (≈{total:.1f}s)")

    def send_sweep(self) -> None:
        """Slow continuous rotation intended to reach a mechanical hard stop, which is
        the only absolute reference available without a position stream."""
        if not self.require_link():
            return
        value = self.current_value()
        if value is None or value == 0:
            messagebox.showwarning("数值为 0", "速度试验需要一个非零数值（建议小值，如 ±16）。")
            return
        if not messagebox.askyesno(
                "低速撞限位速度试验", "这不是电机内部自动校准，也不建立已确认的软限位。\n会持续发送速度（最长约 3 分钟）。\n"
                          "确认限位能承受、且你会盯着随时 STOP 吗？"):
            return
        if not self.armed:
            self.arm()
        self.set_value(value)
        self.link.send(f"sweep {value}")
        self.log(f"sweep {value}（低速连续，撞限位后请点 STOP）")

    def calibration_ready(self, clearing=False):
        if self.position.active:
            return False
        state = self.link.last_state
        fresh = self.link.is_open() and self.link.last_state_at > 0 and time.monotonic()-self.link.last_state_at < 1.5
        supported = all(key in state for key in ('cal_pending', 'cal_attempts', 'cal_observed'))
        idle = all(state.get(key) == 0 for key in ('value', 'target', 'run', 'pulse'))
        motor = self.motor.snapshot(time.monotonic(), self.link.is_open())
        active = motor.get('fresh') and motor.get('stage') in (1, 2)
        if clearing:
            return fresh and supported and idle and not state.get('armed') and not active and bool(state.get('cal_pending') or self.link.calibration_unconfirmed)
        return fresh and supported and idle and state.get('armed') == 1 and not active and not state.get('cal_pending') and not self.link.calibration_unconfirmed

    def calibrate(self):
        if not self.calibration_ready():
            messagebox.showwarning('暂不能自动寻限位', '需要新版控制固件、新鲜状态、ARM 且普通速度/定时/脉冲全部归零；观察锁中不能重复触发。')
            return
        confirmed = messagebox.askyesno('自动寻限位候选试验',
            '确认 RS3 已脱离总线，电机在独立台架、有可承受的机械两端挡块，并由你全程看护吗？\n\n'
            '将只发送一次抓包中的自动校准启动候选，是否单包有效尚未实测。电机可能自行撞限位、回退再撞。\n'
            'STOP、DISARM 和看门狗不保证中止内部校准；需要中止请切断电机电源。')
        if not confirmed:
            return
        # Modal time is unbounded: check state again rather than trust the old readout.
        if not self.calibration_ready():
            messagebox.showwarning('状态已变化', '请等新鲜归零状态后再点击；未发送。')
            return
        self.link.record_event('calibration_request', candidate='04:2F/02 00 01', operator_confirmed=True,
                               rs3_detached_confirmed=True, mechanical_stops_confirmed=True, no_verified_cancel=True)
        self.motor.reset('用户请求重新校准')
        if self.link.send('calibrate'):
            self.log('已写入一次 calibrate；等待设备回复与阶段变化，尚未确认电机启动。')
        self.update_calibration_status()

    def clear_calibration(self):
        if not self.calibration_ready(clearing=True):
            messagebox.showwarning('暂不能解除观察锁', '先 DISARM，等待新鲜归零状态；仍有阶段 1/2 时不能解除。')
            return
        if not messagebox.askyesno('仅解除本地观察锁', '你已亲眼确认电机停止了吗？\n此操作不发送电机取消命令，不清除/恢复电机限位，也不能证明内部校准已结束。'):
            return
        if self.calibration_ready(clearing=True):
            self.link.record_event('calibration_clear_request', operator_confirmed_physically_idle=True, local_only=True)
            if self.link.send('calibration_clear'):
                self.log('已请求解除本地观察锁；等待 ACK，不是取消电机校准。')

    def update_calibration_status(self):
        state = self.link.last_state
        self.calibrate_button.configure(state='normal' if self.calibration_ready() else 'disabled')
        self.calibration_clear_button.configure(state='normal' if self.calibration_ready(clearing=True) else 'disabled')
        if not self.link.is_open() or not self.link.last_state_at or time.monotonic()-self.link.last_state_at >= 1.5:
            text = '未连接或设备状态过期；不能触发。观察锁与电机运动状态需收到新反馈后核对。'
        elif 'cal_pending' not in state:
            text = '需要控制固件 1.2；先 ARM，再 STOP 并等待读数归零。'
        elif self.link.calibration_unconfirmed:
            text = '请求已写入/结果未知，等待设备确认；不自动重试。'
        elif state.get('cal_pending'):
            text = ('已观察到阶段 1/2，等待阶段 3 有限坐标。' if state.get('cal_active_seen') else '等待阶段 1/2；未确认校准启动，不自动重试。')
            text += f" 等待 {state.get('cal_wait_ms', 0)/1000:.1f}s；普通非零控制已锁定。"
        elif state.get('cal_observed'):
            text = '已观察到校准后阶段 3；请核对机械动作、两端软限位和坐标，再记录 A/B。'
        else:
            text = '可手动试发一次自动校准候选；需要 ARM 且普通运动归零。'
        self.calibration_label.configure(text=text)

    # -- event pump --------------------------------------------------------
    def position_event(self, kind, **fields):
        self.link.record_event(kind, **fields)
        return not self.link.logging_error

    def position_context(self):
        now = time.monotonic()
        return {'motor':self.motor.snapshot(now,self.link.is_open()), 'state':self.link.last_state,
                'state_age_s':now-self.link.last_state_at if self.link.last_state_at else float('inf'),
                'state_at':self.link.last_state_at,
                'connected':self.link.is_open() and not self.link.stopping.is_set() and not self.link.logging_error,
                'calibration_unconfirmed':self.link.calibration_unconfirmed}

    def start_position(self):
        try:
            self.position.start(int(self.position_target_var.get()), self.position_context(),time.monotonic(),
                tolerance=int(self.position_tolerance_var.get()),max_speed=int(self.position_speed_var.get()),
                timeout_s=float(self.position_timeout_var.get()))
        except ValueError as exc:
            messagebox.showwarning('不能开始自动到位',str(exc))
            return
        self.link.position_stop_requested = False
        self.link.position_active = self.position.active
        self.set_value(0)
        if not self.position.active:
            self.log('自动到位未启动：日志保存失败。')
            return
        self.log(f'开始自动到位：目标{self.position.target}、误差±{self.position.tolerance}、速度上限{self.position.max_speed}')

    def goto_mark(self, marker):
        context = self.position_context()
        mark = context['motor']['marks'].get(marker)
        if not mark or not mark['valid'] or mark['epoch'] != context['motor']['epoch']:
            messagebox.showwarning('记录已失效','需要本接收段内有效A/B记录；请重新记录。')
            return
        self.position_target_var.set(str(mark['coordinate']))
        self.start_position()

    def update_position_status(self):
        context = self.position_context()
        was_active = self.position.active
        if self.link.position_stop_requested:
            self.position.cancel('operator_zero_or_stop',context,time.monotonic())
            self.link.position_stop_requested = False
        self.position.tick(context,time.monotonic())
        self.link.position_active = self.position.active
        if was_active and not self.position.active:
            result = self.position.result
            self.set_value(0)
            self.log(f"自动到位结束：{result['status']} / {result['reason']}，坐标{result['final_coordinate']}、误差{result['error']}")
        ready = not self.position.active and position_unavailable(context) is None and all(context['state'].get(k)==0 for k in ('value','target','run','pulse'))
        self.position_button.configure(state='normal' if ready else 'disabled')
        for marker,button in (('A',self.goto_a_button),('B',self.goto_b_button)):
            mark = context['motor']['marks'].get(marker)
            valid = ready and mark and mark['valid'] and mark['epoch']==context['motor']['epoch']
            button.configure(state='normal' if valid else 'disabled')
        if self.position.active:
            coordinate = context['motor']['coordinate']
            text = f"{'减速接近' if self.position.phase=='moving' else '归零/稳定确认'}：目标{self.position.target}，当前{coordinate}，允许±{self.position.tolerance}"
        elif self.position.result:
            r = self.position.result
            text = f"{'坐标到位（机械位置请复核）' if r['status']=='arrived' else '已取消/失败'}：目标{r['target']}，最终{r['final_coordinate']}，误差{r['error']}；{reason_text(r['reason'])}"
        else:
            text = '尚未启动；先 ARM、STOP 等归零。误差为原始坐标单位，不代表机械精度。'
        self.position_label.configure(text=text)

    def pump(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "state":
                    self.on_state(payload)
                elif kind == "motor":
                    self.motor.telemetry(payload[0],payload[1])
                elif kind == "line":
                    self.on_device_line(payload)
                elif kind == "error":
                    self.motor.reset('串口错误/中断')
                    self.log(payload)
                    if self.link.stopping.is_set() and self.link.is_open():
                        self.disconnect('serial_error')
                elif kind == 'logging_error':
                    self.session_label.configure(text=payload, foreground='red')
                    self.log(payload)
                elif kind == 'command_blocked':
                    self.log(payload)
        except queue.Empty:
            pass
        self.update_connection_labels()
        self.update_motor_status()
        self.update_position_status()
        self.update_calibration_status()
        self.root.after(50, self.pump)

    def on_device_line(self, line: str) -> None:
        if line.startswith("# STATE"):
            return
        if line.startswith(('# READY ', '# MODE ')):
            self.motor.reset('固件启动/模式改变')
        if line.startswith("# ACK arm"):
            self.armed = True
        elif line.startswith("# ACK disarm"):
            self.armed = False
        if line.startswith(("# ACK", "# ERR", "# WARN", "# VERSION", "# CALIB")):
            self.log(line)
        if self.position.active and line.startswith(('# ERR ', '# ERROR ', '# WARN ')):
            self.position.cancel('device_error_or_warning',self.position_context(),time.monotonic(),status='failed')
            self.link.position_active = False

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
            f"pulse       进行中 {state.get('pulse', '?')}  已完成 {state.get('pulse_done', '?')}",
            f"limit       {state.get('limit', '?')}",
            f"button      {state.get('button', '?')}",
            f"version     {state.get('version', '?')}",
            f"cal pending {state.get('cal_pending', '?')} active_seen {state.get('cal_active_seen', '?')}",
            f"cal attempts/sent/observed {state.get('cal_attempts', '?')}/{state.get('cal_sent', '?')}/{state.get('cal_observed', '?')}",
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

    def update_motor_status(self, active=None):
        state = self.motor.snapshot(time.monotonic(), self.link.is_open() if active is None else active)
        signature = json.dumps({key: state[key] for key in ('epoch','fresh','can_record','marks','records','reason')}, ensure_ascii=False, sort_keys=True)
        if signature != self.last_coordinate_log:
            self.link.record_event('coordinate_snapshot', snapshot={key:value for key,value in state.items() if key != 'history'})
            self.last_coordinate_log = signature
        labels = {0:'尚未进入阶段 3（校准状态待确认）',1:'校准阶段 1',2:'校准阶段 2',3:'校准后工作阶段候选'}
        if state['stage'] is not None:
            text = f"最后阶段 {state['stage']}：{labels.get(state['stage'],'未知阶段')}"
            if state['can_record']:
                text += f"    原始坐标 {state['coordinate']}"
                if state['observed_range_percent'] is not None:
                    text += f"  （本次观察范围 {state['observed_range_percent']:.1f}%）"
                if state['boundary']:
                    text += '    边界坐标候选'
            else:
                raw = state['last_coordinate']
                if raw in (-2147483648,2147483647):
                    text += f"\n原始字段 {raw}：无效哨兵候选，不能记录 A/B"
                elif state['stage'] != 3:
                    text += f"\n原始字段 {raw}：阶段未满足坐标有效条件，不能记录 A/B"
                else:
                    text += f"\n最后原始坐标 {raw}，当前不可记录 A/B"
            if not state['fresh']:
                text += f"\n上次接收 {state['age_s']:.1f}s 前（已过期/连接结束）" if state['age_s'] is not None else '\n状态不可用'
        else:
            text = '尚未收到可识别的电机状态（不代表固件版本旧）'
        self.motor_label.configure(text=text)
        for button in (self.record_a,self.record_b):
            button.configure(state='normal' if state['can_record'] else 'disabled')
        parts = []
        for marker in ('A','B'):
            mark = state['marks'].get(marker)
            parts.append(f"{marker}：{mark['coordinate']} ({'有效' if mark['valid'] else '已失效'})" if mark else f'{marker}：未记录')
        a,b = state['marks'].get('A'),state['marks'].get('B')
        if a and b and a['valid'] and b['valid'] and a['epoch']==b['epoch']:
            parts.append(f"B−A={b['coordinate']-a['coordinate']} 原始单位")
        self.marks_label.configure(text='    '.join(parts))

    def record_coordinate(self, marker):
        try:
            mark = self.motor.record(marker,time.monotonic(),self.link.is_open())
        except ValueError as exc:
            messagebox.showwarning('坐标不可记录',str(exc))
            return
        self.log(f"记录 {marker} 原始坐标 {mark['coordinate']}，接收段 {mark['epoch']}，不发送电机命令")
        self.link.record_event('coordinate_mark', mark=mark)
        self.update_motor_status()

    def export_coordinates(self):
        if not self.motor.records:
            messagebox.showinfo('尚无坐标记录','先在有效阶段 3 记录 A 或 B。')
            return
        stamp = datetime.now(timezone(timedelta(hours=8)))
        path = filedialog.asksaveasfilename(title='导出原始坐标记录',defaultextension='.json',
            initialfile=f'FM03_coordinates_{stamp:%Y%m%d_%H%M%S}.json',filetypes=[('JSON','*.json')])
        if not path:
            return
        payload = {'exported_at_beijing':stamp.isoformat(),'port':self.link.port,
                   'coordinate_kind':'raw position-related value; actual vs reference unresolved',
                   **self.motor.snapshot(time.monotonic(),self.link.is_open())}
        try:
            Path(path).write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        except OSError as exc:
            messagebox.showerror('导出失败',str(exc))
            return
        self.log(f'坐标记录已导出：{path}')
        self.link.record_event('coordinate_export', path=path)


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
    written = []
    controller = PositionController(lambda command: written.append(command) or True)
    position_state = dict(state, version='motor_control 1.2 calibration_trigger', value=0, target=0,
                         run=0, pulse=0, twai_state=1, cal_pending=0, cal_attempts=1)
    context = {'connected':True, 'state_age_s':0, 'state_at':100, 'state':position_state,
               'motor':{'epoch':1,'stage':3,'coordinate':2000,'can_record':True,'age_s':0,'sample_t_us':1000000}}
    controller.start(2048,context,100)
    controller.tick(context,100)
    controller.cancel('offline_self_check',context,100.1)
    assert written == ['run 2 200','stop'] and controller.phase == 'cancelled'
    import tempfile
    from control_session import summarize
    with tempfile.TemporaryDirectory() as directory:
        session = SessionLog(Path(directory), 'FAKE_SELF_CHECK', BAUD)
        session.receive(b'# CHECK fake offline bytes\r\ntail')
        session.event('observation', text='self-check only, no hardware')
        session.close('offline_self_check')
        report = summarize(session.path)
        assert report['ended_normally'] and report['integrity']['rx_offsets_cover_raw']
        assert report['integrity']['rx_unterminated_tail_bytes'] == 4
    print(f'{HOST_VERSION}: protocol and automatic log self-check passed')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port, e.g. COM17")
    parser.add_argument("--connect", action="store_true", help="connect on start")
    parser.add_argument("--check", action="store_true", help="run offline self-checks and exit")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument('--log-dir', type=Path, help='override automatic session root (default: captures/control_sessions beside project/executable)')
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
    if args.log_dir:
        app.link.log_root = args.log_dir.resolve()
    root.protocol("WM_DELETE_WINDOW", app.shutdown)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
