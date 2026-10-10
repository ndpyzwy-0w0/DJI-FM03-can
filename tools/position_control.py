"""FM03 coordinate feedback controller using existing UART `run <v> 200`.

No GUI/serial imports. Call start(context, now), then tick(context, now).
The supplied send callback returns UART write success, not motor execution.
"""
import math

LEASE_MS = 200
FINE_LEASE_MS = 100
FEEDBACK_MAX_AGE_S = .3
STATE_MAX_AGE_S = 1.0
SETTLE_S = .4
COUNTERS = ('tx_failed', 'rejected', 'rx_missed', 'bus_errors', 'watchdog_stops')
REASONS = {
    'connection_or_logging_unavailable':'连接或日志不可用', 'calibration_pending':'仍在校准观察中',
    'not_armed':'尚未ARM或已释放控制', 'state_stale':'设备状态过期',
    'requires_control_firmware_1_2':'需要控制固件1.2',
    'bus_not_running_or_interval_above_50ms':'总线未运行，或拨轮周期不在10～50ms',
    'coordinate_stale_or_invalid':'坐标过期或未进入有效阶段3',
    'coordinate_outside_observed_range':'坐标超出当前观察范围', 'coordinate_timestamp_invalid':'坐标时间无效',
    'calibration_or_feedback_epoch_changed':'校准次数或坐标接收段改变', 'device_counter_changed':'设备错误/丢帧计数变化',
    'timeout':'超过到位时限', 'control_loop_gap':'控制循环中断', 'feedback_time_reversed':'坐标时间倒退',
    'no_coordinate_progress':'坐标持续没有进展', 'too_many_direction_reversals':'反向修正次数过多',
    'stop_write_failed':'归零命令写入失败', 'motion_write_failed':'运动命令写入失败',
    'zero_state_not_held':'归零确认后普通运动状态再次变为非零',
    'direct_stop_outside_tolerance':'直接停止后坐标超出允许误差，请降低速度或开启接近减速',
    'position_log_failed':'到位日志保存失败', 'device_error_or_warning':'设备报告错误或警告',
    'operator_stop':'手动STOP', 'operator_disarm':'手动DISARM', 'operator_zero_or_stop':'手动归零/停止',
    'disconnect':'串口断开', 'window_closed':'窗口关闭',
    'coordinate_in_tolerance_and_stable':'坐标在允许误差内且已稳定',
    'coordinate_exact_and_stable':'坐标精确等于目标且归零后已稳定',
}


def reason_text(reason):
    return REASONS.get(reason, reason)


def unavailable(context):
    motor, state = context['motor'], context['state']
    if not context.get('connected'):
        return 'connection_or_logging_unavailable'
    if context.get('calibration_unconfirmed') or state.get('cal_pending'):
        return 'calibration_pending'
    if state.get('armed') != 1:
        return 'not_armed'
    if context.get('state_age_s', math.inf) > STATE_MAX_AGE_S:
        return 'state_stale'
    if not isinstance(state.get('version'), str) or not state['version'].startswith('motor_control 1.2 '):
        return 'requires_control_firmware_1_2'
    if state.get('twai_state') != 1 or not 10 <= state.get('interval_ms', 0) <= 50:
        return 'bus_not_running_or_interval_above_50ms'
    if not motor.get('can_record') or motor.get('stage') != 3 or motor.get('age_s') is None or motor['age_s'] > FEEDBACK_MAX_AGE_S:
        return 'coordinate_stale_or_invalid'
    if not isinstance(motor.get('coordinate'), int) or not 0 <= motor['coordinate'] <= 4096:
        return 'coordinate_outside_observed_range'
    if not isinstance(motor.get('sample_t_us'), int) or motor['sample_t_us'] <= 0:
        return 'coordinate_timestamp_invalid'
    return None


class PositionController:
    def __init__(self, send, event=lambda *args, **kwargs: None):
        self.send, self.event = send, event
        self.active = False
        self.phase = 'idle'
        self.move_id = 0
        self.result = None

    def start(self, target, context, now, tolerance=4, max_speed=64, timeout_s=60, slowdown=False,
              strict=False, min_speed=1):
        if self.active:
            raise ValueError('自动到位仍在进行，请先取消。')
        if type(target) is not int or not 0 <= target <= 4096:
            raise ValueError('目标坐标必须是0～4096的整数（当前观察范围）。')
        if type(tolerance) is not int or not 1 <= tolerance <= 64:
            raise ValueError('允许坐标误差必须是1～64的整数。')
        if type(max_speed) is not int or not 1 <= max_speed <= min(256, context['state'].get('limit', 0)):
            raise ValueError('最大速度必须为1～256，且不超过固件当前limit。')
        if not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or not 2 <= timeout_s <= 120:
            raise ValueError('超时必须为2～120秒。')
        if type(slowdown) is not bool:
            raise ValueError('接近目标减速必须是布尔开关。')
        if type(strict) is not bool:
            raise ValueError('严格到点必须是布尔开关。')
        if type(min_speed) is not int or not 1 <= min_speed <= max_speed:
            raise ValueError('最低速度必须为1～最大速度的整数。')
        reason = unavailable(context)
        if reason:
            raise ValueError('不能启动自动到位：'+reason_text(reason))
        if any(context['state'].get(key) != 0 for key in ('value', 'target', 'run', 'pulse')):
            raise ValueError('请先STOP并等普通速度、定时和脉冲状态全部归零。')
        self.move_id += 1
        self.active, self.phase, self.result = True, 'moving', None
        self.target, self.tolerance, self.max_speed = target, 0 if strict else tolerance, max_speed
        self.approach_window, self.strict, self.min_speed = tolerance, strict, min_speed
        self.slowdown = slowdown
        self.fine_control = self.fine_pulse_pending = False
        self.fine_started = False
        self.fine_pulses = 0
        self.started, self.deadline = now, now+timeout_s
        self.last_tick_at = now
        self.epoch = context['motor']['epoch']
        self.cal_attempts = context['state'].get('cal_attempts')
        self.counters = {k: context['state'].get(k) for k in COUNTERS}
        self.last_sample = None
        self.last_nonzero = 0
        self.reversals = 0
        self.best_error = abs(target-context['motor']['coordinate'])
        self.progress_at = now
        self.zero_at = self.zero_confirmed_at = self.stable_since = None
        self.stable_coordinates = []
        recorded = self.event('position_start', move_id=self.move_id, target=target, tolerance=tolerance,
                   effective_tolerance=self.tolerance, strict=strict, min_speed=min_speed,
                   max_speed=max_speed, timeout_s=timeout_s, slowdown=slowdown, initial_coordinate=context['motor']['coordinate'],
                   epoch=self.epoch, lease_ms=LEASE_MS, fine_lease_ms=FINE_LEASE_MS)
        if recorded is False:
            self._finish('failed', 'position_log_failed', context, now)

    def _finish(self, status, reason, context, now, stop=True):
        self.active, self.phase = False, status
        coordinate = context['motor'].get('coordinate')
        self.result = dict(move_id=self.move_id, status=status, reason=reason, target=self.target,
                           final_coordinate=coordinate, error=coordinate-self.target if coordinate is not None else None,
                           elapsed_s=round(now-self.started, 3), tolerance=self.tolerance, slowdown=self.slowdown,
                           strict=self.strict, min_speed=self.min_speed, fine_pulses=self.fine_pulses, reversals=self.reversals,
                           firmware_zero_confirmed=self.zero_confirmed_at is not None and all(context['state'].get(k)==0 for k in ('value','target','run','pulse')),
                           stop_write_ok=self.send('stop') if stop else None,
                           coordinate_success_only=True)
        self.event('position_end', **self.result)

    def cancel(self, reason, context, now, status='cancelled'):
        if self.active:
            self._finish(status, reason, context, now)

    def _zero(self, context, now):
        if not self.send('stop'):
            self._finish('failed', 'stop_write_failed', context, now)
            return
        self.phase, self.zero_at = 'settling', now
        self.fine_pulse_pending = False
        if self.strict:
            self.last_nonzero = 0
        self.zero_confirmed_at = self.stable_since = None
        self.stable_coordinates = []

    def tick(self, context, now):
        if not self.active:
            return
        motor, state = context['motor'], context['state']
        reason = unavailable(context)
        if now-self.last_tick_at > .35:
            reason = 'control_loop_gap'
        self.last_tick_at = now
        if motor.get('epoch') != self.epoch or state.get('cal_attempts') != self.cal_attempts:
            reason = 'calibration_or_feedback_epoch_changed'
        if any(state.get(k) != self.counters[k] for k in COUNTERS):
            reason = 'device_counter_changed'
        if now >= self.deadline:
            reason = 'timeout'
        if reason:
            self._finish('failed', reason, context, now)
            return
        if self.phase == 'settling' and self.zero_confirmed_at is not None and any(state.get(k)!=0 for k in ('value','target','run','pulse')):
            self._finish('failed', 'zero_state_not_held', context, now)
            return
        if self.phase == 'settling' and self.zero_confirmed_at is None:
            if context['state_at'] >= self.zero_at and all(state.get(k)==0 for k in ('value','target','run','pulse')):
                self.zero_confirmed_at = now
                self.last_sample = motor['sample_t_us']
            return  # Wait for a distinct MOTOR sample after zero confirmation.
        sample = motor['sample_t_us']
        if self.last_sample is not None and sample < self.last_sample:
            self._finish('failed', 'feedback_time_reversed', context, now)
            return
        if sample == self.last_sample:
            return  # Duplicate data must never renew a motion lease or settle count.
        self.last_sample = sample
        coordinate = motor['coordinate']
        error = self.target-coordinate
        if self.event('position_sample', move_id=self.move_id, coordinate=coordinate, error=error,
                      phase=self.phase, fine_control=self.fine_control, sample_t_us=sample, epoch=self.epoch) is False:
            self._finish('failed', 'position_log_failed', context, now)
            return
        if abs(error) < self.best_error:
            self.best_error, self.progress_at = abs(error), now
        if self.phase == 'settling':
            if abs(error) <= self.tolerance:
                self.stable_since = now if self.stable_since is None else self.stable_since
                self.stable_coordinates.append(coordinate)
                if max(self.stable_coordinates)-min(self.stable_coordinates) > self.tolerance:
                    self.stable_since, self.stable_coordinates = now, [coordinate]
                if now-self.stable_since >= SETTLE_S and len(self.stable_coordinates) >= 4:
                    self._finish('arrived', 'coordinate_exact_and_stable' if self.strict else 'coordinate_in_tolerance_and_stable', context, now, stop=False)
                return
            if not self.slowdown and not self.strict:
                self._finish('failed', 'direct_stop_outside_tolerance', context, now)
                return  # Direct stop never reverses automatically to chase an overshoot.
            self.phase = 'moving'
            if self.strict:
                self.fine_control = True
                # Rebase after the initial braking drift, or drift from an exact reading.
                if not self.fine_started or self.best_error == 0:
                    self.progress_at, self.best_error = now, abs(error)
                self.fine_started = True
            else:
                self.progress_at, self.best_error = now, abs(error)
        if abs(error) <= self.tolerance:
            self._zero(context, now)
            return
        if self.strict and not self.fine_control and abs(error) <= self.approach_window:
            self.fine_control = True
            self._zero(context, now)
            return
        if self.fine_pulse_pending:
            self._zero(context, now)
            return  # One short correction, then confirm zero and inspect new feedback.
        if now-self.progress_at > 3:
            self._finish('failed', 'no_coordinate_progress', context, now)
            return
        direction = 1 if error > 0 else -1
        if self.last_nonzero and direction != self.last_nonzero:
            if self.strict:
                self.reversals += 1
                self.fine_control = True
                self._zero(context, now)
                return
            if not self.slowdown:
                self._zero(context, now)
                return  # The sample skipped across the target; stop immediately.
            self.reversals += 1
            self.last_nonzero = 0
            if self.reversals > 4:
                self._finish('failed', 'too_many_direction_reversals', context, now)
            else:
                self._zero(context, now)
            return
        magnitude = self.min_speed if self.fine_control else self.max_speed
        if self.slowdown and not self.fine_control:
            magnitude = max(self.min_speed, min(self.max_speed, abs(error)//24))
        speed = direction*magnitude
        short_correction = self.strict and self.fine_control and abs(error) <= 2
        duration = FINE_LEASE_MS if short_correction else LEASE_MS
        if self.send(f'run {speed} {duration}'):
            self.last_nonzero = direction
            if short_correction:
                self.fine_pulses += 1
                self.fine_pulse_pending = True
        else:
            self._finish('failed', 'motion_write_failed', context, now)
