"""Read-only FM03 status and session-local A/B marks; no serial operations."""
from collections import deque
from datetime import datetime, timedelta, timezone
import re

from analyze_protocol import crc, plausible_header

FRESH_SECONDS = 0.5
BEIJING = timezone(timedelta(hours=8))


class FramePackets:
    """Whole CAN frames, dual CRC; reconsider following frames after corruption."""
    def __init__(self):
        self.frames = deque()

    def feed(self, stamp, data):
        if not data:
            return []
        self.frames.append((stamp, data))
        result = []
        while self.frames:
            length = plausible_header(self.frames[0][1])
            if length is None or len(self.frames) > 128:
                self.frames.popleft()
                continue
            parts, size = [], 0
            for t, part in self.frames:
                parts.append(part)
                size += len(part)
                if size >= length:
                    break
            if size < length:
                break
            packet = b''.join(parts)
            if size != length or crc(packet[:-2], 0x3692, 0x8408) != int.from_bytes(packet[-2:], 'little'):
                self.frames.popleft()
                continue
            result.append((self.frames[0][0], packet))
            for _ in parts:
                self.frames.popleft()
        return result


def parse_motor_line(line):
    """Firmware emits only independently CRC-verified 0x504 BF>E5 09 states."""
    if not line.startswith('# MOTOR '):
        return None
    fields = dict(re.findall(r'(\w+)=(-?\d+)', line))
    try:
        result = {k: int(fields[k]) for k in ('epoch', 't_us', 'stage', 'coord', 'valid')}
    except (KeyError, ValueError):
        return None
    if (result['epoch'] < 0 or result['t_us'] < 0 or not -1 <= result['stage'] <= 255
            or result['valid'] not in (0,1) or not -2147483648 <= result['coord'] <= 2147483647):
        return None
    return result


class LiveMotor:
    def __init__(self):
        self.epoch = -1
        self.marks = {}
        self.records = []
        self.reset('新会话')

    def _invalidate_marks(self, reason):
        for mark in self.records:
            if mark['valid']:
                mark.update(valid=False, invalid_reason=reason)

    def reset(self, reason='数据中断'):
        self._invalidate_marks(reason)
        self.epoch += 1
        self.streams = {identifier: FramePackets() for identifier in (0x504,0x503)}
        self.stage = self.coordinate = self.stamp = self.seen = None
        self.origin = self.last_frame_stamp = self.firmware_epoch = None
        self.velocity = self.velocity_seen = None
        self.boundary_samples = 0
        self.history = deque(maxlen=600)
        self.packets = 0
        self.reason = reason

    def accept(self, stage, coordinate, stamp, now):
        if self.seen is not None and now-self.seen > FRESH_SECONDS:
            self._invalidate_marks('坐标数据过期')
        if self.stamp is not None and stamp < self.stamp:
            self.reset('设备时间回退/重启')
        if self.stage == 3 and stage != 3:
            self._invalidate_marks('重新校准或阶段改变')
            self.epoch += 1
            self.history.clear()
        if coordinate in (-2147483648,2147483647):
            self._invalidate_marks('坐标哨兵无效')
        if self.stamp == stamp and self.stage == stage and self.coordinate == coordinate:
            return  # Repeated UART telemetry must not make an old sample fresh.
        self.boundary_samples = self.boundary_samples+1 if stage == 3 and coordinate in (0,4096) and coordinate == self.coordinate and self.stage == 3 else 1
        self.stage, self.coordinate, self.stamp, self.seen = stage, coordinate, stamp, now
        self.packets += 1
        if self.origin is None:
            self.origin = stamp
        if stage == 3 and coordinate not in (-2147483648,2147483647):
            self.history.append([round((stamp-self.origin)/1e6,6),coordinate])

    def feed(self, frame, now):
        stamp, _, identifier, extended, rtr, _, data = frame
        if self.last_frame_stamp is not None and stamp < self.last_frame_stamp:
            self.reset('设备时间回退/重启')
        self.last_frame_stamp = stamp
        if extended or rtr or identifier not in self.streams:
            return
        for t, packet in self.streams[identifier].feed(stamp,data):
            if identifier == 0x504 and len(packet) == 28 and packet[4:6] == b'\xbf\xe5' and packet[8:13] == bytes.fromhex('00042f0900'):
                stage = packet[13]
                self.accept(stage,int.from_bytes(packet[14:18],'little',signed=True),t,now)
            elif identifier == 0x503 and len(packet) == 19 and packet[4:6] == b'\xe5\xbf' and packet[8:15] == bytes.fromhex('0004690f040819'):
                self.velocity = int.from_bytes(packet[15:17],'little',signed=True)
                self.velocity_seen = now

    def telemetry(self, fields, now):
        if self.firmware_epoch is not None and fields['epoch'] != self.firmware_epoch:
            self.reset('固件接收段改变')
        self.firmware_epoch = fields['epoch']
        if fields['stage'] == -1:
            self._invalidate_marks('固件数据中断')
            self.stage = self.coordinate = self.stamp = self.seen = None
            return
        valid = fields['stage'] == 3 and fields['coord'] not in (-2147483648,2147483647)
        if bool(fields['valid']) != valid:
            return
        self.accept(fields['stage'],fields['coord'],fields['t_us'],now)

    def snapshot(self, now, active=True):
        age = now-self.seen if self.seen is not None else None
        device_age = ((self.last_frame_stamp-self.stamp)/1e6
                      if self.last_frame_stamp is not None and self.stamp is not None else None)
        fresh = (active and age is not None and 0 <= age <= FRESH_SECONDS
                 and (device_age is None or 0 <= device_age <= FRESH_SECONDS))
        if not fresh:
            self._invalidate_marks('坐标数据过期' if active else '连接/采集已结束')
        valid = fresh and self.stage == 3 and self.coordinate not in (-2147483648,2147483647)
        boundary = 'lower' if valid and self.coordinate == 0 else 'upper' if valid and self.coordinate == 4096 else None
        outward = (boundary == 'lower' and self.velocity is not None and self.velocity < 0 or
                   boundary == 'upper' and self.velocity is not None and self.velocity > 0)
        velocity_fresh = self.velocity_seen is not None and 0 <= now-self.velocity_seen <= .25
        return {'epoch': self.epoch, 'stage': self.stage, 'coordinate': self.coordinate if valid else None,
                'last_coordinate': self.coordinate, 'sample_t_us': self.stamp, 'age_s': round(age,3) if age is not None else None,
                'sample_device_age_s': round(device_age,3) if device_age is not None else None,
                'fresh': fresh, 'can_record': valid, 'boundary': boundary,
                'limit_candidate': bool(boundary and self.boundary_samples >= 2 and velocity_fresh and outward),
                'observed_range_percent': self.coordinate/4096*100 if valid and 0 <= self.coordinate <= 4096 else None,
                'packets': self.packets, 'history': list(self.history), 'marks': {k:dict(v) for k,v in self.marks.items()},
                'records': [dict(v) for v in self.records], 'reason': self.reason}

    def record(self, marker, now, active=True):
        if marker not in ('A','B'):
            raise ValueError('只能记录 A 或 B。')
        if not self.snapshot(now,active)['can_record']:
            raise ValueError('需要当前阶段 3 的新鲜有效坐标，才能记录。')
        mark = {'marker': marker, 'coordinate': self.coordinate, 'stage': self.stage,
                'epoch': self.epoch, 'sample_t_us': self.stamp,
                'recorded_at_beijing': datetime.now(BEIJING).isoformat(),
                'valid': True, 'invalid_reason': None}
        self.marks[marker] = mark
        self.records.append(mark)
        return dict(mark)
