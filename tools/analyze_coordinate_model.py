"""Offline coordinate/velocity model comparison. Never opens a serial port.

Reuses the frame-boundary DUML parser. Bounds are observed extrema, not angle
units. Only a continuous, covered stage-3 stream can be simulated; calibration
motion with zero ordinary velocity is deliberately outside this model.
"""
import argparse
from bisect import bisect_right
import csv
import hashlib
import json
from math import sqrt
from pathlib import Path
from statistics import median

from analyze_calibration import state09
from analyze_protocol import reconstruct_frames
from capture import parse_frame, summarize


def clamp(value, low, high):
    return min(high, max(low, value))


def pieces(speed, a, b, delay=0):
    """ZOH intervals, command effective time = recorded first frame + delay."""
    times = [t + delay for t, v in speed]
    if a < times[0] or b > times[-1] or b < a:
        raise ValueError('Coordinate interval lacks recorded command coverage')
    i = bisect_right(times, a) - 1
    t = a
    while t < b:
        end = min(b, times[i + 1])
        if end > t:
            yield t, end, speed[i][1]
        t = end
        if t < b:
            i += 1


def impulse(speed, a, b, delay=0):
    return sum(v * (end - t) for t, end, v in pieces(speed, a, b, delay))


def fit(pairs):
    xx = sum(p['impulse'] ** 2 for p in pairs)
    if not xx:
        raise ValueError('No velocity excitation for fitting')
    return sum(p['impulse'] * p['increment'] for p in pairs) / xx


def errors(values):
    if not values:
        return None
    ordered = sorted(abs(x) for x in values)
    return {'n': len(values), 'rmse_raw': sqrt(sum(x*x for x in values)/len(values)),
            'mae_raw': sum(ordered)/len(values), 'bias_raw': sum(values)/len(values),
            'p95_abs_raw': ordered[int(.95*(len(ordered)-1))],
            'max_abs_raw': max(ordered)}


def pair_summary(pairs, gain):
    moving = [p for p in pairs if abs(p['impulse']) > 1e-8]
    return {'pairs': len(pairs), 'moving_pairs': len(moving),
            'fitted_gain': fit(moving) if moving else None,
            'moving_residual_at_reference_gain': errors([p['increment']-gain*p['impulse'] for p in moving]),
            'residual_at_reference_gain': errors([p['increment']-gain*p['impulse'] for p in pairs])}


def make_pairs(coordinates, speed, low, high, delay=0):
    result = []
    for (a, pa), (b, pb) in zip(coordinates, coordinates[1:]):
        if not low < pa < high or not low < pb < high or not .08 < b-a < .13:
            continue
        parts = list(pieces(speed, a, b, delay))
        signs = {(v > 0)-(v < 0) for t, end, v in parts if v}
        mean_abs = sum(abs(v)*(end-t) for t, end, v in parts)/(b-a)
        sign = 'positive' if signs == {1} else 'negative' if signs == {-1} else 'mixed' if signs else 'zero'
        result.append({'a_s': a, 'b_s': b, 'from_raw': pa, 'to_raw': pb,
                       'impulse': sum(v*(end-t) for t, end, v in parts),
                       'increment': pb-pa, 'direction': sign,
                       'mean_abs_raw_speed': mean_abs,
                       'speed_bin': '0' if not signs else '0..128' if mean_abs <= 128 else '128..512' if mean_abs <= 512 else '>512',
                       'constant_command': len({v for t, end, v in parts}) == 1})
    return result


def simulate(coordinates, speed, gain, low, high, delay=0):
    """One initial observation, no state reanchoring thereafter, same gain both."""
    clipped = free = coordinates[0][1]
    rows = [{'t_s': coordinates[0][0], 'observed_raw': clipped,
             'step_clamp_raw': clipped, 'output_clamp_raw': clipped,
             'unbounded_internal_raw': free}]
    for (a, _), (b, pb) in zip(coordinates, coordinates[1:]):
        for t, end, v in pieces(speed, a, b, delay):
            delta = gain * v * (end-t)
            clipped = clamp(clipped+delta, low, high)
            free += delta
        rows.append({'t_s': b, 'observed_raw': pb, 'step_clamp_raw': clipped,
                     'output_clamp_raw': clamp(free, low, high),
                     'unbounded_internal_raw': free})
    return rows


def trajectory_summary(rows):
    return {key: errors([r[key]-r['observed_raw'] for r in rows])
            for key in ('step_clamp_raw', 'output_clamp_raw')}


def fragment_timing(raw, entries, origin):
    """Time first/last fragment independently of payload meaning."""
    streams = {}
    for line in raw.split(b'\n')[:-1]:
        frame = parse_frame(line.decode('ascii', errors='replace'))
        if frame is not None:
            streams.setdefault((frame[2], frame[3]), []).append(frame)
    indices = {key: {f[0]: i for i,f in enumerate(frames)} for key,frames in streams.items()}
    spans = {'coordinate': [], 'velocity': []}
    completed_coordinates, completed_speed = [], []
    for e in entries:
        b = e['packet']
        state = state09(b)
        coordinate = e['can_id'] == 0x504 and b[4:6] == b'\xbf\xe5' and state is not None and state[0] == 3
        velocity = e['can_id'] == 0x503 and b[4:6] == b'\xe5\xbf' and b[9:11] == b'\x04\x69' and len(b) == 19 and b[11:15] == bytes.fromhex('0f040819')
        if e['extended'] or b[8] != 0 or not (coordinate or velocity):
            continue
        key = (e['can_id'], e['extended'])
        index = indices[key][e['stamp']]
        frames = streams[key][index:index+e['frames_used']]
        assert b''.join(f[6] for f in frames) == b
        end = frames[-1][0]
        kind = 'coordinate' if coordinate else 'velocity'
        spans[kind].append((end-e['stamp'])/1000)
        if coordinate:
            completed_coordinates.append(((end-origin)/1e6, state[1]))
        else:
            completed_speed.append(((end-origin)/1e6, int.from_bytes(b[15:17], 'little', signed=True)))
    summaries = {kind: {'n': len(values), 'median_ms': median(values), 'max_ms': max(values),
                        'p95_ms': sorted(values)[int(.95*(len(values)-1))]}
                 for kind,values in spans.items()}
    return summaries, completed_coordinates, completed_speed


def boundary_events(coordinates, speed, gain, low, high):
    events = []
    i = 0
    while i < len(coordinates):
        a, bound = coordinates[i]
        if bound not in (low, high):
            i += 1
            continue
        j = i
        while j+1 < len(coordinates) and coordinates[j+1][1] == bound:
            j += 1
        last = coordinates[j][0]
        if j == i or j+1 == len(coordinates):
            i = j+1
            continue
        exit_t, exit_p = coordinates[j+1]
        direction = 1 if bound == high else -1  # outward sign
        inward = next(((t, v) for t, v in speed if a <= t <= exit_t and direction*v < 0), None)
        if inward is None:
            i = j+1
            continue
        reverse, value = inward
        # Seed at the FIRST OBSERVED boundary, discarding any earlier overshoot.
        # This is a conservative local debt; continuous free integration is also
        # tested separately without reanchoring.
        debt = direction*gain*impulse(speed, a, reverse)
        local = simulate([(a, bound), (last, bound), (exit_t, exit_p)], speed, gain, low, high)
        inward_amount = -direction*gain*impulse(speed, reverse, exit_t)
        internal = bound
        free_exit = None
        for t, end, v in pieces(speed, a, coordinates[-1][0]):
            delta = gain*v*(end-t)
            if direction*(internal-bound) >= 0 and direction*(internal+delta-bound) < 0:
                free_exit = t + (bound-internal)/(gain*v)
                break
            internal += delta
        events.append({'boundary_raw': bound, 'first_boundary_s': a,
                       'last_boundary_s': last, 'boundary_samples': j-i+1,
                       'first_inward_command_s': reverse, 'first_inward_raw_speed': value,
                       'first_off_boundary_s': exit_t, 'first_off_boundary_raw': exit_p,
                       'recorded_reverse_to_first_off_ms': 1000*(exit_t-reverse),
                       'off_boundary_observation_bracket_s': [last, exit_t],
                       'outward_debt_from_first_observed_boundary_raw': debt,
                       'inward_integral_to_first_off_raw': inward_amount,
                       'local_step_clamp_at_first_off_raw': local[-1]['step_clamp_raw'],
                       'local_output_clamp_at_first_off_raw': local[-1]['output_clamp_raw'],
                       'local_free_integrator_exit_s': free_exit,
                       'local_free_exit_later_than_observation_ms': 1000*(free_exit-exit_t) if free_exit is not None else None,
                       'outward_packets_between_boundary_samples': sum(direction*v > 0 for t,v in speed if a <= t <= last)})
        i = j+1
    return events


def analyze(path):
    raw = path.read_bytes()
    integrity = summarize(raw[:raw.rfind(b'\n')+1])
    origin = integrity['first_t_us']
    if origin is None:
        raise ValueError('Capture contains no complete CAN frames')
    reconstruction = reconstruct_frames(raw)
    if reconstruction['segment_count'] != 1:
        raise ValueError('This model comparison requires one reliable segment; never bridge capture gaps')
    coords, speed, stages = [], [], []
    for e in reconstruction['packets']:
        b = e['packet']
        t = (e['stamp']-origin)/1e6
        if e['extended'] or b[8] != 0:
            continue
        state = state09(b)
        if e['can_id'] == 0x504 and b[4:6] == b'\xbf\xe5' and state is not None:
            stages.append((e['segment'], t, state[0], state[1]))
            if state[0] == 3:
                coords.append((e['segment'], t, state[1]))
        if e['can_id'] == 0x503 and b[4:6] == b'\xe5\xbf' and b[9:11] == b'\x04\x69' and len(b) == 19 and b[11:15] == bytes.fromhex('0f040819'):
            speed.append((e['segment'], t, int.from_bytes(b[15:17], 'little', signed=True)))
    if not coords or len({seg for seg, t, p in coords}) != 1:
        raise ValueError('Need one reliable stage-3 coordinate segment')
    segment = coords[0][0]
    coordinates = [(t, p) for seg, t, p in coords]
    if len(coordinates) < 2 or any(p in (-2147483648,2147483647) for t,p in coordinates):
        raise ValueError('Need multiple valid stage-3 coordinates, excluding sentinel values')
    speed = [(t, v) for seg, t, v in speed if seg == segment]
    if any(s != 3 for seg, t, s, p in stages if seg == segment and coordinates[0][0] <= t <= coordinates[-1][0]):
        raise ValueError('Calibration stage changes within simulation range')
    command_gaps = [b-a for (a,v),(b,w) in zip(speed,speed[1:]) if coordinates[0][0] <= b and a <= coordinates[-1][0]]
    state_gaps = [b-a for (a,p),(b,q) in zip(coordinates, coordinates[1:])]
    if not command_gaps or min(command_gaps) <= 0 or max(command_gaps) > .05 or min(state_gaps) <= 0 or max(state_gaps) > .15:
        raise ValueError('Unsafe ZOH extrapolation: missing or unordered command/state samples')
    low, high = min(p for t,p in coordinates), max(p for t,p in coordinates)
    cutoff = next(t for t,p in coordinates if p in (low, high))
    pairs = make_pairs(coordinates, speed, low, high)
    train = [p for p in pairs if p['b_s'] < cutoff]
    test = [p for p in pairs if p['a_s'] >= cutoff]
    gain = fit(train)
    rows = simulate(coordinates, speed, gain, low, high)
    groups = {}
    for direction in ('positive', 'negative', 'mixed', 'zero'):
        groups[direction] = pair_summary([p for p in pairs if p['direction'] == direction], gain)
    bins = {direction: {name: pair_summary([p for p in pairs if p['direction'] == direction and p['speed_bin'] == name], gain)
                        for name in ('0..128', '128..512', '>512')}
            for direction in ('positive', 'negative')}
    # Time blocks, leave one block out: no random mixing of adjacent intervals.
    moving = [p for p in pairs if abs(p['impulse']) > 1e-8]
    start, end = moving[0]['a_s'], moving[-1]['b_s']
    folds = []
    for n in range(4):
        left, right = start+(end-start)*n/4, start+(end-start)*(n+1)/4
        held = [p for p in pairs if left <= p['a_s'] and p['b_s'] <= right]
        rest = [p for p in pairs if p['b_s'] < left-.13 or p['a_s'] > right+.13]
        fitted = fit(rest)
        folds.append({'block_s': [left, right], 'excluded_adjacent_buffer_s': .13, 'training_gain': fitted,
                      'held_out': pair_summary(held, fitted)})
    # Delay is an effective timestamp alignment, not a measured motor latency.
    delay_scan = []
    for milliseconds in range(-30, 31):
        shifted = make_pairs(coordinates, speed, low, high, milliseconds/1000)
        training = [p for p in shifted if p['b_s'] < cutoff]
        testing = [p for p in shifted if p['a_s'] >= cutoff]
        k = fit(training)
        delay_scan.append({'delay_ms': milliseconds, 'training_gain': k,
                           'train_rmse_raw': pair_summary(training, k)['residual_at_reference_gain']['rmse_raw'],
                           'test_rmse_raw': pair_summary(testing, k)['residual_at_reference_gain']['rmse_raw']})
    selected = min(delay_scan, key=lambda r:r['train_rmse_raw'])
    shifted_pairs = make_pairs(coordinates, speed, low, high, selected['delay_ms']/1000)
    shifted_groups = {direction: pair_summary([p for p in shifted_pairs if p['direction'] == direction], selected['training_gain'])
                      for direction in ('positive','negative','mixed','zero')}
    shifted_bins = {direction: {name: pair_summary([p for p in shifted_pairs if p['direction'] == direction and p['speed_bin'] == name], selected['training_gain'])
                               for name in ('0..128','128..512','>512')}
                   for direction in ('positive','negative')}
    shifted_rows = simulate(coordinates, speed, selected['training_gain'], low, high, selected['delay_ms']/1000)
    for r,p in zip(rows, shifted_rows):
        r['train_selected_delay_step_clamp_raw'] = p['step_clamp_raw']
    timing, tail_coordinates, tail_speed = fragment_timing(raw, reconstruction['packets'], origin)
    # Same first-boundary sample index for all timing conventions, avoiding a
    # changed train/test membership when that boundary stamp moves.
    boundary_index = next(i for i,(t,p) in enumerate(coordinates) if p in (low,high))
    timing_models = []
    for coordinate_name, coordinate_stream in (('first',coordinates), ('last',tail_coordinates)):
        for speed_name, command_stream in (('first',speed), ('last',tail_speed)):
            boundary_time = coordinate_stream[boundary_index][0]
            ps = make_pairs(coordinate_stream,command_stream,low,high)
            training = [p for p in ps if p['b_s'] < boundary_time]
            testing = [p for p in ps if p['a_s'] >= boundary_time]
            k = fit(training)
            predicted = simulate(coordinate_stream,command_stream,k,low,high)
            timing_models.append({'coordinate_fragment': coordinate_name, 'velocity_fragment': speed_name,
                                  'train_gain': k, 'train': pair_summary(training,k), 'held_out': pair_summary(testing,k),
                                  'trajectory_held_out': trajectory_summary([r for r in predicted if r['t_s'] >= boundary_time])})
    pre = [(t, v) for t,v in speed if stages[0][1] <= t < coordinates[0][0]]
    result = {'raw_log': path.name, 'raw_sha256': hashlib.sha256(raw).hexdigest(),
              'raw_bytes': len(raw), 'crc_valid_packets': len(reconstruction['packets']),
              'reliable_segments': reconstruction['segment_count'],
              'reconstruction_stats': dict(reconstruction['stats']), 'integrity': integrity,
              'origin_first_complete_can_t_us': origin,
              'source_routes': {'coordinate': '0x504 BF>E5 flags=00 04:2F payload=09 00 stage=3 offset3..6 int32LE',
                                'velocity': '0x503 E5>BF flags=00 04:69 payload=0F 04 08 19 speed:int16LE'},
              'stage3_samples': len(coordinates), 'observed_bounds': [low, high],
              'range_s': [coordinates[0][0], coordinates[-1][0]],
              'state_gap_s': {'min': min(state_gaps), 'median': median(state_gaps), 'max': max(state_gaps)},
              'command_gap_max_s': max(command_gaps),
              'command_gap_over_15ms_count': sum(dt > .015 for dt in command_gaps),
              'observed_raw_speed_range': [min(v for t,v in speed), max(v for t,v in speed)],
              'first_to_last_fragment_spans': timing, 'timestamp_convention_sensitivity': timing_models,
              'model_definition': {'step_clamp': 'x <- clip(x + gain*v*dt) each constant-command interval',
                                   'output_clamp': 'z <- z + gain*v*dt, x=clip(z); no hidden-state reset',
                                   'initial_state': coordinates[0][1], 'reanchored_after_start': False,
                                   'bounds': 'Empirical extrema of this capture, not universal firmware constants',
                                   'gain_units': 'raw coordinate/(raw speed*second)',
                                   'gain_fit': 'Zero-intercept increment fit before first observed boundary; same gain for both models',
                                   'timestamp': 'First fragment driver-dequeue time; ZOH, not hardware edge or mechanical time'},
              'pre_stage3_ordinary_speed_nonzero_packets': sum(v != 0 for t,v in pre),
              'pre_stage3_model_applicable': False,
              'training_cutoff_first_observed_boundary_s': cutoff,
              'training_gain': gain, 'all_interior_gain_descriptive_only': fit(pairs),
              'train_direction_counts': {d: sum(p['direction'] == d for p in train) for d in ('positive','negative','mixed','zero')},
              'train_pairs': pair_summary(train, gain), 'held_out_pairs': pair_summary(test, gain),
              'direction_groups': groups, 'direction_speed_bins': bins, 'time_block_cross_validation': folds,
              'trajectory_all': trajectory_summary(rows),
              'trajectory_held_out': trajectory_summary([r for r in rows if r['t_s'] >= cutoff]),
              'trajectory_moving_or_changed_samples': trajectory_summary([r for i,r in enumerate(rows) if i and
                      (r['observed_raw'] != rows[i-1]['observed_raw'] or abs(impulse(speed,rows[i-1]['t_s'],r['t_s'])) > 1e-8)]),
              'boundary_reversals': boundary_events(coordinates, speed, gain, low, high),
              'delay_scan_train_only_selection': delay_scan,
              'train_selected_effective_alignment': {**selected, 'trajectory_all': trajectory_summary(shifted_rows),
                    'trajectory_held_out': trajectory_summary([r for r in shifted_rows if r['t_s'] >= cutoff]),
                    'direction_groups': shifted_groups, 'direction_speed_bins': shifted_bins},
              'limits': ['Only calibrated stage-3 motion modeled, not internal calibration.',
                         'Empirical coordinate behavior cannot identify actual vs target position or physical executing MCU.',
                         'Effective alignment scan combines software timing, ZOH approximation and any physical dynamics.',
                         'Repeated same-capture holdouts are internal consistency, not independent hardware validation.',
                         'State sampling about 100ms limits reversal onset resolution; delay is not mechanical precision.',
                         'No independent cancel command, physical angle scale or reset persistence established.']}
    return result, rows, pairs


def self_check():
    speed = [(0,1),(2,-1),(4,0),(5,0)]
    coords = [(0,8),(1,10),(2,10),(3,8),(4,6)]
    rows = simulate(coords,speed,2,0,10)
    assert [r['step_clamp_raw'] for r in rows] == [8,10,10,8,6]
    assert [r['output_clamp_raw'] for r in rows] == [8,10,10,10,8]
    event = boundary_events(coords,speed,2,0,10)[0]
    assert event['outward_debt_from_first_observed_boundary_raw'] == 2
    assert event['local_free_integrator_exit_s'] == 3
    mirrored = simulate([(t,10-p) for t,p in coords],[(t,-v) for t,v in speed],2,0,10)
    assert [r['step_clamp_raw'] for r in mirrored] == [2,0,0,2,4]
    assert abs(impulse([(0,3),(1,5),(2,0)],.5,1.5)-4) < 1e-8
    assert impulse([(0,3),(1,5),(2,0)],.5,1.5,.1) == 3.8
    assert fit([{'impulse':2,'increment':6},{'impulse':-2,'increment':-6}]) == 3
    try:
        impulse(speed,-.1,1)
    except ValueError:
        pass
    else:
        raise AssertionError('Missing command coverage was extrapolated')
    print('coordinate model self-check passed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path, nargs='?')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-check', action='store_true')
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if args.log is None or args.output is None:
        parser.error('provide log and --output')
    if args.output.resolve() == args.log.resolve() or args.output.parent.resolve() == args.log.parent.resolve():
        parser.error('Analysis outputs must be separate from the raw capture directory')
    result, rows, pairs = analyze(args.log)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    for suffix, data in (('_trajectory.csv',rows), ('_intervals.csv',pairs)):
        with args.output.with_name(args.output.stem+suffix).open('w',newline='',encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)
    print(json.dumps({key:result[key] for key in ('training_gain','train_pairs','held_out_pairs','trajectory_all','trajectory_held_out','boundary_reversals','train_selected_effective_alignment')}, indent=2))


if __name__ == '__main__':
    main()
