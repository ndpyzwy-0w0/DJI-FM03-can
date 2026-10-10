"""Offline UART calibration evidence; never opens hardware or edits source logs."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
from control_session import summarize, write_new_json


def analyze(directory):
    directory = Path(directory)
    rows = [json.loads(line) for line in (directory/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    summary = summarize(directory)
    requests = [row for row in rows if row['kind']=='tx_result' and row['command']=='calibrate']
    if len(requests) != 1:
        raise ValueError('This analysis requires exactly one recorded calibrate attempt.')
    request = requests[0]
    motors = [row for row in rows if row['kind']=='rx_line' and row.get('category')=='motor']
    phases = []
    for row in motors:
        if row['seq'] > request['seq'] and (not phases or phases[-1]['stage'] != row['fields']['stage']):
            phases.append({'stage':row['fields']['stage'], 'seq':row['seq'],
                           'elapsed_s':row['elapsed_s'], 'at_beijing':row['at_beijing'],
                           't_us':row['fields']['t_us'], 'coordinate':row['fields']['coord']})
    completed = next((p for p in phases if p['stage']==3 and p['coordinate'] not in (-2147483648,2147483647)),None)
    during = [row['fields'] for row in rows if row['kind']=='rx_line' and row.get('category')=='state'
              and row['seq'] > request['seq'] and completed and row['seq'] < completed['seq']]
    platforms, state, group = [], None, None
    slopes, previous_interior = {}, None
    for row in rows:
        if row['kind']=='tx_attempt' and row['command'].split()[0] in ('go','set','run','pulse','sweep','stop','disarm','calibrate'):
            group = None
            previous_interior = None
        if row['kind']!='rx_line':
            continue
        if row.get('category')=='state':
            state = row
            continue
        if row.get('category')!='motor':
            continue
        f = row['fields']
        s = state['fields'] if state else {}
        value = s.get('value')
        fresh_state = state is not None and 0 <= row['elapsed_s']-state['elapsed_s'] <= .6
        interior = (f['stage']==3 and 0<f['coord']<4096 and fresh_state and
                    s.get('armed')==1 and isinstance(value,int) and value!=0 and s.get('target')==value)
        if interior and previous_interior and previous_interior[1]==value:
            previous = previous_interior[0]
            delta_s = (f['t_us']-previous['t_us'])/1e6
            if f['epoch']==previous['epoch'] and .08 <= delta_s <= .13:
                slopes.setdefault(str(value),[]).append((f['coord']-previous['coord'])/value/delta_s)
        previous_interior = (f,value) if interior else None
        boundary = f['coord'] if f['stage']==3 and f['coord'] in (0,4096) else None
        outward = ((boundary==0 and s.get('value',0)<0 and s.get('target',0)<0) or
                   (boundary==4096 and s.get('value',0)>0 and s.get('target',0)>0))
        eligible = outward and s.get('armed')==1 and 0 <= row['elapsed_s']-state['elapsed_s'] <= .6
        if not eligible:
            group = None
            continue
        if group is None or group['coordinate']!=boundary or group['epoch']!=f['epoch'] or not 0 < f['t_us']-group['last_t_us'] <= 200000:
            group = {'coordinate':boundary, 'epoch':f['epoch'], 'first_seq':row['seq'],
                     'first_elapsed_s':row['elapsed_s'], 'first_t_us':f['t_us'], 'samples':0}
            platforms.append(group)
        group.update(last_seq=row['seq'], last_elapsed_s=row['elapsed_s'], last_t_us=f['t_us'], value=s['value'])
        group['samples'] += 1
        group['observed_driver_span_s'] = (group['last_t_us']-group['first_t_us'])/1e6
    platforms = [g for g in platforms if g['samples'] >= 2]
    return {'session':directory.as_posix(), 'request':request, 'phases_after_request':phases,
            'host_request_to_first_stage3_s':completed['elapsed_s']-request['elapsed_s'] if completed else None,
            'stage_sample_counts':dict(Counter(str(row['fields']['stage']) for row in motors)),
            'calibration_state_samples':len(during),
            'calibration_ordinary_value_target_all_zero':bool(during) and all(s.get('value')==s.get('target')==0 for s in during),
            'outward_boundary_platforms':platforms,
            'interior_coordinate_gain_by_held_value':{value:{'samples':len(gains), 'median':statistics.median(gains),
                'min':min(gains), 'max':max(gains)} for value,gains in slopes.items()},
            'summary':summary,
            'limits':['Requires exactly one attempt; STATE-held outward value/target classified within 600ms, contiguous MOTOR spacing <=200ms.',
                      'Interior gains use finite 0<coordinate<4096 pairs, matching armed value/target, 80..130ms driver intervals; commands split pairs. These are coordinate units/(raw speed unit*second), not mechanical precision.',
                      'UART fields are adapter telemetry, not an independent complete CAN capture or mechanical observation.',
                      'Host elapsed clock/buffering and driver dequeue time are not physical bus or mechanical timing.',
                      'Current request was made within armed request/zero-speed traffic; necessity of those background packets unknown.',
                      'Coordinate clipping does not prove true encoder position, physical blockage detection, cancellation, or persistent limits.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    write_new_json(args.output,analyze(args.session))
