"""Offline diagnosis of a recorded control session; never touches serial/hardware."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import statistics

from control_session import file_sha256, summarize, write_new_json


def distribution(values):
    return {'count': len(values), 'min': min(values), 'median': statistics.median(values), 'max': max(values)} if values else None


def analyze(directory):
    directory = Path(directory)
    rows = [json.loads(line) for line in (directory/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    summary = summarize(directory)
    motors = [row for row in rows if row['kind']=='rx_line' and row.get('category')=='motor']
    states = [row for row in rows if row['kind']=='rx_line' and row.get('category')=='state']
    chunks = [row for row in rows if row['kind']=='rx_chunk']
    batches = [row for row in chunks if row['bytes']==4096]
    transitions, previous = [], None
    by_chunk = defaultdict(list)
    device_intervals = []
    previous_motor = None
    for row in motors:
        fields = row['fields']
        signature = tuple(fields[key] for key in ('epoch','stage','coord'))
        if signature != previous:
            transitions.append({key:row[key] for key in ('seq','at_beijing','elapsed_s','fields')})
            previous = signature
        if fields['stage'] >= 0 and fields['t_us'] > 0:
            by_chunk[row['completed_in_chunk_seq']].append(fields['t_us'])
            if previous_motor and previous_motor['epoch']==fields['epoch'] and fields['t_us'] > previous_motor['t_us']:
                device_intervals.append((fields['t_us']-previous_motor['t_us'])/1e6)
            previous_motor = fields
    batch_spans = [(max(times)-min(times))/1e6 for times in by_chunk.values() if len(times)>1]
    command_ack = []
    for row in rows:
        if row['kind']!='tx_result' or row['command']=='status':
            continue
        command = row['command']
        # ACK has no command id. Match sequential equal command text only; repeated
        # identical requests can be ambiguous, so this is apparent host lag only.
        if command.startswith('go '):
            prefix = '# ACK go requested='+command.split()[1]+' '
        else:
            prefix = '# ACK '+command.split()[0]+' '
        ack = next((event for event in rows if event['seq']>row['seq'] and event['kind']=='rx_line' and event['line'].startswith(prefix)), None)
        command_ack.append({'tx_seq':row['seq'],'tx_id':row['tx_id'],'at_beijing':row['at_beijing'],
                            'command':command,'write_ok':row['ok'],'ack_seq':ack['seq'] if ack else None,
                            'apparent_host_ack_lag_s':ack['elapsed_s']-row['elapsed_s'] if ack else None})
    return {
        'session_directory': directory.as_posix(), 'summary_recomputed':summary,
        'motor_stage_coordinate_counts':[{'stage':stage,'coord':coord,'count':count}
            for (stage,coord),count in sorted(Counter((row['fields']['stage'],row['fields']['coord']) for row in motors).items())],
        'motor_field_transitions':transitions,
        'actual_motor_state_notifications':sum(row['fields']['stage']>=0 for row in motors),
        'adapter_invalidation_notifications':sum(row['fields']['stage']==-1 for row in motors),
        'stage3_samples':sum(row['fields']['stage']==3 for row in motors),
        'stage3_finite_samples':sum(row['fields']['stage']==3 and row['fields']['coord'] not in (-2147483648,2147483647) for row in motors),
        'rx_chunk_byte_counts':dict(Counter(str(row['bytes']) for row in chunks)),
        'successive_4096_byte_batch_intervals_s':distribution([b['received_elapsed_s']-a['received_elapsed_s'] for a,b in zip(batches,batches[1:])]),
        'same_epoch_device_motor_sample_intervals_s':distribution(device_intervals),
        'device_sample_span_within_each_host_chunk_s':distribution(batch_spans),
        'coordinate_freshness_snapshot_counts':[{'stage':stage,'fresh':fresh,'last_coord':coord,'count':count}
            for (stage,fresh,coord),count in Counter((row['snapshot']['stage'],row['snapshot']['fresh'],row['snapshot']['last_coordinate'])
                 for row in rows if row['kind']=='coordinate_snapshot').items()],
        'commands_with_apparent_ack_lag':command_ack,
        'source_hashes':{name:file_sha256(directory/name) for name in ('session.json','rx.log','events.jsonl','summary.json') if (directory/name).is_file()},
        'timing_caution':'No clock synchronization inferred. ACK lag includes firmware/USB/driver/host buffering; not mechanical response latency. Device spacing is driver dequeue timing, not bus edges.',
        'state_caution':'# MOTOR valid is computed by our adapter decoder (stage3 and finite), not a separately identified FM03 validity flag. Stage -1 is adapter invalidation, not a motor calibration stage.',
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=analyze(args.session)
    if args.output:
        write_new_json(args.output,result)
    else:
        print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
