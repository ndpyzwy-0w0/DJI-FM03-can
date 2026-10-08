"""Compare a saved FM03 calibration capture with the observed baseline catalog.

Offline only. Reuses the project's frame-boundary parser and never opens a port.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
from statistics import median

from analyze_protocol import crc, reconstruct_frames
from analyze_state import analyze as analyze_periodic_state
from build_protocol_catalog import identity
from capture import parse_frame, summarize

ROOT = Path(__file__).resolve().parents[1]


def state09(packet):
    body = packet[11:-2]
    if packet[9:11] == b'\x04\x2f' and len(body) == 15 and body[:2] == b'\x09\x00':
        return body[2], int.from_bytes(body[3:7], 'little', signed=True)
    return None


def route(entry):
    packet = entry['packet']
    return (f"0x{entry['can_id']:03X} {packet[4]:02X}>{packet[5]:02X} "
            f"flags={packet[8]:02X} cmd={packet[9]:02X}:{packet[10]:02X}")


def details(entries, origin):
    bodies = [e['packet'][11:-2] for e in entries]
    intervals = [(b['stamp']-a['stamp'])/1000 for a,b in zip(entries,entries[1:])
                 if a['segment'] == b['segment'] and b['stamp'] > a['stamp']]
    return {'packets':len(entries), 'unique_payloads':len(set(bodies)),
            'variable_payload_offsets':[i for i in range(len(bodies[0]))
                                        if len({b[i] for b in bodies}) > 1],
            'payload_byte_values':[sorted({b[i] for b in bodies}) for i in range(len(bodies[0]))],
            'common_payloads':Counter(b.hex(' ').upper() for b in bodies).most_common(5),
            'median_interval_ms':median(intervals) if intervals else None,
            'first_relative_s':round((entries[0]['stamp']-origin)/1e6,6),
            'last_relative_s':round((entries[-1]['stamp']-origin)/1e6,6),
            'sample_packet':entries[0]['packet'].hex(' ').upper()}


def analyze(path):
    raw = path.read_bytes()
    complete = raw[:raw.rfind(b'\n')+1]
    integrity = summarize(complete)
    origin = integrity['first_t_us']
    if origin is None:
        raise ValueError('Capture contains no complete CAN frames')
    reconstruction = reconstruct_frames(raw)
    entries = reconstruction['packets']
    for e in entries:
        b = e['packet']
        assert crc(b[:3],0x77,0x8c) == b[3], 'A reconstructed header failed CRC8'
    catalog = json.loads((ROOT/'reports/20261007_protocol_catalog.json').read_text(encoding='utf-8'))
    old_groups = defaultdict(set)
    historical = []
    for source in catalog['captures']:
        reference = ROOT/'captures'/Path(source['raw_log']).name
        old_raw = reference.read_bytes()
        old_entries = reconstruct_frames(old_raw)['packets']
        states = Counter()
        dial_context = {}
        pairings = Counter()
        for e in old_entries:
            b = e['packet']
            if e['can_id'] == 0x503 and b[4:6] == b'\xe5\xbf' and b[9:11] == b'\x04\x69' and b[11:15] == bytes.fromhex('0f040819'):
                dial_context[e['segment']] = (e['stamp'], int.from_bytes(b[15:17], 'little', signed=True))
            key = identity((e['can_id'],e['extended'],e['segment'],e['stamp'],b))
            old_groups[key].add(b[11:-2])
            value = state09(b)
            if value is not None and e['can_id'] == 0x504 and b[4:6] == b'\xbf\xe5':
                states[value] += 1
                preceding = dial_context.get(e['segment'])
                if preceding and 0 <= e['stamp'] - preceding[0] <= 50000:
                    speed = preceding[1]
                    pairings[((speed > 0) - (speed < 0), value[1])] += 1
        historical.append({'file':reference.name, 'sha256':hashlib.sha256(old_raw).hexdigest(),
                           'crc_valid_packets':len(old_entries),
                           'preceding_dial_sign_vs_state09_int32': [{'dial_sign':sign,'int32_le_offset3':value,'samples':count} for (sign,value),count in sorted(pairings.items())],
                           'state09_combinations':[{'state':s,'int32_le_offset3':v,'packets':n}
                                                   for (s,v),n in sorted(states.items())]})
    groups = defaultdict(list)
    for e in entries:
        b=e['packet']
        groups[identity((e['can_id'],e['extended'],e['segment'],e['stamp'],b))].append(e)
    comparison=[]
    for key,rows in sorted(groups.items()):
        old=old_groups.get(key,set())
        new={e['packet'][11:-2] for e in rows}
        new_offsets=([i for i in range(len(next(iter(new))))
                      if {b[i] for b in new}-{b[i] for b in old}] if old else [])
        comparison.append({'route':route(rows[0]), 'payload_length':key[-2], 'subtype':key[-1],
                           **details(rows,origin), 'historical_layout_present':bool(old),
                           'historical_unique_payloads':len(old),
                           'new_byte_values_at_offsets':new_offsets,
                           'new_payload_count':len(new-old)})
    focus=[e for e in entries if e['can_id']==0x504 and e['packet'][4:6]==b'\xbf\xe5'
           and state09(e['packet']) is not None]
    runs=[]
    for e in focus:
        s,v=state09(e['packet'])
        if not runs or (s,v,e['segment']) != (runs[-1]['state'],runs[-1]['int32_le_offset3'],runs[-1]['segment']):
            runs.append({'state':s,'int32_le_offset3':v,'segment':e['segment'],
                         'first_t_us':e['stamp'],'last_t_us':e['stamp'],'packets':1,
                         'sample_packet':e['packet'].hex(' ').upper()})
        else:
            runs[-1]['last_t_us']=e['stamp']; runs[-1]['packets']+=1
    for run in runs:
        run['first_relative_s']=round((run['first_t_us']-origin)/1e6,6)
        run['last_relative_s']=round((run['last_t_us']-origin)/1e6,6)
    non_dial=[e for e in entries if e['can_id']==0x503 and e['packet'][4:6]==b'\xe5\xbf'
              and e['packet'][9:11]!=b'\x04\x69']
    commands=[]
    frames=[f for line in complete.splitlines() if (f:=parse_frame(line.decode('ascii',errors='replace')))]
    for e in non_dial:
        b=e['packet']
        fragments=[f for f in frames if f[0]>=e['stamp'] and f[2]==e['can_id']][:e['frames_used']]
        assert b''.join(f[6] for f in fragments)==b
        replies=[p for p in entries if p['segment']==e['segment'] and p['stamp']>=e['stamp']
                 and p['packet'][4:6]==b[4:6][::-1] and p['packet'][6:8]==b[6:8]
                 and p['packet'][9:11]==b[9:11] and p['packet'][8]&0x80]
        commands.append({'route':route(e),'relative_s':round((e['stamp']-origin)/1e6,6),
                         'device_t_us':e['stamp'],'sequence':int.from_bytes(b[6:8],'little'),
                         'packet':b.hex(' ').upper(),'payload':b[11:-2].hex(' ').upper(),
                         'frames':[{'device_t_us':f[0],'capture_sequence':f[1],
                                    'dlc':f[5],'data':f[6].hex(' ').upper()} for f in fragments],
                         'same_sequence_reverse_flags80_replies':len(replies)})
    dial=[]
    for key,rows in sorted(groups.items()):
        b=rows[0]['packet']
        if b[9:11]!=b'\x04\x69': continue
        values=[int.from_bytes(e['packet'][15:17],'little',signed=True) for e in rows]
        dial.append({'route':route(rows[0]),'packets':len(rows),'value_counts':dict(Counter(values)),
                     'prefix':b[11:15].hex(' ').upper(),'median_interval_ms':details(rows,origin)['median_interval_ms']})
    return {'raw_log':path.name,'sha256':hashlib.sha256(raw).hexdigest(),'raw_bytes':len(raw),
            'origin_first_complete_can_frame_t_us':origin,
            'can_frame_span_s':(integrity['last_t_us']-origin)/1e6,
            'integrity_complete_lines':integrity,'trailing_bytes':len(raw.rsplit(b'\n',1)[-1]),
            'crc8_crc16_valid_packets':len(entries),'reconstruction_stats':dict(reconstruction['stats']),
            'reliable_segments':reconstruction['segment_count'],
            'offset_convention':'zero-based payload offsets, excluding 11-byte header and trailing CRC16',
            'historical_references':historical,'groups':comparison,'dial_routes':dial,
            'periodic_state_validation':analyze_periodic_state(path),
            'e5bf_non_dial_commands':commands,'state09_bfe5_runs':runs,
            'limits':['Relative time uses the first complete parsed CAN frame, not the button press.',
                      'Physical senders, movement, limit-contact times and units are not inferred.',
                      'Zero status-counter increments do not establish perfect capture boundaries.',
                      'Packet reconstruction can reject fragments; absence is limited to accepted packets.',
                      'A finite offset3 value is a result candidate, not a proven position or travel unit.']}


def self_check():
    packet=b'\x00'*9+b'\x04\x2f'+bytes.fromhex('09 00 03 AD 0E 00 00 00 00 00 00 00 00 00 00')+b'\x00\x00'
    assert state09(packet)==(3,3757)
    negative=packet[:14]+b'\x00\x00\x00\x80'+packet[18:]
    assert state09(negative)==(3,-2147483648)
    assert state09(packet[:9]+b'\x04\x69'+packet[11:]) is None
    assert state09(packet[:-1]) is None
    print('calibration analyzer self-check passed')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path,nargs='?')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--self-check',action='store_true')
    args=parser.parse_args()
    if args.self_check: self_check(); return
    if args.log is None or args.output is None: parser.error('provide a log and --output')
    result=analyze(args.log)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('raw_log','sha256','crc8_crc16_valid_packets','e5bf_non_dial_commands','state09_bfe5_runs','dial_routes')},ensure_ascii=False,indent=2))


if __name__=='__main__': main()
