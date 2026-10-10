"""Actual controller + simulated plant/fake serial. No hardware is opened."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import motor_control as mc
from position_control import PositionController
from test_motor_control import FakeSerial


def context(position=2000, now=100, sample=1000000):
    return {'connected':True, 'calibration_unconfirmed':False, 'state_age_s':0, 'state_at':now,
            'state':{'version':'motor_control 1.2 calibration_trigger', 'armed':1, 'value':0,
                'target':0, 'run':0, 'pulse':0, 'limit':2048, 'interval_ms':50, 'twai_state':1,
                'cal_pending':0, 'cal_attempts':2, 'tx_failed':0, 'rejected':0, 'rx_missed':0,
                'bus_errors':0, 'watchdog_stops':0},
            'motor':{'stage':3, 'coordinate':position, 'can_record':True, 'age_s':0,
                     'sample_t_us':sample, 'epoch':2}}


class Plant:
    """Independent simple physics: command lease, feedback every100ms, state every500ms."""
    def __init__(self, start, gain=6, delay=.05):
        self.position, self.gain, self.delay = float(start), gain, delay
        self.now, self.velocity, self.until = 100., 0, 0
        self.pending, self.written, self.events = [], [], []
        self.controller = PositionController(self.send,lambda kind,**fields:self.events.append((kind,fields)))
        self.ctx = context(start)
        self.sample = 1000000

    def send(self, command):
        self.written.append((self.now,command))
        if command=='stop':
            self.pending.append((self.now+self.delay,0,0))
        else:
            verb,value,ms = command.split()
            assert verb=='run' and ms=='200'
            self.pending.append((self.now+self.delay,int(value),self.now+int(ms)/1000))
        return True

    def run(self, target, timeout=60, slowdown=True):
        self.controller.start(target,self.ctx,self.now,timeout_s=timeout,slowdown=slowdown)
        for step in range(int(timeout*100)+10):
            self.now = 100+(step+1)/100
            while self.pending and self.pending[0][0] <= self.now+1e-8:
                _,self.velocity,self.until = self.pending.pop(0)
            if self.now >= self.until:
                self.velocity = 0
            self.position = max(0,min(4096,self.position+self.velocity*self.gain/100))
            if step%10==9:
                self.sample += 100000
                self.ctx['motor'].update(coordinate=round(self.position),sample_t_us=self.sample,age_s=0)
            else:
                self.ctx['motor']['age_s'] += .01
            if step%50==49:
                self.ctx['state'].update(value=self.velocity,target=self.velocity,run=int(self.velocity!=0))
                self.ctx['state_at']=self.now
            self.ctx['state_age_s']=self.now-self.ctx['state_at']
            self.controller.tick(self.ctx,self.now)
            if not self.controller.active:
                return self.controller.result
        raise AssertionError('controller did not finish')


class PositionTests(unittest.TestCase):
    def test_direct_mode_keeps_speed_until_tolerance_in_both_directions(self):
        for initial,near,arrived,speed in ((1000,1950,1997,64),(3000,2050,2003,-64)):
            written=[];events=[]
            controller=PositionController(lambda cmd:written.append(cmd) or True,
                                          lambda kind,**fields:events.append((kind,fields)))
            c=context(initial);controller.start(2000,c,100);controller.tick(c,100)
            c['motor'].update(coordinate=near,sample_t_us=1100000);controller.tick(c,100.1)
            self.assertEqual(written,[f'run {speed} 200']*2)
            c['motor'].update(coordinate=arrived,sample_t_us=1200000);controller.tick(c,100.2)
            self.assertEqual(written[-1],'stop')
            c['state_at']=100.3;controller.tick(c,100.3)
            for i in range(4,11):
                c['motor']['sample_t_us']+=100000;controller.tick(c,100+i*.1)
            self.assertEqual(controller.phase,'arrived')
            self.assertFalse(controller.result['slowdown'])
            self.assertFalse(events[0][1]['slowdown'])

    def test_direct_mode_stops_when_sample_skips_target_without_reverse(self):
        for initial,crossed in ((1000,2100),(3000,1900)):
            written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
            c=context(initial);controller.start(2000,c,100);controller.tick(c,100)
            c['motor'].update(coordinate=crossed,sample_t_us=1100000);controller.tick(c,100.1)
            self.assertEqual(written[-1],'stop')
            c['state_at']=100.2;controller.tick(c,100.2)
            c['motor']['sample_t_us']+=100000;controller.tick(c,100.3)
            self.assertEqual(controller.result['reason'],'direct_stop_outside_tolerance')
            self.assertEqual(controller.result['final_coordinate'],crossed)
            self.assertEqual(len([cmd for cmd in written if cmd.startswith('run ')]),1)
            self.assertEqual(controller.result['reversals'],0)

    def test_reaches_internal_and_boundary_targets_with_different_gains_and_delays(self):
        for start,target,gain,delay in ((200,3072,5.937,.05),(3700,1024,8.16,.1),
                                        (2048,0,6,.05),(2048,4096,6,.1),(200,1000,2,.05)):
            with self.subTest(start=start,target=target,gain=gain):
                p=Plant(start,gain,delay)
                result=p.run(target)
                self.assertEqual(result['status'],'arrived',result)
                self.assertLessEqual(abs(result['error']),4)
                self.assertEqual(p.velocity,0)
                speeds=[abs(int(c.split()[1])) for _,c in p.written if c.startswith('run ')]
                self.assertLessEqual(max(speeds),64)
                self.assertLess(speeds[-1],speeds[0])

    def test_stalled_coordinate_fails_and_stop_is_requested(self):
        p=Plant(1000,gain=0)
        result=p.run(2000)
        self.assertEqual(result['reason'],'no_coordinate_progress')
        self.assertEqual(p.written[-1][1],'stop')

    def test_short_deadline_fails_instead_of_claiming_arrival(self):
        p=Plant(0)
        result=p.run(4096,timeout=2)
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['reason'],'timeout')

    def test_invalid_start_conditions_never_write(self):
        changes=[('connected',False),('calibration_unconfirmed',True),('state_age_s',2)]
        contexts=[]
        for key,value in changes:
            c=context();c[key]=value;contexts.append(c)
        for key,value in (('armed',0),('cal_pending',1),('interval_ms',100),('twai_state',2),
                          ('version','motor_control 1.1 status_coordinates'),('target',16),('pulse',1)):
            c=context();c['state'][key]=value;contexts.append(c)
        for key,value in (('stage',2),('coordinate',2147483647),('age_s',.4),('can_record',False)):
            c=context();c['motor'][key]=value;contexts.append(c)
        for c in contexts:
            written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
            with self.assertRaises(ValueError):controller.start(2048,c,100)
            self.assertEqual(written,[])

    def test_active_faults_stop_without_auto_resume(self):
        for field,value,area in (('age_s',.4,'motor'),('epoch',3,'motor'),('stage',1,'motor'),
                ('cal_attempts',3,'state'),('tx_failed',1,'state'),('armed',0,'state')):
            written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
            c=context(1000);controller.start(2000,c,100,slowdown=True);controller.tick(c,100)
            c[area][field]=value;controller.tick(c,100.1)
            self.assertEqual(controller.phase,'failed')
            self.assertEqual(written[-1],'stop')
            count=len(written);controller.tick(context(1000),101)
            self.assertEqual(len(written),count)

    def test_duplicate_feedback_does_not_renew_lease(self):
        written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
        c=context(1000);controller.start(2000,c,100,slowdown=True);controller.tick(c,100)
        controller.tick(c,100.1);controller.tick(c,100.2)
        self.assertEqual(written,['run 41 200'])
        c['motor']['age_s']=.31;controller.tick(c,100.31)
        self.assertEqual(written[-1],'stop')

    def test_missing_zero_confirmation_never_reports_success(self):
        written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
        c=context(2000);controller.start(2000,c,100,slowdown=True);controller.tick(c,100)
        c['state_at']=99.9
        for i in range(1,11):
            c['motor']['sample_t_us'] += 100000;controller.tick(c,100+i*.1)
        self.assertTrue(controller.active)
        self.assertEqual(controller.phase,'settling')
        self.assertEqual(written,['stop'])

    def test_later_nonzero_state_invalidates_zero_confirmation(self):
        written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
        c=context(2000);controller.start(2000,c,100,slowdown=True);controller.tick(c,100)
        c['state_at']=100.1;controller.tick(c,100.1)
        c['state'].update(value=1,target=1,run=1)
        c['motor']['sample_t_us']+=100000
        controller.tick(c,100.2)
        self.assertEqual(controller.phase,'failed')
        self.assertEqual(controller.result['reason'],'zero_state_not_held')
        self.assertFalse(controller.result['firmware_zero_confirmed'])
        self.assertEqual(written[-1],'stop')

    def test_overshoot_waits_for_zero_before_reverse_and_caps_corrections(self):
        written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
        c=context(1000);controller.start(2000,c,100,slowdown=True);controller.tick(c,100)
        now=100
        for reversal in range(1,6):
            now+=.1
            c['motor'].update(coordinate=3000 if reversal%2 else 1000,
                              sample_t_us=c['motor']['sample_t_us']+100000)
            c['state'].update(value=41 if reversal%2 else -41,target=41 if reversal%2 else -41,run=1)
            controller.tick(c,now)
            self.assertEqual(written[-1],'stop')
            if reversal==5:
                self.assertEqual(controller.result['reason'],'too_many_direction_reversals')
                break
            count=len(written)
            now+=.1;controller.tick(c,now)
            self.assertEqual(len(written),count)  # Still waiting for firmware zero.
            now+=.1;c['state'].update(value=0,target=0,run=0);c['state_at']=now
            controller.tick(c,now)
            now+=.1;c['motor']['sample_t_us']+=100000;controller.tick(c,now)
            self.assertEqual(written[-1],f'run {"-41" if reversal%2 else "41"} 200')
        self.assertFalse(controller.active)

    def test_cancel_short_write_and_logging_failure(self):
        for operation in ('cancel','write_failure','log_failure'):
            written=[]
            def send(cmd):written.append(cmd);return operation!='write_failure' or cmd=='stop'
            def event(kind,**fields):return False if operation=='log_failure' and kind=='position_sample' else True
            controller=PositionController(send,event);c=context(1000)
            controller.start(2000,c,100,slowdown=True)
            if operation=='cancel':controller.cancel('operator_stop',c,100)
            else:controller.tick(c,100)
            self.assertFalse(controller.active)
            self.assertEqual(written[-1],'stop')

    def test_ui_pause_and_timestamp_reversal_fail_even_if_snapshot_is_fresh(self):
        for change in ('pause','timestamp'):
            written=[];controller=PositionController(lambda cmd:written.append(cmd) or True)
            c=context(1000);controller.start(2000,c,100,slowdown=True);controller.tick(c,100)
            if change=='timestamp':c['motor']['sample_t_us']-=1
            controller.tick(c,100.4 if change=='pause' else 100.1)
            self.assertEqual(controller.phase,'failed')
            self.assertEqual(written[-1],'stop')


class PositionGuiTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        patcher=patch.object(mc,'default_log_root',return_value=Path(self.temp.name))
        patcher.start();self.addCleanup(patcher.stop)
        self.root=mc.tk.Tk();self.root.withdraw();self.addCleanup(self.root.destroy)
        with patch.object(mc.list_ports,'comports',return_value=[]):self.app=mc.MotorControlApp(self.root,None,False)
        self.fake=FakeSerial();self.app.link.serial=self.fake;self.addCleanup(self.app.link.close)
        self.app.link.last_state=context()['state'];self.app.link.last_state_at=100
        self.app.motor.telemetry(dict(epoch=2,t_us=1000000,stage=3,coord=1000,valid=1),100)

    def test_start_manual_exclusion_and_zero_cancels_without_resume(self):
        app=self.app
        with patch.object(mc.time,'monotonic',return_value=100):
            app.start_position();app.update_position_status()
            self.assertTrue(app.position.active)
            self.assertTrue(self.fake.written[0].startswith('run '))
            for command in ('go 256','calibrate','arm','interval 100'):
                self.assertFalse(app.link.send(command))
            self.assertTrue(app.link.send('go 0'))
            app.update_position_status()
            self.assertFalse(app.position.active)
            self.assertEqual(app.position.phase,'cancelled')
            count=len(self.fake.written);app.update_position_status()
            self.assertEqual(len(self.fake.written),count)

    def test_slowdown_switch_is_off_by_default_and_applies_to_next_move(self):
        app=self.app
        with patch.object(mc.time,'monotonic',return_value=100):
            self.assertFalse(app.position_slowdown_var.get())
            app.start_position();app.update_position_status()
            self.assertEqual(self.fake.written[-1],'run 64 200\n')
            self.assertEqual(str(app.position_slowdown_button['state']),'disabled')
            app.stop();app.update_position_status()
            self.assertEqual(str(app.position_slowdown_button['state']),'normal')
            app.position_slowdown_var.set(True);app.start_position();app.update_position_status()
            self.assertTrue(app.position.slowdown)
            self.assertEqual(self.fake.written[-1],'run 43 200\n')

    def test_mark_validity_and_direct_zero_stop(self):
        app=self.app
        with patch.object(mc.time,'monotonic',return_value=100):
            app.record_coordinate('A');app.goto_mark('A');app.update_position_status()
            self.assertEqual(app.position.target,1000)
            app.zero();self.assertFalse(app.position.active)
        with patch.object(mc.time,'monotonic',return_value=101),patch.object(mc.messagebox,'showwarning'):
            count=len(self.fake.written);app.goto_mark('A')
            self.assertEqual(len(self.fake.written),count)

    def test_disconnect_ends_move_and_position_journal_is_summarized(self):
        from control_session import SessionLog, summarize
        app=self.app
        with patch.object(mc.time,'monotonic',return_value=100):
            app.link.session=SessionLog(Path(self.temp.name),'FAKE',mc.BAUD)
            app.link.last_session_dir=app.link.session.path
            app.start_position();app.update_position_status()
            app.disconnect()
        self.assertFalse(app.position.active)
        report=summarize(app.link.last_session_dir)
        self.assertEqual([r['kind'] for r in report['position_events']],['position_start','position_end'])
        self.assertEqual(report['position_events'][-1]['status'],'cancelled')
        self.assertEqual(report['position_events'][-1]['reason'],'disconnect')
        self.assertEqual(report['command_write_results']['stop'],1)


if __name__=='__main__':unittest.main()
