import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'arm-readback/2026-09-08'))
time.ticks_add=lambda a,b:a+b
time.ticks_diff=lambda a,b:a-b
time.ticks_ms=lambda:100
from factory.z_grab_service import GrabService
from factory import z_custom_grab as pose

class UART:
    def __init__(self,servo=False):
        self.frames=[];self.rx=b'';self.servo=servo
        self.positions=list(pose.POST_GRAB_HOLD_POSE)
    def read(self):
        result,self.rx=self.rx,b'';return result or None
    def write(self,frame):
        self.frames.append(frame)
        if self.servo and frame.endswith('PRAD!'):
            i=int(frame[1:4]);self.rx=('#%03dP%04d!'%(i,self.positions[i])).encode()
        elif self.servo and frame[5:9].isdigit() and 'T' in frame:
            i=int(frame[1:4]);p=int(frame[5:9]);self.positions[i]=1089 if i==1 and p==1123 else p
        return len(frame)

class ServiceTests(unittest.TestCase):
    def test_release_only_opens_gripper_once(self):
        link,servo=UART(),UART(True);s=GrabService(link,servo)
        s.handle('CAR,RELEASE,0008',0)
        s.handle('CAR,RELEASE,0008',1)
        for now in range(0,2000,10):
            if s.sequence is not None:s.step(now)
        self.assertEqual(s.last_result,'DONE')
        self.assertEqual(servo.frames,['#005P1200T1000!','#005PRAD!'])
        count=len(servo.frames);s.handle('CAR,RELEASE,0008',2100)
        self.assertEqual(len(servo.frames),count)
    def test_release_failure_does_not_report_done(self):
        link,servo=UART(),UART(True);s=GrabService(link,servo)
        s.handle('CAR,RELEASE,0008',0);servo.positions[5]=1700
        with self.assertRaises(ValueError):
            for now in range(0,2000,10):s.step(now)
        self.assertNotIn('ARM,DONE,0008\n',link.frames)
    def test_disconnect_stops_active_action_without_reply_or_restart(self):
        link,servo=UART(),UART(True);s=GrabService(link,servo)
        s.handle('CAR,GRAB,0002',0)
        def broken(*args): raise OSError('disconnected')
        link.read=broken;link.write=broken
        s.poll()
        self.assertIsNone(s.sequence);self.assertEqual(s.state,'IDLE')
        self.assertEqual(s.last_result,'ERROR')
        stops=[f for f in servo.frames if 'PDST' in f]
        self.assertEqual(stops,['#%03dPDST!'%i for i in range(5)])
        count=len(servo.frames);s.poll();self.assertEqual(len(servo.frames),count)
    def test_ping_and_invalid_never_move(self):
        link,servo=UART(),UART(True);s=GrabService(link,servo)
        s.handle('CAR,PING,0001',0);s.handle('CAR,GRAB,bad',0)
        self.assertFalse(servo.frames);self.assertEqual(link.frames,['ARM,PONG,0001\n'])
    def test_sequence_order_and_idempotence(self):
        link,servo=UART(),UART(True);s=GrabService(link,servo)
        s.handle('CAR,GRAB,0002',0);s.handle('CAR,GRAB,0002',0)
        s.handle('CAR,GRAB,0003',0)
        self.assertIn('ARM,BUSY,0003\n',link.frames)
        for now in range(0,15000,10):
            if s.sequence is not None:s.step(now)
        self.assertEqual(s.state,'IDLE');self.assertIn('ARM,DONE,0002\n',link.frames)
        motions=[f for f in servo.frames if f[5:9].isdigit() and 'T' in f]
        self.assertEqual(len(motions),13)
        self.assertEqual(motions[0],'#005P1200T1000!')
        self.assertEqual(motions[7],'#005P1700T1000!')
        self.assertTrue(all(not f.startswith('#005') for f in motions[8:]))
        count=len(servo.frames);s.handle('CAR,GRAB,0002',16000)
        self.assertEqual(len(servo.frames),count)
    def test_stop_retains_gripper(self):
        link,servo=UART(),UART(True);s=GrabService(link,servo)
        s.handle('CAR,GRAB,0002',0);s.handle('CAR,STOP,0003',10)
        self.assertIn('ARM,ERROR,0002\n',link.frames)
        self.assertEqual([f for f in servo.frames if 'PDST' in f],
                         ['#%03dPDST!'%i for i in range(5)])
    def test_unexpected_start_is_rejected(self):
        link,servo=UART(),UART(True);servo.positions[1]=500
        s=GrabService(link,servo);s.handle('CAR,GRAB,0002',0)
        with self.assertRaises(ValueError):
            for now in range(0,100,10):s.step(now)
        self.assertFalse([f for f in servo.frames if 'T' in f])
    def test_upright_and_variable_grip_start(self):
        for axes in ([1500]*5,list(pose.POST_GRAB_HOLD_POSE[:5])):
            link,servo=UART(),UART(True);servo.positions=axes+[1665]
            s=GrabService(link,servo);s.handle('CAR,GRAB,0004',0)
            for now in range(0,16000,10):
                if s.sequence is not None:s.step(now)
            self.assertEqual(s.last_result,'DONE')
            motions=[f for f in servo.frames if f[5:9].isdigit() and 'T' in f]
            self.assertEqual(motions[0],'#005P1200T1000!')
    def test_open_must_finish_before_descent(self):
        link,servo=UART(),UART(True);servo.positions=[1500]*6
        s=GrabService(link,servo);s.handle('CAR,GRAB,0004',0)
        for now in range(0,100,10):s.step(now)
        self.assertEqual(s.state,'OPEN_WAIT')
        servo.positions[5]=1600
        with self.assertRaises(ValueError):
            for now in range(100,2000,10):s.step(now)
        motions=[f for f in servo.frames if f[5:9].isdigit() and 'T' in f]
        self.assertEqual(motions,['#005P1200T1000!'])

if __name__=='__main__':unittest.main()
