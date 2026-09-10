"""Explicit-command grab service. Construction and PING never move servos.

Dedicated Wi-Fi TCP connection to the car access point.
The application must call poll() regularly. Never run alongside z_main's
action scheduler: both would otherwise own servo UART2.
"""
import time
from factory import z_custom_grab as pose


class GrabService:
    def __init__(self, link_uart, servo_uart):
        self.link = link_uart
        self.servo = servo_uart
        self.buffer = b''
        self.sequence = None
        self.last_sequence = None
        self.last_result = None
        self.state = 'IDLE'
        self.index = 0
        self.deadline = 0
        self.reply = b''
        self.positions = []

    def emit(self, kind, sequence):
        self.link.write('ARM,%s,%s\n' % (kind, sequence))

    def stop(self):
        # Retain gripper torque/closure while halting the five arm axes.
        for i in range(5):
            self.servo.write('#%03dPDST!' % i)

    def finish(self, result):
        self.last_sequence, self.last_result = self.sequence, result
        sequence = self.sequence
        self.sequence = None
        self.state = 'IDLE'
        self.emit(result, sequence)

    def handle(self, line, now):
        parts = line.split(',')
        if len(parts) != 3 or parts[0] != 'CAR':
            return
        command, sequence = parts[1:]
        if len(sequence) != 4 or not sequence.isdigit() or sequence == '0000':
            return
        if command == 'PING':
            if self.sequence is None:
                # A fresh explicit car run starts with PING. Cached GRAB
                # replies are retained until that new run, including retries.
                self.last_sequence = self.last_result = None
            self.emit('PONG', sequence)
        elif command == 'STOP':
            if self.sequence is not None:
                self.stop()
                self.finish('ERROR')
            self.emit('ACK', sequence)
        elif command in ('GRAB','RELEASE'):
            if sequence == self.sequence:
                self.emit('ACK', sequence)
            elif sequence == self.last_sequence:
                self.emit(self.last_result, sequence)
            elif self.sequence is not None:
                self.emit('BUSY', sequence)
            else:
                self.sequence = sequence
                self.emit('ACK', sequence)
                if command == 'RELEASE':
                    frame = '#005P%04dT1000!' % pose.PICKUP_OPEN_POSE[5]
                    if self.servo.write(frame) != len(frame):
                        raise ValueError('Short release write')
                    self.state = 'RELEASE_WAIT'
                    self.deadline = time.ticks_add(now,1500)
                else:
                    self.begin_read('START', now)

    def begin_read(self, phase, now):
        self.phase, self.index, self.positions = phase, 0, []
        if phase in ('OPEN','RELEASE'):
            self.index = 5
        self.query(now)

    def query(self, now):
        self.servo.read()
        self.reply = b''
        self.servo.write('#%03dPRAD!' % self.index)
        self.deadline = time.ticks_add(now, 500)
        self.state = 'READ'

    def begin_pose(self, values, duration, phase, now):
        self.values, self.duration, self.phase = values, duration, phase
        self.index = 0
        self.state = 'SEND'
        self.deadline = now

    def step(self, now):
        if self.state == 'READ':
            self.reply += self.servo.read() or b''
            prefix = ('#%03dP' % self.index).encode()
            start = self.reply.find(prefix)
            end = self.reply.find(b'!', start) if start >= 0 else -1
            if end >= 0:
                value = self.reply[start+len(prefix):end]
                if not value.isdigit():
                    raise ValueError('Malformed position')
                position = int(value)
                if not 500 <= position <= 2500:
                    raise ValueError('Invalid position range')
                self.positions.append(position)
                self.index += 1
                count = 5 if self.phase == 'FINAL' else 6
                if self.index < count:
                    self.query(now)
                    return
                if self.phase == 'START':
                    axes = self.positions[:5]
                    starts = ((1500,)*5, pose.POST_GRAB_HOLD_POSE[:5])
                    if not any(all(abs(a-b)<=40 for a,b in zip(axes,start))
                               for start in starts):
                        raise ValueError('Unsupported start pose')
                    # Grip width varies with the object; reopen before descent.
                    frame = '#005P%04dT1000!' % pose.PICKUP_OPEN_POSE[5]
                    if self.servo.write(frame) != len(frame):
                        raise ValueError('Short open write')
                    self.state = 'OPEN_WAIT'
                    self.deadline = time.ticks_add(now,1500)
                    return
                if self.phase == 'RELEASE':
                    if abs(self.positions[0]-pose.PICKUP_OPEN_POSE[5])>40:
                        raise ValueError('Release not reached')
                    self.finish('DONE')
                    return
                if self.phase == 'OPEN':
                    if abs(self.positions[0]-pose.PICKUP_OPEN_POSE[5])>40:
                        raise ValueError('Gripper not open')
                    self.begin_pose(pose.PICKUP_OPEN_POSE,3000,'LOW',now)
                    return
                expected = (pose.POST_GRAB_HOLD_POSE if self.phase == 'FINAL'
                            else pose.PICKUP_OPEN_POSE)
                if any(abs(a-b)>40 for a,b in zip(self.positions,expected)):
                    raise ValueError('Position not reached')
                if self.phase == 'LOW':
                    frame = pose.close_gripper_command()
                    if self.servo.write(frame) != len(frame):
                        raise ValueError('Short gripper write')
                    self.state = 'CLOSE_WAIT'
                    self.deadline = time.ticks_add(now,1500)
                else:
                    self.finish('DONE')
            elif len(self.reply)>128 or time.ticks_diff(now,self.deadline)>=0:
                raise ValueError('Position query timeout')
        elif self.state == 'SEND' and time.ticks_diff(now,self.deadline)>=0:
            frame = '#%03dP%04dT%04d!' % (self.index,self.values[self.index],self.duration)
            if self.servo.write(frame) != len(frame):
                raise ValueError('Short write')
            self.index += 1
            if self.index == len(self.values):
                self.state = 'POSE_WAIT'
                self.deadline = time.ticks_add(now,self.duration+500)
            else:
                self.deadline = time.ticks_add(now,30)
        elif self.state == 'POSE_WAIT' and time.ticks_diff(now,self.deadline)>=0:
            self.begin_read(self.phase,now)
        elif self.state == 'CLOSE_WAIT' and time.ticks_diff(now,self.deadline)>=0:
            self.begin_pose(pose.POST_GRAB_HOLD_TARGETS,4000,'FINAL',now)
        elif self.state == 'RELEASE_WAIT' and time.ticks_diff(now,self.deadline)>=0:
            self.begin_read('RELEASE',now)
        elif self.state == 'OPEN_WAIT' and time.ticks_diff(now,self.deadline)>=0:
            self.begin_read('OPEN',now)

    def poll(self):
        now = time.ticks_ms()
        try:
            self.buffer += self.link.read() or b''
            if len(self.buffer)>256:
                self.buffer = b''
                raise ValueError('Link overflow')
            while b'\n' in self.buffer:
                raw, self.buffer = self.buffer.split(b'\n',1)
                self.handle(raw.decode().strip(),now)
            if self.sequence is not None:
                self.step(now)
        except Exception:
            self.buffer = b''
            if self.sequence is not None:
                self.stop()
                try:
                    self.finish('ERROR')
                except OSError:
                    # A broken transport cannot deliver the error, but must
                    # leave the action stopped and never restart on reconnect.
                    pass


def serve():
    from machine import UART
    from factory.z_wifi_link import WifiLink
    link = WifiLink()
    service = GrabService(link, UART(2,115201,tx=17,rx=16,timeout=0))
    try:
        while True:
            service.poll()
            time.sleep_ms(5)
    finally:
        if service.sequence is not None:
            service.stop()
        link.close()
