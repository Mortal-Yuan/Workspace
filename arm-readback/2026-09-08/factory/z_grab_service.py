"""Explicit-command grab service. Construction and PING never move servos.

Dedicated UART1: car TX GPIO1 -> arm RX21; arm TX22 -> car RX GPIO2.
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
        self.emit(result, self.sequence)
        self.sequence = None
        self.state = 'IDLE'

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
        elif command == 'GRAB':
            if sequence == self.sequence:
                self.emit('ACK', sequence)
            elif sequence == self.last_sequence:
                self.emit(self.last_result, sequence)
            elif self.sequence is not None:
                self.emit('BUSY', sequence)
            else:
                self.sequence = sequence
                self.emit('ACK', sequence)
                self.begin_read('START', now)

    def begin_read(self, phase, now):
        self.phase, self.index, self.positions = phase, 0, []
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
                self.positions.append(int(value))
                self.index += 1
                count = 5 if self.phase == 'FINAL' else 6
                if self.index < count:
                    self.query(now)
                    return
                expected = (pose.POST_GRAB_HOLD_POSE if self.phase in ('START','FINAL')
                            else pose.PICKUP_OPEN_POSE)
                if any(abs(a-b)>40 for a,b in zip(self.positions,expected)):
                    raise ValueError('Position not reached')
                if self.phase == 'START':
                    self.begin_pose(pose.PICKUP_OPEN_POSE,3000,'LOW',now)
                elif self.phase == 'LOW':
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
            if self.sequence is not None:
                self.stop()
                self.finish('ERROR')


def serve():
    from machine import UART
    service = GrabService(UART(1,115200,tx=22,rx=21,timeout=0),
                          UART(2,115201,tx=17,rx=16,timeout=0))
    try:
        while True:
            service.poll()
            time.sleep_ms(5)
    finally:
        if service.sequence is not None:
            service.stop()
