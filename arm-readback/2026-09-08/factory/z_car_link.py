from machine import UART
import time


class ZL_CAR_LINK(object):
    """Independent UART1 link to the car controller; never moves servos."""

    def __init__(self, baud=115200, tx=22, rx=21):
        self.uart = UART(1, baudrate=baud, bits=8, parity=None, stop=1,
                         tx=tx, rx=rx, timeout=0)
        self.buffer = b''

    def _handle(self, line):
        fields = line.split(',')
        if len(fields) == 3 and fields[0] == 'CAR' and fields[1] == 'PING':
            sequence = fields[2]
            if len(sequence) == 4 and sequence.isdigit():
                response = 'ARM,PONG,%s\n' % sequence
                self.uart.write(response)
                return response.strip()
        return None

    def poll(self):
        if self.uart.any():
            self.buffer += self.uart.read()
            if len(self.buffer) > 128:
                self.buffer = self.buffer[-64:]
        newline = self.buffer.find(b'\n')
        if newline < 0:
            return None
        raw = self.buffer[:newline]
        self.buffer = self.buffer[newline + 1:]
        try:
            line = raw.decode().strip()
        except Exception:
            return None
        return self._handle(line)

    def serve_for(self, duration_ms=30000):
        deadline = time.ticks_add(time.ticks_ms(), duration_ms)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            response = self.poll()
            if response:
                print(response)
            time.sleep_ms(5)
