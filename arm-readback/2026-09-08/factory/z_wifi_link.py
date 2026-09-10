"""Wi-Fi client transport for the car's dedicated WPA2 access point."""
import time
import socket
import network


class WifiLink:
    def __init__(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        self.sock = None
        self.buffer = b''
        self.retry_at = time.ticks_ms()
        self.last_rx = self.retry_at
        self.wifi_retry_at = self.retry_at

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.buffer = b''
        self.retry_at = time.ticks_add(time.ticks_ms(), 1000)

    def fail(self):
        self.close()
        raise OSError('Wi-Fi link lost')

    def write(self, frame):
        if self.sock is None:
            raise OSError('Wi-Fi disconnected')
        data = frame.encode() if isinstance(frame, str) else frame
        try:
            count = self.sock.send(data)
        except OSError:
            self.fail()
        if count != len(data):
            self.fail()
        return count

    def read(self):
        now = time.ticks_ms()
        if not self.wlan.isconnected():
            if self.sock is not None:
                self.fail()
            if time.ticks_diff(now, self.wifi_retry_at) >= 0:
                self.wifi_retry_at = time.ticks_add(now, 10000)
                try:
                    self.wlan.disconnect()
                    self.wlan.connect('CarArm-8F9C', 'ArmLink-2026-8F9C')
                except OSError:
                    # AP may boot later than the arm; keep the service alive.
                    pass
            return None
        if self.sock is None:
            if time.ticks_diff(now, self.retry_at) < 0:
                return None
            sock = socket.socket()
            try:
                # Only connect while disconnected, after GrabService has aborted.
                sock.settimeout(0.25)
                sock.connect(('192.168.4.1', 8266))
                sock.setblocking(False)
            except OSError:
                sock.close()
                self.retry_at = time.ticks_add(now, 1000)
                return None
            self.sock = sock
            self.last_rx = now
            print('ARM_WIFI_CONNECTED')
        if not self.wlan.isconnected() or time.ticks_diff(now, self.last_rx) > 2000:
            self.fail()
        try:
            data = self.sock.recv(256)
        except OSError as exc:
            if exc.args and exc.args[0] in (11, 35, 10035):
                return None
            self.fail()
        if data == b'':
            self.fail()
        self.buffer += data
        if len(self.buffer) > 512:
            self.fail()
        result = b''
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            if line == b'CAR,HEART,0000':
                self.last_rx = now
                self.write(b'ARM,HEART,0000\n')
            else:
                result += line + b'\n'
        return result or None
