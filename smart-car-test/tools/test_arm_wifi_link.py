import importlib.util
import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

time.ticks_ms = lambda: 1000
time.ticks_add = lambda a,b:a+b
time.ticks_diff = lambda a,b:a-b
spec = importlib.util.spec_from_file_location('wifi_under_test',
    Path(__file__).resolve().parents[2]/'arm-readback/2026-09-08/factory/z_wifi_link.py')
module = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'network':types.SimpleNamespace()}):
    spec.loader.exec_module(module)

class Socket:
    def __init__(self, chunks): self.chunks=list(chunks);self.sent=[];self.closed=False
    def recv(self,n):
        if not self.chunks: raise OSError(11)
        return self.chunks.pop(0)
    def send(self,data): self.sent.append(data);return len(data)
    def close(self): self.closed=True

def link(chunks):
    obj=module.WifiLink.__new__(module.WifiLink)
    obj.sock=Socket(chunks);obj.wlan=types.SimpleNamespace(isconnected=lambda:True)
    obj.buffer=b'';obj.last_rx=1000;obj.retry_at=0
    return obj

class WifiTests(unittest.TestCase):
    def test_ap_unavailable_retries_without_killing_service(self):
        obj=link([]);obj.sock=None;obj.wifi_retry_at=0;calls=[]
        def unavailable(*args):calls.append(1);raise OSError('AP absent')
        obj.wlan=types.SimpleNamespace(isconnected=lambda:False,
            disconnect=lambda:None,connect=unavailable)
        self.assertIsNone(obj.read());self.assertEqual(len(calls),1)
        self.assertIsNone(obj.read());self.assertEqual(len(calls),1)
        with patch.object(time,'ticks_ms',lambda:11000):obj.read()
        self.assertEqual(len(calls),2)
    def test_fragmented_and_combined_frames(self):
        obj=link([b'CAR,HE',b'ART,0000\nCAR,PING,0001\nCAR,GRAB,0002\n'])
        self.assertIsNone(obj.read())
        self.assertEqual(obj.read(),b'CAR,PING,0001\nCAR,GRAB,0002\n')
        self.assertEqual(obj.sock.sent,[b'ARM,HEART,0000\n'])
        self.assertIsNone(obj.read())
    def test_eof_discards_partial_command(self):
        obj=link([b'CAR,GRAB,',b'']);sock=obj.sock
        self.assertIsNone(obj.read())
        with self.assertRaises(OSError):obj.read()
        self.assertTrue(sock.closed);self.assertEqual(obj.buffer,b'')
    def test_heartbeat_timeout_even_with_socket_open(self):
        obj=link([]);obj.last_rx=-1001
        with self.assertRaises(OSError):obj.read()
        self.assertIsNone(obj.sock)
    def test_partial_send_closes_connection(self):
        obj=link([]);obj.sock.send=lambda data:3
        with self.assertRaises(OSError):obj.write('ARM,PONG,0001\n')
        self.assertIsNone(obj.sock)

if __name__=='__main__':unittest.main()
