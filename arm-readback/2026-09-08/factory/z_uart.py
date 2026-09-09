#导包
from machine import UART
import time

class ZL_UART(object):
    
    def __init__(self, baud=115200):
        self.uart2 = ''                                             # 串口2
        self.baud = baud                                            # 波特率
        self.mode = 0
        self.uart_get_ok = 0
        self.uart_receive_str = ''
        self.uart_send_flag = 0
        
        self.uart2 = UART(2, self.baud)                             # 使用给定波特率初始化
        self.uart2.init(self.baud, bits=8, parity=None, stop=1)     # 使用给定参数初始化
        self.uart2.write('uart2 init ok!\r\n')
            
    #发送字符串 只需传入要发送的字符串即可
    def uart_send_str(self, temp):
        self.uart_send_flag = 1
        self.uart2.write(temp)                                      # 串口发送数据

    # 串口接收数据，主要处理数据接受格式，主要格式为<...> {...} $...!  #...! 4种格式，...内容长度不限
    def recv_str(self):
        if self.uart2.any() > 0:
            self.uart_receive_str = self.uart_receive_str + self.uart2.read().decode("utf-8","ignore")
        
        if self.uart_send_flag:
            self.uart_receive_str = ''
            self.uart_send_flag = 0
            return
    
        if len(self.uart_receive_str) < 2:
            return
        
        self.mode = 0
        if self.mode == 0:
            if self.uart_receive_str.find('<') >= 0:
                self.mode = 1
                #print('mode1 start')
            elif self.uart_receive_str.find('{') >= 0:
                self.mode = 2
                #print('mode2 start')
            elif self.uart_receive_str.find('#') >= 0:
                self.mode = 3
                #print('mode3 start')
            elif self.uart_receive_str.find('$') >= 0:
                self.mode = 4
                #print('mode4 start')
        
        if self.mode == 1:
            if self.uart_receive_str.find('>') >= 0:
                self.uart_get_ok = 1
                self.mode = 0
                #print('mode1 end')
        elif self.mode == 2:
            if self.uart_receive_str.find('}') >= 0:
                self.uart_get_ok = 2
                self.mode = 0
                #print('mode2 end')
        elif self.mode == 3:
            if self.uart_receive_str.find('!') >= 0:
                self.uart_get_ok = 3
                self.mode = 0
                #print('mode3 end')
        elif self.mode == 4:
            if self.uart_receive_str.find('!') >= 0:
                self.uart_get_ok = 4
                self.mode = 0
                #print('mode4 end')
 
#程序入口
if __name__ == '__main__':
    uart = ZL_UART()                                         # 实例化串口
    try:                                                     # 异常处理
        while 1:                                             # 无限循环
            uart.recv_str()                                 # 串口接收并根据格式处理数据
            if uart.uart_get_ok:
                print(int(time.time()*1000))
                if uart.uart_receive_str == '<$LEDON!>':
                    uart.uart_send_str("1111")
                    print('1111')
                elif uart.uart_receive_str == '{LEDON!}':
                    uart.uart_send_str("2222")
                    print('2222')
                elif uart.uart_receive_str == '$LEDON!':
                    uart.uart_send_str("3333")
                    print('3333')
                elif uart.uart_receive_str == '#LEDON!':
                    uart.uart_send_str("4444")
                    print('4444')
                
                uart.uart_receive_str = ''
                uart.uart_get_ok = 0

    except:
        pass
