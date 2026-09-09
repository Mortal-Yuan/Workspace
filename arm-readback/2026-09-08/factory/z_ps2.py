import time
from machine import Pin
import _thread
import re

class ZL_PS2(object):
    # These are our button constants
    SELECT = 1
    AL = 2
    AR = 3
    START = 4
    LU = 5
    LR = 6
    LD = 7
    LL = 8
    L2 = 9
    R2 = 10
    L1 = 11
    R1 = 12
    RU = 13
    RR = 14
    RD = 15
    RL = 16
    KEYS = dict([
                 (LU, "LU"), (LD, "LD"), (LL, "LL"), (LR, "LR"),
                 (RU, "RU"), (RD, "RD"), (RL, "RL"), (RR, "RR"),
                 (L1, "L1"), (L2, "L2"), (R1, "R1"), (R2, "R2"),
                 (SELECT, "SELECT"), (START, "START"), (AL, "AL"), (AR, "AR"), 
                ])

    pre_cmd_set_red_remote_mode = [ #红灯模式下遥控模式下的按键的配置 
        "<PS2_RED01:#005P0600T2000!^$DST!>",    #L2
        "<PS2_RED02:#005P2400T2000!^$DST!>",    #R2
        "<PS2_RED03:#004P2400T2000!^$DST!>",    #L1
        "<PS2_RED04:#004P0600T2000!^$DST!>",    #R1
        "<PS2_RED05:#002P2400T2000!^$DST!>",    #RU
        "<PS2_RED06:#003P2400T2000!^$DST!>",    #RR
        "<PS2_RED07:#002P0600T2000!^$DST!>",    #RD
        "<PS2_RED08:#003P0600T2000!^$DST!>",    #RL
        "<PS2_RED09:$AIMODE!>",                 #SE
        "<PS2_RED10:$!>",                       #AL
        "<PS2_RED11:$!>",                       #AR
        "<PS2_RED12:$DJR!>",                    #ST
        "<PS2_RED13:#001P0600T2000!^$DST!>",    #LU
        "<PS2_RED14:#000P0600T2000!^$DST!>",    #LR
        "<PS2_RED15:#001P2400T2000!^$DST!>",    #LD
        "<PS2_RED16:#000P2400T2000!^$DST!>"     #LL
    ]
    
    pre_cmd_set_red_ai_mode = [ #红灯模式下智能模式下的按键的配置 
        "<PS2_RED01:$!>",                       #L2
        "<PS2_RED02:$!>",                       #R2
        "<PS2_RED03:$!>",                       #L1
        "<PS2_RED04:$!>",                       #R1
        "<PS2_RED05:$DGT:18-21,1!>",            #RU
        "<PS2_RED06:$DGT:27-31,1!>",            #RR
        "<PS2_RED07:$DGT:2-2,1!>",              #RD
        "<PS2_RED08:$DGT:22-26,1!>",            #RL
        "<PS2_RED09:$AIMODE!>",                 #SE
        "<PS2_RED10:$!>",                       #AL
        "<PS2_RED11:$!>",                       #AR
        "<PS2_RED12:$DGT:1-1,1!>",              #ST
        "<PS2_RED13:$DGT:3-7,1!>",              #LU
        "<PS2_RED14:$DGT:13-17,1!>",            #LR
        "<PS2_RED15:$DGT:1-1,1!>",              #LD
        "<PS2_RED16:$DGT:8-12,1!>"              #LL
    ]

    pre_cmd_set_grn = [#绿灯模式下按键的配置
        "<PS2_GRN01:#005P0600T2000!^$DST!>",    #L2
        "<PS2_GRN02:#005P2400T2000!^$DST!>",    #R2 
        "<PS2_GRN03:#004P0600T2000!^$DST!>",    #L1
        "<PS2_GRN04:#004P2400T2000!^$DST!>",    #R1
        "<PS2_GRN05:#002P2400T2000!^$DST!>",    #RU
        "<PS2_GRN06:#003P2400T2000!^$DST!>",    #RR
        "<PS2_GRN07:#002P0600T2000!^$DST!>",    #RD 
        "<PS2_GRN08:#003P0600T2000!^$DST!>",    #RL
        "<PS2_GRN09:$DGT:1-3,3!>",              #SE
        "<PS2_GRN10:$!>",                       #AL
        "<PS2_GRN11:$!>",                       #AR
        "<PS2_GRN12:$DJR!>",                    #ST
        "<PS2_GRN13:#001P0600T2000!^$DST!>",    #LU
        "<PS2_GRN14:#000P0600T2000!^$DST!>",    #LR
        "<PS2_GRN15:#001P2400T2000!^$DST!>",    #LD 
        "<PS2_GRN16:#000P2400T2000!^$DST!>"     #LL
    ]
   
    PS2_LED_RED = 0x73
    PS2_LED_GRN = 0x41
    ps2_buf = [0,0,0,0,0,0,0,0,0]
    flag_thread = 0
    ps2_ready_flag1 = 1
    ps2_ready_flag2 = 1
    ps2_ai_mode = 0

    def __init__(self, PIN_DAT=19, PIN_CMD=18, PIN_ATT=15, PIN_CLK=23, flag_thread=1):  # DI=DAT、DO=CMD 可以在此处将针脚调整为你自己对应的针脚
        
        self.PIN_DAT = PIN_DAT
        self.PIN_CMD = PIN_CMD
        self.PIN_ATT = PIN_ATT
        self.PIN_CLK = PIN_CLK
        self.flag_thread = flag_thread
        
        self.dat = Pin(self.PIN_DAT, Pin.IN) 
        self.cmd = Pin(self.PIN_CMD, Pin.OUT) 
        self.att = Pin(self.PIN_ATT, Pin.OUT)
        self.clk = Pin(self.PIN_CLK, Pin.OUT)
        
        self.cmd.value(1)
        self.clk.value(1)
        self.att.value(1)
        
        if self.flag_thread:
            _thread.start_new_thread(self.loop_ps2, ())
    

    def ps2_transfer(self, data):
        write_data = data
        read_data = 0
        
        for i in range(8):
            self.cmd.value((write_data & (0x01 << i)))
            #time.sleep_us(16)
            
            self.clk.value(1)
            time.sleep_us(5)
            
            self.clk.value(0)
            time.sleep_us(5)
            
            if self.dat.value() == 1:
                read_data |= 0x01 << i
         
        self.clk.value(1)
        time.sleep_us(10)
            
        return read_data
    
    def ps2_write_read(self):
        self.att.value(0)
        self.ps2_buf[0] = self.ps2_transfer(0x01)
        self.ps2_buf[1] = self.ps2_transfer(0x42)
        self.ps2_buf[2] = self.ps2_transfer(self.ps2_buf[0])
        self.ps2_buf[3] = self.ps2_transfer(self.ps2_buf[0])
        self.ps2_buf[4] = self.ps2_transfer(self.ps2_buf[0])
        self.ps2_buf[5] = self.ps2_transfer(self.ps2_buf[0])
        self.ps2_buf[6] = self.ps2_transfer(self.ps2_buf[0])
        self.ps2_buf[7] = self.ps2_transfer(self.ps2_buf[0])
        self.ps2_buf[8] = self.ps2_transfer(self.ps2_buf[0])     
        self.att.value(1)
        return self.ps2_buf
    
    def get_buf(self):
        return self.ps2_buf
    
    buf_bak=0xffff
    last_mode = 0
    def get_str(self):
        return_str = ''
        
        temp = (self.ps2_buf[3]<<8) + self.ps2_buf[4]
        temp2 = 0xffff
        
        if self.last_mode != self.ps2_buf[1]:
            self.last_mode = self.ps2_buf[1]
            self.buf_bak = 0xffff
            time.sleep(0.1)
            return return_str

        if self.buf_bak != temp:
            temp2 = temp
            temp &= self.buf_bak
            
            for i in range(16):#16个按键一次轮询
                if((1<<i) & temp) :
                    pass
                else :
                    if self.ps2_buf[1] == self.PS2_LED_RED:
                        if self.ps2_ai_mode == 0:
                            return_str = self.pre_cmd_set_red_remote_mode[i]
                        elif self.ps2_ai_mode == 1:
                            return_str = self.pre_cmd_set_red_ai_mode[i]
                    elif self.ps2_buf[1] == self.PS2_LED_GRN:
                        return_str = self.pre_cmd_set_grn[i]
                    else:
                        continue
                    
                    if((1<<i) & self.buf_bak): #press 表示按键按下了
                        if return_str.find('^') > 0:
                            return_str = return_str.split(':')
                            return_str = return_str[1]
                            return_str = return_str.replace('^','\0')
                        else:
                            if return_str.find('$') > 0:
                                return_str = return_str.split('$')
                                return_str = return_str[1]
                                return_str = return_str.split('!')
                                return_str = return_str[0]
                                return_str = '$' +return_str+ '!'
                                print(return_str)
                            elif return_str.find('#') > 0:
                                return_str = return_str.split(':')
                                return_str = return_str[1]
                                return_str = return_str.replace('>','\0')
                                print(return_str)
                        # 按了select键后切换模式（在遥控模式和智能模式之间切换）
                        if i == 8:
                            if self.ps2_ai_mode == 0:
                                self.ps2_ai_mode = 1
                            elif self.ps2_ai_mode == 1:
                                self.ps2_ai_mode = 0
                                
                        self.bak = 0xffff;
                    else:#release 表示按键松开了                                         
                        if return_str.find('^') > 0:
                            return_str = return_str.split('^')
                            return_str = return_str[1]
                            return_str = return_str.replace('>','\0')
                        else:
                            return_str = ''

            self.buf_bak = temp2

        return return_str
    
    def get_mode(self):
        return self.ps2_buf[1]
    
    def get_joysticks(self):
        return self.ps2_buf[7],self.ps2_buf[8],self.ps2_buf[5],self.ps2_buf[6]
    
    def data_is_ready(self): 
        #2个摇杆向上推
        if self.ps2_buf[0]==0xff and self.ps2_buf[1]==self.PS2_LED_RED and self.ps2_buf[2]==0x5a:
            lx,ly,rx,ry = self.get_joysticks()
            if abs(128-lx)<5 and abs(0-ly)<5 and abs(128-rx)<5 and abs(0-ry)<5:
                self.ps2_ready_flag1 = 1
            if self.ps2_ready_flag1 == 1 and abs(128-lx)<5 and abs(128-ly)<5 and abs(128-rx)<5 and abs(128-ry)<5:
                self.ps2_ready_flag2 = 1
        return self.ps2_ready_flag2
    
    def clear_data(self):
        self.ps2_buf = 0
    #获取系统时间，毫秒为单位
    #def millis(self):
        #return int(time.time_ns()//1000000)
    
    def loop_ps2(self):
        while 1:
            self.ps2_write_read()
            time.sleep(0.02)
            

# 程序入口
if __name__ == '__main__':
    ps2 = ZL_PS2(PIN_DAT=19, PIN_CMD=18, PIN_ATT=15, PIN_CLK=23, flag_thread=1)  # 实例化一个ps2对象,默认线程
    try:
        while 1:                #无限循环
            if(ps2.flag_thread):#线程模式
                
                #获取时间戳
                print(time.time())
                
                '''
                
                #获取数据是否正常 
                if(ps2.data_is_ready()):
                    print("data ok")
                    #获取buf数据
                    key = ps2.get_buf()         
                    print("线程数据：",hex(key[0]),hex(key[1]),hex(key[2]),hex(key[3]),hex(key[4]),hex(key[5]),hex(key[6]),hex(key[7]),hex(key[8]))
                    
                    #获取摇杆数据 用于控制车
                    print(ps2.get_joysticks())
                    
                    #获取按键数据 用于按键配置
                    print(ps2.get_str())
                else:
                    print("none")


                #获取配置数据
                mystr = ps2.get_str()
                if(len(mystr)):
                    print(mystr)
                    
                '''
                if(ps2.data_is_ready()):
                    joysticks_left_x,joysticks_left_y,joysticks_right_x,joysticks_right_y = ps2.get_joysticks()
                    print(joysticks_left_x,joysticks_left_y,joysticks_right_x,joysticks_right_y)

            else:#非线程模式
                key = ps2.ps2_write_read()  #非线程模式
                print("非线程数据：",hex(key[0]),hex(key[1]),hex(key[2]),hex(key[3]),hex(key[4]),hex(key[5]),hex(key[6]),hex(key[7]),hex(key[8]))
                
                mystr = ps2.get_str()
                if(len(mystr)):
                    print(mystr)
            
            time.sleep(0.2)           
            
    except:                     # 异常处理
        pass        
