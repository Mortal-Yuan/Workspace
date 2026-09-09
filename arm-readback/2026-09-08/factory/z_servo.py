import time
import _thread
import math
import machine

from machine import Timer
from machine import PWM

# originally by Radomir Dopieralski http://sheep.art.pl
# from https://bitbucket.org/thesheep/micropython-servo

class ZL_SERVO(object):
    """
    A simple class for controlling hobby servos.
    Args:
        pin (machine.Pin): The pin where servo is connected. Must support PWM.
        freq (int): The frequency of the signal, in hertz.
        min_us (int): The minimum signal length supported by the servo.
        max_us (int): The maximum signal length supported by the servo.
        angle (int): The angle between the minimum and maximum positions.
    """
    servo_num = 6
    
    servo_dict = {
        'pin': [32,33,25,26,27,14],
        'cur': [1500,1500,1500,1500,1500,1500],
        'aim': [1500,1500,1500,1500,1500,1500],
        'time':[1500,1500,1500,1500,1500,1500],
        'bias':[0,0,0,0,0,0],
        'inc': [0,0,0,0,0,0]
    }

    servo_pin_control=[0,0,0,0,0,0]
    
    tim0 = Timer(0)
    
    # 构造方法    
    def __init__(self,b0=0,b1=0,b2=0,b3=0,b4=0,b5=0):
        self.servo_dict['bias'][0] = b0
        self.servo_dict['bias'][1] = b1
        self.servo_dict['bias'][2] = b2
        self.servo_dict['bias'][3] = b3
        self.servo_dict['bias'][4] = b4
        self.servo_dict['bias'][5] = b5
        
        #self.pwm = PWM(self.servo_pin, freq=self.freq, duty=0)
        self.servo_timer_open()

        for i in range(self.servo_num):
            self.servo_pin_control[i] = PWM(machine.Pin(self.servo_dict['pin'][i]))
            self.servo_pin_control[i].freq(50)    # 频率
            self.servo_pin_control[i].duty(77)    # 占空比0-1023
            self.servo_set(i,self.servo_dict['cur'][0]+self.servo_dict['bias'][0],20)

    def servo_timer_open(self):
        self.tim0.init(period=20, mode=Timer.PERIODIC, callback = lambda t:self.servo_timer()) # periodic with 20ms period

    def servo_timer_close(self):
        self.tim0.deinit()

    
    # 舵机增量处理
    def servo_inc_handle(self, index):
        if int(index) >= int(self.servo_num):
            return
        if self.servo_dict['pin'][index] and self.servo_dict['inc'][index]:
            aim_tmp = self.servo_dict['aim'][index]

            if aim_tmp > 2500:
                aim_tmp = 2500
            elif aim_tmp < 500:
                aim_tmp = 500

            if abs(aim_tmp-self.servo_dict['cur'][index]) <= abs(self.servo_dict['inc'][index]):
                self.servo_dict['cur'][index] = aim_tmp
                self.servo_dict['inc'][index] = 0
            else:
                self.servo_dict['cur'][index] = self.servo_dict['cur'][index] + self.servo_dict['inc'][index]
                #print(self.servo_dict['cur'][0])
            
    # 定时器回调函数
    def servo_timer(self):
        for index in range(self.servo_num):
            
            if(self.servo_pin_control[index]):
                #print(self.millis_us())
                # 舵机偏差调节
                temp = self.servo_dict['cur'][index] + self.servo_dict['bias'][index]
                if temp > 2500:
                    temp = 2500
                elif temp < 500:
                    temp = 500
                    
                self.servo_pin_control[index].duty(int(temp*1024//20000))
            # 舵机增量处理
            self.servo_inc_handle(index)

    # 某个舵机经过mytime时间运行到myaim位置
    def servo_set(self, myindex, myaim, mytime):
        
        if int(myindex) >= int(self.servo_num) and myindex != 255:
            return
        
        if myindex == 255:
            myindex = 5
            
        if self.servo_pin_control[int(myindex)]:
            
            if mytime < 20:
                mytime = 20
            elif mytime > 9999:
                mytime = 9999
            
            self.servo_dict['aim'][myindex] = myaim
            self.servo_dict['time'][myindex] = mytime
            self.servo_dict['inc'][myindex] = float((myaim-self.servo_dict['cur'][myindex])/(mytime/20.000))
 
    # 舵机停止
    def servo_stop(self, myindex):
        if int(myindex) >= int(self.servo_num) and myindex != 255:
            return
            
        self.servo_dict['inc'][myindex] = 0
        self.servo_dict['aim'][myindex] = self.servo_dict['cur'][myindex]        

'''
def millis_us():
    print(int(time.time_ns()//1000))

tim0 = Timer(0)
tim0.init(period=1000, mode=Timer.PERIODIC, callback=lambda t:millis_us()) # periodic with 20ms period
'''
 
# 程序入口
if __name__ == '__main__':
    servo = ZL_SERVO()
    #servo.servo_timer_close()
    while 1:                    # 无限循环
        #for index in range(6):
        servo.servo_set(0,2500,1000)
            #temp = 1000
            #servo.servo_pin_control[index].duty(temp*1024//20000)
        time.sleep(1)
        #for index in range(6):  
        servo.servo_set(0,500,1000)
            #temp = 2000
            #servo.servo_pin_control[index].duty(temp*1024//20000)
        time.sleep(1)
        print(time.time())
        
        #print(servo.servo_dict['cur'][0])
        #print(int((float(servo.servo_dict['cur'][0])/200000.000)*1024))
        #pass
