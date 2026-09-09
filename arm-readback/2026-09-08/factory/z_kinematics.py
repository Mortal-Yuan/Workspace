from math import *
import time


class ZL_KINEMATICS(object):
    pi = 3.1415926
    servo_angle = [0,0,0,0]#记录角度
    servo_pwm = [0,0,0,0]#记录pwm

    '''

    设置四个关节的长度
    单位0.1mm
    l0  底盘到第二个舵机中心轴的距离
    l1  第二个舵机到第三 个舵机的距离
    l2  第三个舵机到第四 个舵机的距离
    l3  第四个舵机到机械臂(闭合后)最高点的距离

    '''
    
    def __init__(self, L0=170, L1=105, L2=75, L3=185):
        #放大10倍
        self.l0 = L0*10
        self.l1 = L1*10
        self.l2 = L2*10
        self.l3 = L3*10

    '''
    x,y 为映射到平面的坐标
    z为距离地面的距离
    Alpha 为爪子和平面的夹角 -25~-65范围比较好
    '''
    #x,y,z为爪子闭合时的末端点的坐标，alpha为爪子与水平面的角度
    def kinematics_analysis(self, x, y, z, Alpha):
        #放大10倍
        x = x*10
        y = y*10
        z = z*10


        if(x == 0 and y != 0) :
            theta6 = 0.0
        elif(x > 0 and y == 0):
            theta6 = 90
        elif(x < 0 and y == 0):
            theta6 = -90
        else :
            theta6 = atan(x/y)*180.0/self.pi #计算云台旋转角度

        
        
        y = sqrt(x*x + y*y) #x y 坐标的斜边 
        y = y-self.l3 * cos(Alpha*self.pi/180.0)  #求y1+y2
        z = z-self.l0-self.l3*sin(Alpha*self.pi/180.0) #求z1+z2

        
        
    
        if (z<-self.l0):
            return 1
        
        
        if(sqrt(y*y + z*z) > (self.l1+self.l2)) :
            return 2
        
        
        
        ccc = acos(y / sqrt(y * y + z * z));
        bbb = (y*y+z*z+self.l1*self.l1-self.l2*self.l2)/(2*self.l1*sqrt(y*y+z*z));
        if(bbb > 1 or bbb < -1) :
            return 3;

        if (z < 0) :
            zf_flag = -1
        else :
            zf_flag = 1

        theta5 = ccc * zf_flag + acos(bbb);     #计算1号舵机的弧度
        theta5 = theta5 * 180.0 / self.pi;           #转化为角度 
        if(theta5 > 180.0 or theta5 < 0.0) :
            return 4

        aaa = -(y*y+z*z-self.l1*self.l1-self.l2*self.l2)/(2*self.l1*self.l2);
        if (aaa > 1 or aaa < -1) :
            return 5

        theta4 = acos(aaa); #2号舵机的弧度
        theta4 = 180.0 - theta4 * 180.0 / self.pi ;  #转化为角度
        if (theta4 > 135.0 or theta4 < -135.0) :
            return 6

        
        theta3 = Alpha - theta5 + theta4;   #计算3号舵机角度
        if(theta3 > 90.0 or theta3 < -90.0) :
            return 7


        self.servo_angle[0] = theta6
        self.servo_angle[1] = theta5-90
        self.servo_angle[2] = theta4
        self.servo_angle[3] = theta3    

        self.servo_pwm[0] = (int)(1500-2000.0 * self.servo_angle[0] / 270.0);
        self.servo_pwm[1] = (int)(1500+2000.0 * self.servo_angle[1] / 270.0);
        self.servo_pwm[2] = (int)(1500+2000.0 * self.servo_angle[2] / 270.0);
        self.servo_pwm[3] = (int)(1500+2000.0 * self.servo_angle[3] / 270.0);
        
        

        return 0;

    #此函数可移植到要执行的主程序里面去
    def kinematics_move(self,x,y,z,mytime): 
        alpha_list = []
        if(y < 0):
            return 0;
        #寻找3号舵机的最佳角度
        flag = 0;
        best_alpha = 0
        
        for i in range(-25,-65,-1) :
            if self.kinematics_analysis(x,y,z,i):
                alpha_list.append(i)
        if len(alpha_list) > 0:
            if y > 2150:
                best_alpha = max(alpha_list)
            else:
                best_alpha = min(alpha_list)
            flag = 1
            
        
        #用3号舵机与水平最大的夹角作为最佳值
        if(flag) :
            self.kinematics_analysis(x,y,z,best_alpha);
            testStr = '{'
            for j in range(0,4) :
                #set_servo(j, servo_pwm[j], time);
                #print(servo_pwm[j])
                testStr += "#%03dP%04dT%04d!" % (j,self.servo_pwm[j],mytime)
            testStr += '}'
            #print(testStr)
            return testStr

        return ''


#程序入口
if __name__ == '__main__':
    kms = ZL_KINEMATICS(170,105,75,185)                      # 实例化
    try:                                                     # 异常处理
        while 1:                                             # 无限循环
            print('//----------------------------------------------------------')
            print('start:', time.time())
            print(kms.kinematics_move(250,0,20,1000))
            print('end:',time.time())
            print('----------------------------------------------------------//')
            print(' ')
            print(' ')
            #time.sleep(0.5)
    except:
        pass