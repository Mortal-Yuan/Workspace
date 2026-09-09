from machine import Pin,ADC
import time

class ZL_ADC(object):
    def __init__(self, SA1=36, SA2=39, SA3=4, SA4=12, SA5=35):
        self.SA1 = ADC(Pin(SA1))
        self.SA2 = ADC(Pin(SA2))
        self.SA3 = ADC(Pin(SA3))
        self.SA4 = ADC(Pin(SA4))
        self.SA5 = ADC(Pin(SA5))
        
        # 初始化
        self.SA1.atten(ADC.ATTN_11DB)  # 11dB衰减, 最大输入电压约3.6v
        self.SA2.atten(ADC.ATTN_11DB)  # 11dB衰减, 最大输入电压约3.6v
        self.SA3.atten(ADC.ATTN_11DB)  # 11dB衰减, 最大输入电压约3.6v
        self.SA4.atten(ADC.ATTN_11DB)  # 11dB衰减, 最大输入电压约3.6v
        self.SA5.atten(ADC.ATTN_11DB)  # 11dB衰减, 最大输入电压约3.6v
   
    # 读取模拟值
    def adc_value(self, adc_pin):
        if adc_pin == 'SA1':
            return round(self.SA1.read())
        elif adc_pin == 'SA2':
            return round(self.SA2.read())
        elif adc_pin == 'SA3':
            return round(self.SA3.read())
        elif adc_pin == 'SA4':
            return round(self.SA4.read())
        elif adc_pin == 'SA5':
            return round(self.SA5.read())

# 程序入口
if __name__ == '__main__':
    adc = ZL_ADC()        # 实例化adc对象
    try:
        while 1:           
            print(str(adc.adc_value('SA4')))
            time.sleep(0.3)
    except:
        pass
