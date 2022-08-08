#!/usr/bin/python
# -*- coding: UTF-8 -*-
#!/usr/bin/python
# -*- coding: UTF-8 -*-
import serial #导入模块
import time
import timeout_decorator
import time
import math
class motor:
    def __init__(self):
        self.param1=0x01
        self.param2=0x23
        self.param3=0x7A
        self.param4=0x60

    def setpos(self,data0,data1,data2,data3):
        ID=0x01
        stop_bit=0xff+ID-self.param1-self.param2-self.param3-self.param4-data0-data1-data2-data3
        stop_bit=0xff&stop_bit
        return stop_bit
    # @timeout_decorator.timeout(2)
    def motor_position(self,position_abs,count):
        try:
            portx="/dev/motor"
            bps=38400
            #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
            timex=None
            ser=serial.Serial(portx,bps,stopbits=1,timeout=timex)
            # ser=serial.Serial('COM4',38400)#windows下使用
            if position_abs<0:
                data=position_abs
                data=(position_abs )&0xffffffff
                data=hex(data)
                data0=int(data[2:4],16)##高8位
                data1=int(data[4:6],16)
                data2=int(data[6:8],16)
                data3=int(data[8:10],16)
            else:
                data=hex(int(position_abs))
                data=data[2:]
                data='0x'+(data.zfill(8))
                # print('data',data)
                data0=int(data[2:4],16)##高8位
                data1=int(data[4:6],16)
                data2=int(data[6:8],16)
                data3=int(data[8:10],16)   
                
            stop_bit=self.setpos(data0, data1, data2, data3)
            power_on=[0x01, 0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0f]
            position_command=[0x01, 0x23, 0x7A, 0x60,  0x00, data3, data2, data1, data0,stop_bit]
            status=[0x01, 0x23, 0x81, 0x60, 0x00, 0x55, 0x55, 0x08, 0x00, 0x49]
            control_1=[0x01, 0x2B, 0x40, 0x60, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x05]
            control_2=[0x01, 0x2B, 0x40, 0x60, 0x00, 0x3F, 0x00, 0x00, 0x00, 0xf5]
            flag = ser.is_open
            if flag:
                # print('success\n')
                # print("串口详情参数:", ser)
                result=ser.write(bytearray(power_on))#
                # print("写总字节数:",result)
                time.sleep(0.1)
                result=ser.write(bytearray(position_command))#
                time.sleep(0.1)
                result=ser.write(bytearray(status))#
                time.sleep(0.1)
                result=ser.write(bytearray(control_1))#
                time.sleep(0.1)
                result=ser.write(bytearray(control_2))#
                time.sleep(0.1)
                for i in range(count):
                    result=ser.write([0x01, 0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE7])
                    data=ser.read(10).hex()
                    
                    if data[0:10]=='014b786000':
                        print(data)
                        data1=data[10:12]
                        data2=data[12:14]
                        data3=data[14:16]
                        data4=data[16:18]
                       
                        data_end=int(data4+data3+data2+data1,16)
                        
                        if data_end>280 and data_end<100000:
                            # print(data_end)
                            stop_bit_1=self.setpos(0, 0, 0, 0)
                            result=ser.write(bytearray(power_on))#
                            result=ser.write(bytearray([0x01, 0x23, 0x7A, 0x60,  0x00,0, 0, 0, 0,stop_bit_1]))#
                            result=ser.write(bytearray(status))#
                            result=ser.write(bytearray(control_1))#
                            result=ser.write(bytearray(control_2))#
                            time.sleep(2)
                            print("夹持机构error")
                            assert False
                        else:
                            print("夹持机构true")
                    time.sleep(0.2)    
            else:
                print('Open Error\n')
        except Exception as e:
            print(e)
            