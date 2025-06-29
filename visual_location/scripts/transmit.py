#! /usr/bin/env python3
# coding:UTF-8
import serial
import time
import numpy as np

class send_signal:
    def __init__(self, port="COM12", baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1):
        """
        初始化串口通信
        :param port: 串口号
        :param baudrate: 波特率
        :param bytesize: 数据位
        :param parity: 校验位
        :param stopbits: 停止位
        :param timeout: 超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.esp32_ser = None

    def connect(self):
        """连接串口"""
        try:
            self.esp32_ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout
            )
            if self.esp32_ser.is_open:
                print(f"成功打开ESP32串口 {self.esp32_ser.name}")
                return True
            return False
        except Exception as e:
            print(f"ESP32串口连接异常: {str(e)}")
            return False

    def Control_send(self, th1, th2, th3, th4, th5, th6):
        """
        发送控制信号
        :param th1-th6: 六个控制参数
        :return: 是否发送成功
        """
        if not self.esp32_ser or not self.esp32_ser.is_open:
            print("ESP32串口未连接")
            return False

        try:
            send_float = [float(th1), float(th2), float(th3), float(th4), float(th5), float(th6)]
            send_string = ','.join([f'{num:06.2f}' for num in send_float])
            send_byte = str.encode(send_string)
            
            self.esp32_ser.write(send_byte)
            self.esp32_ser.flushInput()
            time.sleep(0.1)
            print(f"已发送: {send_string}")
            return True
        except Exception as e:
            print(f"发送数据异常: {str(e)}")
            self.close()
            return False

    def close(self):
        """关闭串口连接"""
        if self.esp32_ser and self.esp32_ser.is_open:
            self.esp32_ser.close()
            print("ESP32串口已关闭")

    #测试是否可以通过python运行程序自控
if __name__ == "__main__":
    import random
    esp32 = send_signal("COM12", 115200)
    
    if esp32.connect():
        try:
            while True:
                if not esp32.esp32_ser or not esp32.esp32_ser.is_open:
                    if not esp32.connect():
                        time.sleep(1)
                        continue
                
                try:
                    th1 = 80 
                    th2 = 80 
                    th3 = 50 
                    th4 = 50 
                    th5 = 50 
                    th6 = 50 
                    
                    if esp32.Control_send(th1, th2, th3, th4, th5, th6):
                        data = esp32.esp32_ser.readline().decode('utf-8').strip()
                        print(f"接收数据: {data}")
                except Exception as e:
                    print(f"数据发送异常: {str(e)}")
                    esp32.close()
                time.sleep(1)
        except KeyboardInterrupt:
            esp32.close()
            print("程序已停止")