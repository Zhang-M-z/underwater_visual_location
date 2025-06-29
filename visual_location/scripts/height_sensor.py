#! /usr/bin/env python3
# coding:UTF-8
import serial
import time

class HeightSensor:
    def __init__(self, port="COM15", baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=1):
        """
        初始化高度传感器
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
        self.ser = None

    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout
            )
            if self.ser.is_open:
                print(f"成功打开串口 {self.ser.name}")
                return True
            return False
        except Exception as e:
            print(f"串口连接异常: {str(e)}")
            return False

    def read_distance(self):
        """
        读取距离数据
        :return: 距离值，如果读取失败返回None
        """
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return None

        try:
            # 发送数据
            send_data = "$1,21,1,LAKEAS*02FA"
            self.ser.write(send_data.encode())
            # print(f"已发送: {send_data}")

            # 接收数据
            time.sleep(0.5)
            recv_data = self.ser.read_all()
            if recv_data:
                response = recv_data.decode()
                f_find = response.find('f')
                M_find = response.find('M')
                if f_find != -1 and M_find != -1:
                    output = response[f_find+2:M_find-1]
                    distance = float(output)
                    print(f'The distance is {distance:e} m')
                    return distance
            return None
        except Exception as e:
            print(f"读取数据异常: {str(e)}")
            return None

    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")

# 使用示例
if __name__ == "__main__":
    sensor = HeightSensor()
    if sensor.connect():
        try:
            while True:
                distance = sensor.read_distance()
                if distance is not None:
                    print(f"测量距离: {distance:.3f} 米")
        except KeyboardInterrupt:
            sensor.close()