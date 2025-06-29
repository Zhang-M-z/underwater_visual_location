#! /usr/bin/env python3
# coding:UTF-8
import serial
import time

class LaserRangeFinder:
    FRAME_HEAD = bytes([0x55, 0x7A])  # 读取数据的响应帧头

    def __init__(self, port: str, baudrate: int = 9600, dev_id: int = 0x01, timeout: float = 1.0):
        self.dev_id = dev_id
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(self.port, self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=self.timeout)

    def start_measurement(self):
        """发送水下连续测量命令"""
        cmd = self._build_cmd(0x11)  # 水下连续测量命令
        self.ser.write(cmd)

    def _build_cmd(self, cmd_code: int) -> bytes:
        # 构造6字节命令：[0xAA,0x75, cmd_code, 0x01, dev_id, BCC]
        frame = bytearray([0xAA, 0x75, cmd_code, 0x01, self.dev_id])
        bcc = 0
        for b in frame:
            bcc ^= b
        frame.append(bcc)
        return bytes(frame)

    def _parse_response(self, data: bytes):
        # 判断帧格式：[0x55, 0x7A, 0xAA, 0x03, ID, LO, HI, BCC]
        if len(data) != 8 or data[0:2] != self.FRAME_HEAD:
            return None
        # 校验 BCC
        bcc = 0
        for b in data[:7]:
            bcc ^= b
        if bcc != data[7] or data[2] != 0xAA:
            return None
        # 解析距离
        distance_mm = data[6] << 8 | data[5]
        return distance_mm / 1000.0  # 单位：米
    
    def connect(self) -> bool:
        """
        建立与激光测距仪的连接
        :return: 连接是否成功
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=self.timeout)
            if not self.ser.is_open:
                self.ser.open()
            print("连接成功")
            return True
            # 测试连接：查询设备ID
        except Exception as e:
            print(f"连接时发生错误：{str(e)}")
            return False
        
    def run(self):
        """持续读取串口数据并解析距离"""
        print("启动连续测量中...")
        self.start_measurement()
        print("开始读取距离（Ctrl+C 停止）：")

        while True:
            try:
                frame = self.ser.read(8)
                if len(frame) == 8:
                    distance = self._parse_response(frame)
                    if distance is not None:
                        print(f"[{time.strftime('%H:%M:%S')}] 距离：{distance:.3f} m")
                # time.sleep(0.5)
            except KeyboardInterrupt:
                print("\n已手动停止程序")
                break
            except Exception as e:
                print("错误:", e)
    def read(self):
        """返回一次结果"""

if __name__ == "__main__":
    # 修改为实际串口号
    lrf = LaserRangeFinder(port="COM19", baudrate=9600)
    lrf.connect()
    lrf.run()
