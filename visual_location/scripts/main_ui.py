#! /usr/bin/env python3
# coding:UTF-8

#pragma region 主函数
import sys
import os
import time
import threading
import serial.tools.list_ports
from PySide6 import QtWidgets, QtCore, QtGui
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QTimer, Qt, Slot, Signal, QObject
from PySide6.QtWidgets import QFileDialog, QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QPushButton, QSlider, QTextEdit, QMainWindow, QRadioButton, QDoubleSpinBox, QCheckBox, QProgressBar
import numpy as np
import datetime
# 导入传感器模块

try:
    from laser_sensor import LaserRangeFinder
    LASER_AVAILABLE = True
except ImportError:
    print("警告: 激光测距模块导入失败")
    LASER_AVAILABLE = False
    
try:
    from height_sensor import HeightSensor
    HEIGHT_AVAILABLE = True
except ImportError:
    print("警告: 高度传感器模块导入失败")
    HEIGHT_AVAILABLE = False

try:
    from transmit import send_signal
    TRANSMIT_AVAILABLE = True
except ImportError:
    print("警告: 信号发送模块导入失败")
    TRANSMIT_AVAILABLE = False
#pragma endregion

# 传感器数据更新信号类
class SensorSignals(QObject):
    laser_data_updated = Signal(float)
    height_data_updated = Signal(float)
    orientation_data_updated = Signal(dict)
    position_data_updated = Signal(dict)
    error_occurred = Signal(str, str)  # 错误类型，错误消息
    connection_status_changed = Signal(str, bool)  # 传感器类型，连接状态
    log_message = Signal(str)  # 日志消息


# 传感器读取类
class SensorReader:
    def __init__(self, signals):
        self.signals = signals
        self.running = False
        
        # 初始化传感器为None
        self.laser = None
        self.height_sensor = None
        self.esp32 = None
        
        # 传感器状态
        self.laser_connected = False
        self.height_connected = False
        self.esp32_connected = False
    
    def init_laser(self, port, baudrate=9600, dev_id=0x01):
        """初始化激光传感器"""
        if not LASER_AVAILABLE:
            self.signals.error_occurred.emit("激光传感器", "模块导入失败")
            self.signals.log_message.emit("激光传感器模块导入失败")
            return False
        
        try:
            if self.laser:
                self.laser.stop()
                
            self.laser = LaserRangeFinder(port=port, baudrate=baudrate, dev_id=dev_id)
            self.signals.log_message.emit(f"激光传感器初始化成功: {port}, {baudrate}bps")
            return True
        except Exception as e:
            self.signals.error_occurred.emit("激光传感器", f"初始化失败: {str(e)}")
            self.signals.log_message.emit(f"激光传感器初始化失败: {str(e)}")
            self.laser = None
            return False
            
    def init_height_sensor(self, port, baudrate=9600):
        """初始化高度传感器"""
        if not HEIGHT_AVAILABLE:
            self.signals.error_occurred.emit("高度传感器", "模块导入失败")
            self.signals.log_message.emit("高度传感器模块导入失败")
            return False
        
        try:
            if self.height_sensor:
                self.height_sensor.close()
                
            self.height_sensor = HeightSensor(port=port, baudrate=baudrate)
            self.signals.log_message.emit(f"高度传感器初始化成功: {port}, {baudrate}bps")
            return True
        except Exception as e:
            self.signals.error_occurred.emit("高度传感器", f"初始化失败: {str(e)}")
            self.signals.log_message.emit(f"高度传感器初始化失败: {str(e)}")
            self.height_sensor = None
            return False
            
    def init_esp32(self, port, baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1):
        """初始化ESP32通信"""
        if not TRANSMIT_AVAILABLE:
            self.signals.error_occurred.emit("ESP32通信", "模块导入失败")
            self.signals.log_message.emit("ESP32通信模块导入失败")
            return False
        
        try:
            if self.esp32:
                self.esp32.close()
                
            self.esp32 = send_signal(port=port, baudrate=baudrate, bytesize=bytesize, 
                                     parity=parity, stopbits=stopbits, timeout=timeout)
            self.signals.log_message.emit(f"ESP32通信初始化成功: {port}, {baudrate}bps")
            return True
        except Exception as e:
            self.signals.error_occurred.emit("ESP32通信", f"初始化失败: {str(e)}")
            self.signals.log_message.emit(f"ESP32通信初始化失败: {str(e)}")
            self.esp32 = None
            return False

    def connect_laser(self):
        """连接激光传感器"""
        if not self.laser:
            self.signals.error_occurred.emit("激光传感器", "未初始化")
            self.signals.log_message.emit("激光传感器未初始化")
            return False
        
        try:
            if self.laser.connect():
                self.laser_connected = True
                self.signals.connection_status_changed.emit("laser", True)
                self.signals.log_message.emit("激光传感器连接成功")
                return True
            else:
                self.laser_connected = False
                self.signals.connection_status_changed.emit("laser", False)
                self.signals.log_message.emit("激光传感器连接失败")
                return False
        except Exception as e:
            self.laser_connected = False
            self.signals.error_occurred.emit("激光传感器", f"连接失败: {str(e)}")
            self.signals.connection_status_changed.emit("laser", False)
            self.signals.log_message.emit(f"激光传感器连接失败: {str(e)}")
            return False
    
    def connect_height_sensor(self):
        """连接高度传感器"""
        if not self.height_sensor:
            self.signals.error_occurred.emit("高度传感器", "未初始化")
            self.signals.log_message.emit("高度传感器未初始化")
            return False
        
        try:
            if self.height_sensor.connect():
                self.height_connected = True
                self.signals.connection_status_changed.emit("height", True)
                self.signals.log_message.emit("高度传感器连接成功")
                return True
            else:
                self.height_connected = False
                self.signals.connection_status_changed.emit("height", False)
                self.signals.log_message.emit("高度传感器连接失败")
                return False
        except Exception as e:
            self.height_connected = False
            self.signals.error_occurred.emit("高度传感器", f"连接失败: {str(e)}")
            self.signals.connection_status_changed.emit("height", False)
            self.signals.log_message.emit(f"高度传感器连接失败: {str(e)}")
            return False
    
    def connect_esp32(self):
        """连接ESP32"""
        if not self.esp32:
            self.signals.error_occurred.emit("ESP32通信", "未初始化")
            self.signals.log_message.emit("ESP32通信未初始化")
            return False
        
        try:
            if self.esp32.connect():
                self.esp32_connected = True
                self.signals.connection_status_changed.emit("esp32", True)
                self.signals.log_message.emit("ESP32通信连接成功")
                return True
            else:
                self.esp32_connected = False
                self.signals.connection_status_changed.emit("esp32", False)
                self.signals.log_message.emit("ESP32通信连接失败")
                return False
        except Exception as e:
            self.esp32_connected = False
            self.signals.error_occurred.emit("ESP32通信", f"连接失败: {str(e)}")
            self.signals.connection_status_changed.emit("esp32", False)
            self.signals.log_message.emit(f"ESP32通信连接失败: {str(e)}")
            return False

    def laser_callback(self, data):
        # 处理激光测距数据
        if isinstance(data, Exception):
            self.signals.error_occurred.emit("激光传感器", str(data))
            self.signals.log_message.emit(f"激光传感器错误: {str(data)}")
            self.laser_connected = False
            self.signals.connection_status_changed.emit("laser", False)
        else:
            self.signals.laser_data_updated.emit(data)
    
    def read_laser(self):
        """读取激光测距数据的线程函数"""
        while self.running and self.laser:
            try:
                if not self.laser.ser.is_open:
                    if not self.laser.connect():
                        self.signals.error_occurred.emit("激光传感器", "连接丢失，尝试重连失败")
                        self.signals.log_message.emit("激光传感器连接丢失，尝试重连失败")
                        self.laser_connected = False
                        self.signals.connection_status_changed.emit("laser", False)
                        time.sleep(1)
                        continue
                    else:
                        self.laser_connected = True
                        self.signals.connection_status_changed.emit("laser", True)
                        self.signals.log_message.emit("激光传感器重连成功")
                # 这里不需要额外处理，因为LaserRangeFinder类已经有回调处理
            except Exception as e:
                self.signals.error_occurred.emit("激光传感器", f"读取失败: {str(e)}")
                self.signals.log_message.emit(f"激光传感器读取失败: {str(e)}")
                self.laser_connected = False
                self.signals.connection_status_changed.emit("laser", False)
                time.sleep(1)
    
    def read_height(self):
        """读取高度传感器数据的线程函数"""
        while self.running and self.height_sensor:
            try:
                if not self.height_sensor.ser or not self.height_sensor.ser.is_open:
                    if not self.height_sensor.connect():
                        self.signals.error_occurred.emit("高度传感器", "连接丢失，尝试重连失败")
                        self.signals.log_message.emit("高度传感器连接丢失，尝试重连失败")
                        self.height_connected = False
                        self.signals.connection_status_changed.emit("height", False)
                        time.sleep(1)
                        continue
                    else:
                        self.height_connected = True
                        self.signals.connection_status_changed.emit("height", True)
                        self.signals.log_message.emit("高度传感器重连成功")
                
                distance = self.height_sensor.read_distance()
                if distance is not None:
                    self.signals.height_data_updated.emit(distance)
            except Exception as e:
                self.signals.error_occurred.emit("高度传感器", f"读取失败: {str(e)}")
                self.signals.log_message.emit(f"高度传感器读取失败: {str(e)}")
                self.height_connected = False
                self.signals.connection_status_changed.emit("height", False)
                if self.height_sensor:
                    self.height_sensor.close()
                time.sleep(1)
            time.sleep(0.1)  # 避免过于频繁的读取

    def control_send(self, th1, th2, th3, th4, th5, th6):
        """向ESP32发送控制信号"""
        if not self.esp32:
            self.signals.error_occurred.emit("ESP32通信", "未初始化")
            self.signals.log_message.emit("ESP32通信未初始化")
            return False
        
        try:
            if not self.esp32.esp32_ser or not self.esp32.esp32_ser.is_open:
                if not self.esp32.connect():
                    self.signals.error_occurred.emit("ESP32通信", "连接丢失，尝试重连失败")
                    self.signals.log_message.emit("ESP32通信连接丢失，尝试重连失败")
                    self.esp32_connected = False
                    self.signals.connection_status_changed.emit("esp32", False)
                    return False
                else:
                    self.esp32_connected = True
                    self.signals.connection_status_changed.emit("esp32", True)
                    self.signals.log_message.emit("ESP32通信重连成功")
            
            if self.esp32.Control_send(th1, th2, th3, th4, th5, th6):
                data = self.esp32.esp32_ser.readline().decode('utf-8').strip()
                if data:
                    self.signals.log_message.emit(f"ESP32返回: {data}")
                return True
            else:
                return False
        except Exception as e:
            self.signals.error_occurred.emit("ESP32通信", f"发送失败: {str(e)}")
            self.signals.log_message.emit(f"ESP32通信发送失败: {str(e)}")
            self.esp32_connected = False
            self.signals.connection_status_changed.emit("esp32", False)
            if self.esp32:
                self.esp32.close()
            return False
    
    def start(self):
        """启动所有传感器读取线程"""
        if self.running:
            return
        
        self.running = True
        self.signals.log_message.emit("启动传感器读取...")
        
        # 启动激光传感器读取线程
        if self.laser and self.laser_connected:
            try:
                laser_thread = threading.Thread(target=self.laser.start_continuous_blocking, 
                                               args=(self.laser_callback,), 
                                               daemon=True)
                laser_thread.start()
                self.signals.log_message.emit("激光传感器数据读取线程已启动")
            except Exception as e:
                self.signals.error_occurred.emit("激光传感器", f"启动线程失败: {str(e)}")
                self.signals.log_message.emit(f"激光传感器启动线程失败: {str(e)}")
        
        # 启动高度传感器读取线程
        if self.height_sensor and self.height_connected:
            try:
                height_thread = threading.Thread(target=self.read_height, daemon=True)
                height_thread.start()
                self.signals.log_message.emit("高度传感器数据读取线程已启动")
            except Exception as e:
                self.signals.error_occurred.emit("高度传感器", f"启动线程失败: {str(e)}")
                self.signals.log_message.emit(f"高度传感器启动线程失败: {str(e)}")
    
    def stop(self):
        """停止所有传感器读取"""
        if not self.running:
            return
            
        self.running = False
        self.signals.log_message.emit("停止所有传感器...")
        
        # 关闭激光传感器
        if self.laser:
            try:
                self.laser.stop()
                self.laser_connected = False
                self.signals.connection_status_changed.emit("laser", False)
                self.signals.log_message.emit("激光传感器已停止")
            except Exception as e:
                self.signals.log_message.emit(f"关闭激光传感器时出错: {str(e)}")
        
        # 关闭高度传感器
        if self.height_sensor:
            try:
                self.height_sensor.close()
                self.height_connected = False
                self.signals.connection_status_changed.emit("height", False)
                self.signals.log_message.emit("高度传感器已停止")
            except Exception as e:
                self.signals.log_message.emit(f"关闭高度传感器时出错: {str(e)}")
        
        # 关闭ESP32通信
        if self.esp32:
            try:
                self.esp32.close()
                self.esp32_connected = False
                self.signals.connection_status_changed.emit("esp32", False)
                self.signals.log_message.emit("ESP32通信已停止")
            except Exception as e:
                self.signals.log_message.emit(f"关闭ESP32通信时出错: {str(e)}")
                
    def emergency_stop(self):
        """紧急停止所有设备"""
        self.signals.log_message.emit("紧急停止！停止所有设备...")
        
        # 先发送停止命令到ESP32
        if self.esp32 and self.esp32_connected:
            try:
                # 发送中立位置（停止命令）
                neutral_values = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
                self.control_send(*neutral_values)
                self.signals.log_message.emit("紧急停止命令已发送到ESP32")
            except Exception as e:
                self.signals.log_message.emit(f"发送紧急停止命令失败: {str(e)}")
                
        # 然后停止所有传感器
        self.stop()


# 主窗口控制器类
class MainWindowController:
    def __init__(self, window):
        self.ui = window
        
        # 初始化传感器信号和读取器
        self.sensor_signals = SensorSignals()
        self.sensor_reader = SensorReader(self.sensor_signals)
        
        # 设置日志信号连接
        self.sensor_signals.log_message.connect(self.log_message)
        self.sensor_signals.error_occurred.connect(self.log_error)
        self.sensor_signals.connection_status_changed.connect(self.update_connection_status)
        self.sensor_signals.laser_data_updated.connect(self.update_laser_data)
        self.sensor_signals.height_data_updated.connect(self.update_height_data)
        
        # 设置端口刷新按钮连接
        self.ui.pushButtonLaserRefresh.clicked.connect(self.refresh_laser_ports)
        self.ui.pushButtonHeightRefresh.clicked.connect(self.refresh_height_ports)
        self.ui.pushButtonESP32Refresh.clicked.connect(self.refresh_esp32_ports)
        self.ui.pushButtonTempDepthRefresh.clicked.connect(self.refresh_tempdepth_ports)
        
        # 设置连接按钮连接
        self.ui.pushButtonLaserConnect.clicked.connect(self.connect_laser)
        self.ui.pushButtonHeightConnect.clicked.connect(self.connect_height)
        self.ui.pushButtonESP32Connect.clicked.connect(self.connect_esp32)
        self.ui.pushButtonTempDepthConnect.clicked.connect(self.connect_tempdepth)
        
        # 设置摄像头按钮连接
        self.ui.pushButtonWebCamScan.clicked.connect(self.scan_webcam)
        self.ui.pushButtonStereoCameraDetect.clicked.connect(self.detect_stereocam)
        self.ui.pushButtonWebCamConnect.clicked.connect(self.connect_webcam)
        self.ui.pushButtonStereoCameraConnect.clicked.connect(self.connect_stereocam)
        self.ui.pushButtonCameraSnapshot.clicked.connect(self.take_snapshot)
        self.ui.pushButtonCameraRecord.clicked.connect(self.toggle_recording)
        
        # 设置紧急停止按钮连接
        self.ui.pushButtonEmergencyStop.clicked.connect(self.emergency_stop)
        
        # 设置日志按钮连接
        self.ui.pushButtonClearLog.clicked.connect(self.clear_log)
        self.ui.pushButtonSaveLog.clicked.connect(self.save_log)
        
        # 初始化端口列表
        self.refresh_all_ports()
        
        # 数据记录变量
        self.recording = False
        self.record_file = None
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.record_data)
        
        # 视频录制变量
        self.video_recording = False
        self.video_recorder = None
        
        # 初始化UI状态
        self.log_message("系统初始化完成")
    
    def refresh_all_ports(self):
        """刷新所有可用串口"""
        self.refresh_laser_ports()
        self.refresh_height_ports()
        self.refresh_esp32_ports()
        self.refresh_tempdepth_ports()
    
    def refresh_laser_ports(self):
        """刷新激光传感器可用串口"""
        self.ui.comboBoxLaserPort.clear()
        ports = self.get_available_ports()
        for port in ports:
            self.ui.comboBoxLaserPort.addItem(port)
        self.log_message(f"刷新激光传感器串口列表，找到{len(ports)}个可用串口")
    
    def refresh_height_ports(self):
        """刷新高度传感器可用串口"""
        self.ui.comboBoxHeightPort.clear()
        ports = self.get_available_ports()
        for port in ports:
            self.ui.comboBoxHeightPort.addItem(port)
        self.log_message(f"刷新高度传感器串口列表，找到{len(ports)}个可用串口")
    
    def refresh_esp32_ports(self):
        """刷新ESP32可用串口"""
        self.ui.comboBoxESP32Port.clear()
        ports = self.get_available_ports()
        for port in ports:
            self.ui.comboBoxESP32Port.addItem(port)
        self.log_message(f"刷新ESP32串口列表，找到{len(ports)}个可用串口")
    
    def refresh_tempdepth_ports(self):
        """刷新温深传感器可用串口"""
        self.ui.comboBoxTempDepthPort.clear()
        ports = self.get_available_ports()
        for port in ports:
            self.ui.comboBoxTempDepthPort.addItem(port)
        self.log_message(f"刷新温深传感器串口列表，找到{len(ports)}个可用串口")
    
    def get_available_ports(self):
        """获取系统中可用的串口列表"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append(port.device)
        return ports
    
    def connect_laser(self):
        """连接激光传感器"""
        if self.ui.pushButtonLaserConnect.text() == "连接":
            port = self.ui.comboBoxLaserPort.currentText()
            if not port:
                self.log_error("激光传感器", "请选择串口")
                return
            
            self.log_message(f"正在连接激光传感器 {port}...")
            if self.sensor_reader.init_laser(port):
                if self.sensor_reader.connect_laser():
                    self.ui.pushButtonLaserConnect.setText("断开")
                    self.ui.comboBoxLaserPort.setEnabled(False)
                    self.ui.labelLaserStatus.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
        else:
            # 断开连接
            if self.sensor_reader.laser:
                self.sensor_reader.laser.stop()
                self.ui.pushButtonLaserConnect.setText("连接")
                self.ui.comboBoxLaserPort.setEnabled(True)
                self.ui.labelLaserStatus.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
                self.ui.labelLaserValue.setText("0.00 m")
                self.log_message("激光传感器已断开连接")
    
    def connect_height(self):
        """连接高度传感器"""
        if self.ui.pushButtonHeightConnect.text() == "连接":
            port = self.ui.comboBoxHeightPort.currentText()
            if not port:
                self.log_error("高度传感器", "请选择串口")
                return
            
            self.log_message(f"正在连接高度传感器 {port}...")
            if self.sensor_reader.init_height_sensor(port):
                if self.sensor_reader.connect_height_sensor():
                    self.ui.pushButtonHeightConnect.setText("断开")
                    self.ui.comboBoxHeightPort.setEnabled(False)
                    self.ui.labelHeightStatus.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
        else:
            # 断开连接
            if self.sensor_reader.height_sensor:
                self.sensor_reader.height_sensor.close()
                self.ui.pushButtonHeightConnect.setText("连接")
                self.ui.comboBoxHeightPort.setEnabled(True)
                self.ui.labelHeightStatus.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
                self.ui.labelHeightValue.setText("0.00 m")
                self.log_message("高度传感器已断开连接")
    
    def connect_esp32(self):
        """连接ESP32"""
        if self.ui.pushButtonESP32Connect.text() == "连接":
            port = self.ui.comboBoxESP32Port.currentText()
            if not port:
                self.log_error("ESP32", "请选择串口")
                return
            
            self.log_message(f"正在连接ESP32 {port}...")
            if self.sensor_reader.init_esp32(port):
                if self.sensor_reader.connect_esp32():
                    self.ui.pushButtonESP32Connect.setText("断开")
                    self.ui.comboBoxESP32Port.setEnabled(False)
                    self.ui.labelESP32Status.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
                    self.ui.labelESP32Value.setText("已连接")
        else:
            # 断开连接
            if self.sensor_reader.esp32:
                self.sensor_reader.esp32.close()
                self.ui.pushButtonESP32Connect.setText("连接")
                self.ui.comboBoxESP32Port.setEnabled(True)
                self.ui.labelESP32Status.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
                self.ui.labelESP32Value.setText("未连接")
                self.log_message("ESP32已断开连接")
    
    def connect_tempdepth(self):
        """连接温深传感器"""
        if self.ui.pushButtonTempDepthConnect.text() == "连接":
            port = self.ui.comboBoxTempDepthPort.currentText()
            if not port:
                self.log_error("温深传感器", "请选择串口")
                return
            
            self.log_message(f"正在连接温深传感器 {port}...")
            # 这里需要实际实现温深传感器连接逻辑
            # 临时模拟成功连接
            self.ui.pushButtonTempDepthConnect.setText("断开")
            self.ui.comboBoxTempDepthPort.setEnabled(False)
            self.ui.labelTempDepthStatus.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
            self.ui.labelTempDepthValue.setText("25.3℃ / 1.45m")
            self.log_message("温深传感器连接成功")
        else:
            # 断开连接
            self.ui.pushButtonTempDepthConnect.setText("连接")
            self.ui.comboBoxTempDepthPort.setEnabled(True)
            self.ui.labelTempDepthStatus.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
            self.ui.labelTempDepthValue.setText("0.00℃ / 0.00m")
            self.log_message("温深传感器已断开连接")
    
    def update_connection_status(self, sensor_type, status):
        """更新连接状态"""
        if sensor_type == "laser":
            if status:
                self.ui.labelLaserStatus.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
                self.ui.pushButtonLaserConnect.setText("断开")
                self.ui.comboBoxLaserPort.setEnabled(False)
            else:
                self.ui.labelLaserStatus.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
                self.ui.pushButtonLaserConnect.setText("连接")
                self.ui.comboBoxLaserPort.setEnabled(True)
                self.ui.labelLaserValue.setText("0.00 m")
        elif sensor_type == "height":
            if status:
                self.ui.labelHeightStatus.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
                self.ui.pushButtonHeightConnect.setText("断开")
                self.ui.comboBoxHeightPort.setEnabled(False)
            else:
                self.ui.labelHeightStatus.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
                self.ui.pushButtonHeightConnect.setText("连接")
                self.ui.comboBoxHeightPort.setEnabled(True)
                self.ui.labelHeightValue.setText("0.00 m")
        elif sensor_type == "esp32":
            if status:
                self.ui.labelESP32Status.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
                self.ui.pushButtonESP32Connect.setText("断开")
                self.ui.comboBoxESP32Port.setEnabled(False)
                self.ui.labelESP32Value.setText("已连接")
            else:
                self.ui.labelESP32Status.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
                self.ui.pushButtonESP32Connect.setText("连接")
                self.ui.comboBoxESP32Port.setEnabled(True)
                self.ui.labelESP32Value.setText("未连接")
    
    def update_laser_data(self, data):
        """更新激光测距数据"""
        self.ui.labelLaserValue.setText(f"{data:.2f} m")
    
    def update_height_data(self, data):
        """更新高度传感器数据"""
        self.ui.labelHeightValue.setText(f"{data:.2f} m")
    
    def log_message(self, message):
        """记录日志"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.ui.textEditSystemLog.append(log_entry)
        # 滚动到底部
        scrollbar = self.ui.textEditSystemLog.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def log_error(self, error_type, error_message):
        """记录错误日志"""
        self.log_message(f"错误 ({error_type}): {error_message}")
    
    def emergency_stop(self):
        """紧急停止按钮响应"""
        self.log_message("触发紧急停止！")
        # 停止传感器和设备
        self.sensor_reader.emergency_stop()
    
    def clear_log(self):
        """清空日志"""
        self.ui.textEditSystemLog.clear()
        self.log_message("日志已清空")
    
    def save_log(self):
        """保存日志"""
        log_text = self.ui.textEditSystemLog.toPlainText()
        if not log_text:
            self.log_message("没有日志内容可保存")
            return
        
        # 选择保存路径
        filename, _ = QFileDialog.getSaveFileName(
            self.ui, "保存日志", "", "文本文件 (*.txt);;所有文件 (*)"
        )
        
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(log_text)
                self.log_message(f"日志已保存到 {filename}")
            except Exception as e:
                self.log_error("文件操作", f"保存日志失败: {str(e)}")
    
    def record_data(self):
        """记录传感器数据"""
        if not self.recording or not self.record_file:
            return
        
        try:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            laser_data = self.ui.labelLaserValue.text().replace(" m", "")
            height_data = self.ui.labelHeightValue.text().replace(" m", "")
            
            self.record_file.write(f"{timestamp},{laser_data},{height_data}\n")
            self.record_file.flush()
        except Exception as e:
            self.log_error("数据记录", f"记录数据失败: {str(e)}")
            self.stop_recording()
    
    def start_recording(self):
        """开始记录数据"""
        if self.recording:
            return
        
        # 选择保存路径
        filename, _ = QFileDialog.getSaveFileName(
            self.ui, "保存数据记录", "", "CSV文件 (*.csv);;所有文件 (*)"
        )
        
        if not filename:
            return
        
        try:
            self.record_file = open(filename, 'w', encoding='utf-8')
            # 写入CSV头
            self.record_file.write("时间戳,激光距离(m),高度(m)\n")
            self.recording = True
            self.record_timer.start(100)  # 每100毫秒记录一次
            self.log_message(f"开始记录数据到 {filename}")
            self.ui.checkBoxRECALLDATA.setChecked(True)
        except Exception as e:
            self.log_error("数据记录", f"开始记录失败: {str(e)}")
            if self.record_file:
                self.record_file.close()
                self.record_file = None
    
    def stop_recording(self):
        """停止记录数据"""
        if not self.recording:
            return
        
        self.record_timer.stop()
        self.recording = False
        
        if self.record_file:
            self.record_file.close()
            self.record_file = None
        
        self.log_message("停止记录数据")
        self.ui.checkBoxRECALLDATA.setChecked(False)
    
    def scan_webcam(self):
        """扫描网络摄像头"""
        self.log_message("正在扫描网络摄像头...")
        # 模拟扫描
        import random
        ips = ["192.168.1." + str(random.randint(100, 150)) for _ in range(3)]
        
        self.ui.comboBoxWebCamIP.clear()
        for ip in ips:
            self.ui.comboBoxWebCamIP.addItem(ip)
        
        self.log_message(f"找到 {len(ips)} 个网络摄像头")
    
    def detect_stereocam(self):
        """检测双目摄像头"""
        self.log_message("正在检测双目摄像头...")
        # 模拟检测
        self.ui.comboBoxStereoCameraID.clear()
        self.ui.comboBoxStereoCameraID.addItem("0")
        self.ui.comboBoxStereoCameraID.addItem("1")
        self.log_message("检测到2个摄像头设备")
    
    def connect_webcam(self):
        """连接网络摄像头"""
        if self.ui.pushButtonWebCamConnect.text() == "连接":
            ip = self.ui.comboBoxWebCamIP.currentText()
            if not ip:
                self.log_error("网络摄像头", "请输入IP地址")
                return
            
            self.log_message(f"正在连接网络摄像头 {ip}...")
            # 模拟连接成功
            self.ui.pushButtonWebCamConnect.setText("断开")
            self.ui.comboBoxWebCamIP.setEnabled(False)
            self.ui.labelWebCamStatus.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
            self.ui.labelWebCamValue.setText("已连接")
            self.log_message(f"网络摄像头 {ip} 连接成功")
        else:
            # 断开连接
            self.ui.pushButtonWebCamConnect.setText("连接")
            self.ui.comboBoxWebCamIP.setEnabled(True)
            self.ui.labelWebCamStatus.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
            self.ui.labelWebCamValue.setText("未连接")
            self.log_message("网络摄像头已断开连接")
    
    def connect_stereocam(self):
        """连接双目摄像头"""
        if self.ui.pushButtonStereoCameraConnect.text() == "连接":
            cam_id = self.ui.comboBoxStereoCameraID.currentText()
            if not cam_id:
                self.log_error("双目摄像头", "请选择设备ID")
                return
            
            self.log_message(f"正在连接双目摄像头 {cam_id}...")
            # 模拟连接成功
            self.ui.pushButtonStereoCameraConnect.setText("断开")
            self.ui.comboBoxStereoCameraID.setEnabled(False)
            self.ui.labelStereoCameraStatus.setStyleSheet("background-color: rgb(0, 255, 0); border-radius: 10px;")
            self.ui.labelStereoCameraValue.setText("已连接")
            self.log_message(f"双目摄像头 {cam_id} 连接成功")
        else:
            # 断开连接
            self.ui.pushButtonStereoCameraConnect.setText("连接")
            self.ui.comboBoxStereoCameraID.setEnabled(True)
            self.ui.labelStereoCameraStatus.setStyleSheet("background-color: rgb(255, 0, 0); border-radius: 10px;")
            self.ui.labelStereoCameraValue.setText("未连接")
            self.log_message("双目摄像头已断开连接")
    
    def take_snapshot(self):
        """拍摄照片"""
        # 检查是否有摄像头连接
        if (self.ui.labelWebCamStatus.styleSheet().find("rgb(0, 255, 0)") == -1 and 
            self.ui.labelStereoCameraStatus.styleSheet().find("rgb(0, 255, 0)") == -1):
            self.log_error("摄像头", "没有连接的摄像头")
            return
        
        self.log_message("正在拍摄照片...")
        
        # 选择保存路径
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename, _ = QFileDialog.getSaveFileName(
            self.ui, "保存照片", f"snapshot_{timestamp}.jpg", "图像文件 (*.jpg *.png);;所有文件 (*)"
        )
        
        if filename:
            # 模拟拍照保存
            self.log_message(f"照片已保存到 {filename}")
        else:
            self.log_message("已取消拍照")
    
    def toggle_recording(self):
        """切换视频录制状态"""
        if not self.video_recording:
            # 检查是否有摄像头连接
            if (self.ui.labelWebCamStatus.styleSheet().find("rgb(0, 255, 0)") == -1 and 
                self.ui.labelStereoCameraStatus.styleSheet().find("rgb(0, 255, 0)") == -1):
                self.log_error("摄像头", "没有连接的摄像头")
                return
            
            # 选择保存路径
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename, _ = QFileDialog.getSaveFileName(
                self.ui, "保存视频", f"video_{timestamp}.mp4", "视频文件 (*.mp4 *.avi);;所有文件 (*)"
            )
            
            if not filename:
                return
            
            # 开始录制
            self.video_recording = True
            self.ui.pushButtonCameraRecord.setText("停止录像")
            self.ui.pushButtonCameraRecord.setStyleSheet("background-color: rgb(255, 0, 0); color: rgb(255, 255, 255);")
            self.log_message(f"开始录制视频到 {filename}")
        else:
            # 停止录制
            self.video_recording = False
            self.ui.pushButtonCameraRecord.setText("录像")
            self.ui.pushButtonCameraRecord.setStyleSheet("")
            self.log_message("视频录制已停止")
    
    def handle_close_event(self, event):
        """处理窗口关闭事件"""
        self.log_message("系统即将关闭，停止所有设备...")
        self.sensor_reader.stop()
        
        # 停止数据记录
        if self.recording:
            self.stop_recording()
            
        # 停止视频录制
        if self.video_recording:
            self.toggle_recording()
            
        event.accept()


def main():
    # 先创建QUiLoader
    loader = QUiLoader()
    
    # 然后创建应用程序
    app = QtWidgets.QApplication(sys.argv)
    
    # 设置应用程序样式
    app.setStyle("Fusion")
    
    # 加载UI文件 - 使用绝对路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    ui_path = os.path.join(current_dir, "main_window.ui")
    ui_file = QtCore.QFile(ui_path)
    if not ui_file.exists():
        print(f"错误：找不到UI文件：{ui_path}")
        sys.exit(1)
        
    ui_file.open(QtCore.QFile.ReadOnly)
    # loader.load会创建窗口并加载UI
    window = loader.load(ui_file)
    ui_file.close()
    
    # 这里创建我们的主窗口控制器，而不是窗口本身
    controller = MainWindowController(window)
    
    # 添加关闭事件处理
    window.closeEvent = lambda event: controller.handle_close_event(event)
    
    # 显示窗口
    window.show()
    
    # 执行应用程序
    sys.exit(app.exec())


if __name__ == "__main__":
    main() 