#! /usr/bin/env python3
# coding:UTF-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/Visual_location/visual_location/scripts")
import rospy
import time
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import Quaternion
import device_model
'''
data = {
    'time':[],
    'AngX':[],
    'AngY':[],
    'AngZ':[],
    'AsX':[],
}
'''

data_time = [0,0,0,0,0,0,0,0,0,0]
data_AngX = [0,0,0,0,0,0,0,0,0,0]
data_AngY = [0,0,0,0,0,0,0,0,0,0]
data_AngZ = [0,0,0,0,0,0,0,0,0,0]
data_AsX = [0,0,0,0,0,0,0,0,0,0]


def updateData(DeviceModel):
    # 获取当前时间戳
    global data_AngX,data_AngY,data_AngZ,data_AsX,data_time
    current_time = time.time()
    
    # 获取设备的数据
    ang_x = DeviceModel.get(DeviceModel.addrLis[0], "AngX")
    ang_y = DeviceModel.get(DeviceModel.addrLis[0], "AngY")
    ang_z = DeviceModel.get(DeviceModel.addrLis[0], "AngZ")
    as_x = DeviceModel.get(DeviceModel.addrLis[0], "AsX")
    
    #print(f'AngX={ang_x}, AngY={ang_y}, AngZ={ang_z}, AsX={as_x}\n')
    #rospy.loginfo("%f,%f,%f,%f",ang_x,ang_y,ang_z,as_x)
    # 打印数据
    #print(f'AngX={ang_x}, AngY={ang_y}, AngZ={ang_z}, AsX={as_x}\n')

    # 将数据保存到列表中
    '''
    data['time'].append(current_time)
    data['AngX'].append(ang_x)
    data['AngY'].append(ang_y)
    data['AngZ'].append(ang_z)
    data['AsX'].append(as_x)
    '''
    if not ((ang_x == None) or (ang_y == None) or (ang_z == None) or (as_x == None)):
        data_AngX.pop(0)
        data_AngX.append(ang_x)
        
        data_AngY.pop(0)
        data_AngY.append(ang_y)
            
        data_AngZ.pop(0)
        data_AngZ.append(ang_z)
        
        data_AsX.pop(0)
        data_AsX.append(as_x)
   
    
    # 可以选择将数据保存到CSV文件
    '''
    with open('sensor_data.csv', mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([current_time, ang_x, ang_y, ang_z, as_x])
    '''
# 绘图
'''
def plot_data():
    # 画图
    plt.figure(figsize=(10, 6))
    
    # 绘制三个角度数据的曲线
    plt.plot(data['time'], data['AngX'], label='AngX', color='r')
    plt.plot(data['time'], data['AngY'], label='AngY', color='g')
    plt.plot(data['time'], data['AngZ'], label='AngZ', color='b')
    plt.plot(data['time'], data['AsX'], label='AsX', color='y')

    # 设置标题和标签
    plt.title('Sensor Data Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.legend(loc='upper right')
    
    # 显示图形
    plt.grid(True)
    plt.show()
'''
if __name__ == "__main__":
    # 读取的modbus地址列表 List of Modbus addresses read
    
    rospy.init_node("imu_node")
    imu_pub = rospy.Publisher("cmd_imu",Quaternion,queue_size=10)
    imu_msg = Quaternion()
    addrLis = [0x50]
    
    # 拿到设备模型 Get the device model
    device = device_model.DeviceModel("测试设备1", "/dev/ttyUSB0", 9600, addrLis, updateData, 30)
    
    
    # 开启设备 Turn on the device
    device.openDevice()
    device.startLoopRead()
    # 开启轮询 Enable loop reading
  
    #rospy.loginfo("%f,%f,%f,%f",imu_msg.x,imu_msg.y,imu_msg.z,imu_msg.w)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
       
        imu_msg.x = data_AngX[9]
        imu_msg.y = data_AngY[9]
        imu_msg.z = data_AngZ[9] 
        imu_msg.w = data_AsX[9]
        
        imu_pub.publish(imu_msg)
        
        rate.sleep()
        rospy.loginfo("%f,%f,%f,%f",imu_msg.x,imu_msg.y,imu_msg.z,imu_msg.w)
    
    
    # 等待一定时间，进行数据记录
    #time.sleep(30)
    
    # 停止轮询 Stop reading
    #device.stopLoopRead()
    
    # 等待一段时间再关闭设备 Close the device
    #time.sleep(1)
    #device.closeDevice()
    
    # 绘制记录的数据 Plot the data
    #plot_data()