# ROS 串口通信配置方法

## 下载serial软件包
首先下载serial软件包：
```
sudo apt-get install ros-noetic-serial
```
>注意：中间的`noetic`是ros的版本名称
需要换成自己的ros版本

检查是否安装成功
```bash
roscd serial
```
如果成功，会看到![alt text](image.png)

## 创建功能包并添加依赖
假设工作空间的名称为`catkin_ws`
在终端输入以下命令创建名为“serial_port”的功能包
```bash
cd ~/catkin_ws/src 
catkin_create_pkg serial_port roscpp rospy std_msgs serial
```
>!关键：一定要添加 `serial` 这个依赖，其余根据需要添加

## 添加串口通信的代码
在serial_port/src目录下添加`serial_port.py`文件
```bash
cd ~/catkin_ws/src/serial_port/src/
gedit serial_port.py
```

在`serial_port.py`文件内添加以下代码
```python
#!/usr/bin/env python3

'''串口通信'''
import rospy
import serial
from serial import SerialException
import struct
import numpy as np
from tf.transformations import euler_from_quaternion
import pandas as pd
# 话题
from nav_msgs.msg import Odometry
from topic.msg import U_control_vector

# 包头包尾
FRAME_HEADER=b'\xAA\xBB'
FRAME_FOOTER=b'\xCC\xDD'

# 发送和接收的有用数据的字节数，不计包头、包尾、名称、轨迹索引
SEND_DATA_SIZE=12*4
RECV_DATA_SIZE=4*4

# 控制频率[Hz]
CONTROL_RATE=50                    

class SerialPortNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('serial_port', anonymous=True)
        
        # 添加通信函数
        self.U_control_vector_publisher=rospy.Publisher('/reed_quad_x/U_control_vector',U_control_vector,queue_size=1)
        
        self.odometry_subscriber = rospy.Subscriber('/reed_quad_x/odometry_sensor1/odometry', Odometry, self.odometry_callback, queue_size=1)
        
        # 初始化串口
        self.sp = None
        self.init_serial_port()
        
        self.Nx=12
        self.Nu=4
        
        self.state = np.zeros(self.Nx)        # 当前状态 x_k
        self.controlVector=np.zeros(self.Nu)  # 控制量 u_k
        
        self.xk_history=[]
        self.uk_history=[]
    
    
    def odometry_callback(self, msg):
        # 订阅位置姿态信息
        (roll,pitch,yaw)=euler_from_quaternion([msg.pose.pose.orientation.x,
                                               msg.pose.pose.orientation.y, 
                                               msg.pose.pose.orientation.z,
                                               msg.pose.pose.orientation.w], axes='sxyz')

        self.state=np.array([msg.pose.pose.position.x,    msg.pose.pose.position.y,    msg.pose.pose.position.z,
                             msg.twist.twist.linear.x,    msg.twist.twist.linear.y,    msg.twist.twist.linear.z,
                            roll,pitch,yaw,
                            msg.twist.twist.angular.x,   msg.twist.twist.angular.y,   msg.twist.twist.angular.z])
        
    def init_serial_port(self):
        try:
            # 创建串口对象
            self.sp = serial.Serial(
                port='/dev/ttyUSB0',  # 串口设备名
                baudrate=115200,      # 波特率
                timeout=0.1           # 超时时间(秒)
            )
        except SerialException as e:
            rospy.logerr("Unable to open port: %s", str(e))
            return False
        
        if self.sp.is_open:
            rospy.loginfo("%s is opened.", self.sp.port)
            return True
        else:
            rospy.logerr("Port not open")
            return False
            
    def send_floats(self, float_list):
        # 发送数据
        
        if len(float_list) != 12:
            rospy.logerr("必须提供12个float数据")
            return
        
        try:
            # 打包数据（4字节 × 12 = 48字节）
            data = struct.pack('<12f', *float_list)
            
            # 添加帧头帧尾 (2+48+2=52字节)
            data=FRAME_HEADER+data+FRAME_FOOTER
            
            # 发送
            self.sp.write(data)
            rospy.loginfo(f"已发送 {len(data)} 字节数据")
            rospy.loginfo(f"发送的原始数据：{data}")
            
        except struct.error as e:
            rospy.logerr(f"打包失败: {e}")
        except serial.SerialException as e:
            rospy.logerr(f"发送失败: {e}")


    def receive_floats(self):
        # 接收数据
        try:
            self.sp.reset_input_buffer()
            
            # 读取直到帧尾
            raw_data = self.sp.read_until(FRAME_FOOTER)
            
            # 检查帧头和长度
            if raw_data.startswith(FRAME_HEADER) and len(raw_data) == 2+3+RECV_DATA_SIZE+2:
                name = raw_data[2] # 数据类型名称
                traj_idx= struct.unpack('<1H',raw_data[3:5]) # 轨迹索引
                rospy.logerr(f"traj_idx={traj_idx}")
                u_hex = raw_data[5:-2]  # 取出 u 的数据

                floats = struct.unpack('<4f', u_hex) # 变成4个float类型
                rospy.loginfo(f"解析成功")
                return floats
            else:
                rospy.logerr(f"无效数据帧:{raw_data}")
                return None

        except Exception as e:
            rospy.logerr(f"接收失败: {e}")
            return None
        
    def run(self):
        if not self.sp or not self.sp.is_open:
            rospy.logerr("Serial port not available")
            return
            
        # 数据传输
        rate = rospy.Rate(CONTROL_RATE)  # 50Hz
        while not rospy.is_shutdown():
            try:
                self.sp.reset_output_buffer()  # 清空输出缓冲区（待发送的数据）
                self.sp.reset_input_buffer()   # 清空输入缓冲区（待读取的数据）

                # 发送数据
                self.send_floats(self.state)
                self.xk_history.append(self.state)

                # # 用于测试
                # float_array = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
                # self.send_floats(float_array)
                
                # 接收数据
                received_floats=self.receive_floats()
                rospy.loginfo(f"收到数据: {received_floats}")
                # self.uk_history.append({"traj_idx": traj_idx, "received_floats": received_floats})
                self.uk_history.append(received_floats)
            
                # 取 FPGA 发过来的4个控制量, 发给 tinympc_ours
                if received_floats!=None:
                    self.controlVector=received_floats
                
                    # 把收到的数据发给 tinympc_ours.py
                    cmd=U_control_vector()
                    cmd.header.stamp=rospy.Time.now()
                    cmd.u_ControlVector=self.controlVector
                    self.U_control_vector_publisher.publish(cmd)
                
            except SerialException as e:
                rospy.logerr("Serial communication error: %s", str(e))
                break
                
            rate.sleep()
        
        # 关闭串口
        if self.sp.is_open:
            self.sp.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    try:
        serial_node = SerialPortNode()
        serial_node.run()
    except rospy.ROSInterruptException:
        df_xk_history=pd.DataFrame(serial_node.xk_history)
        df_xk_history.to_excel("/home/yhw/catkin_ws/src/serial_port/xk_history.xlsx")
        df_uk_history=pd.DataFrame(serial_node.uk_history)
        df_uk_history.to_excel("/home/yhw/catkin_ws/src/serial_port/uk_history.xlsx")
        pass
```

### 关键代码解释
#### 1. 串口的初始化
使用以下命令创建串口对象
```python
# 创建串口对象
self.sp = serial.Serial(
    port='/dev/ttyUSB0',  # 串口设备名
    baudrate=115200,      # 波特率
    timeout=0.1           # 超时时间(秒)
)
```
其中：
- `port`: 串口设备名，根据实际串口名称更改
- `baudrate`: 波特率，要与实际串口的波特率想匹配
- `timeout`: 超时时间，可以自行调整

如果创建失败，则要检测异常，相应代码如下：
```python
def init_serial_port(self):
    try:
        # 创建串口对象
        self.sp = serial.Serial(
            port='/dev/ttyUSB0',  # 串口设备名
            baudrate=115200,      # 波特率
            timeout=0.1           # 超时时间(秒)
        )
    except SerialException as e:
        rospy.logerr("Unable to open port: %s", str(e))
        return False
    
    if self.sp.is_open:
        rospy.loginfo("%s is opened.", self.sp.port)
        return True
    else:
        rospy.logerr("Port not open")
        return False
```

#### 2. 通过串口发送数据
在我的代码中，是发送12个float数据。需要根据实际情况进行修改。
打包后的数据结构
|  包头(2Byte)  |           12个float数据（48 Byte）          |  包尾(2Byte)  |
|-----|------|---|
| AA BB | ... | CC DD |

```python
def send_floats(self, float_list):
    # 发送数据
    if len(float_list) != 12:
        rospy.logerr("必须提供12个float数据")
        return
    
    try:
        # 打包数据（4字节 × 12 = 48字节）
        data = struct.pack('<12f', *float_list)
        
        # 添加帧头帧尾 (2+48+2=52字节)
        data=FRAME_HEADER+data+FRAME_FOOTER
        
        # 发送
        self.sp.write(data)
        rospy.loginfo(f"已发送 {len(data)} 字节数据")
        rospy.loginfo(f"发送的原始数据：{data}")
        
    except struct.error as e:
        rospy.logerr(f"打包失败: {e}")
    except serial.SerialException as e:
        rospy.logerr(f"发送失败: {e}")
```


#### 3. 通过串口接收数据
在我的代码中，是接收4个float数据。需要根据实际情况进行修改。
打包后的数据结构
|  包头(2Byte)  | name(1Byte) | 轨迹索引(2Byte) |   4个float数据（48 Byte）          |  包尾(2Byte)  |
|-----|---|---|---|---|
| AA BB | 01 | ... | ... | CC DD |

注意：这里的`name`只是作为一个标记
```python
def receive_floats(self):
    # 接收数据
    try:
        self.sp.reset_input_buffer()
        
        # 读取直到帧尾
        raw_data = self.sp.read_until(FRAME_FOOTER)
        
        # 检查帧头和长度
        if raw_data.startswith(FRAME_HEADER) and len(raw_data) == 2+3+RECV_DATA_SIZE+2:
            name = raw_data[2] # 数据类型名称
            traj_idx= struct.unpack('<1H',raw_data[3:5]) # 轨迹索引
            rospy.logerr(f"traj_idx={traj_idx}")
            u_hex = raw_data[5:-2]  # 取出 u 的数据

            floats = struct.unpack('<4f', u_hex) # 变成4个float类型
            rospy.loginfo(f"解析成功")
            return floats
        else:
            rospy.logerr(f"无效数据帧:{raw_data}")
            return None

    except Exception as e:
        rospy.logerr(f"接收失败: {e}")
        return None
```

#### 4. 主要运行流程
在`run`函数中包含了串口通信的主要流程:
1. 首先清空输出缓冲区和输入缓冲区
2. 调用上文的 `send_floats` 函数发送数据
3. 调用上文的 `receive_floats` 函数接收数据
4. 把接收到的数据发布出去

具体代码如下：
```python
def run(self):
    if not self.sp or not self.sp.is_open:
        # 检查串口是否创建且已经打开
        rospy.logerr("Serial port not available")
        return
        
    # 数据传输
    rate = rospy.Rate(CONTROL_RATE)  # 50Hz
    while not rospy.is_shutdown():
        try:
            self.sp.reset_output_buffer()  # 清空输出缓冲区（待发送的数据）
            self.sp.reset_input_buffer()   # 清空输入缓冲区（待读取的数据）

            # 发送数据
            self.send_floats(self.state)
            
            # 接收数据
            received_floats=self.receive_floats()
            rospy.loginfo(f"收到数据: {received_floats}")
        
            # 取 FPGA 发过来的4个控制量, 发布给 tinympc_ours.py
            if received_floats!=None:
                self.controlVector=received_floats
                cmd=U_control_vector()
                cmd.header.stamp=rospy.Time.now()
                cmd.u_ControlVector=self.controlVector
            
        except SerialException as e:
            rospy.logerr("Serial communication error: %s", str(e))
            break
            
        rate.sleep()
    
    # 关闭串口
    if self.sp.is_open:
        self.sp.close()
        rospy.loginfo("Serial port closed")
```

## 修改 `CMakeLists.txt`
在`serial_port/CMakeLists.txt`文件中添加：
```
catkin_install_python(PROGRAMS src/serial_port.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

最后，在终端中使用`catkin_make` 或者 `catkin build`进行编译，即可。