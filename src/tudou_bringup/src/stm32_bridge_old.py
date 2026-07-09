#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

# 请根据你们 usartx.h 中的宏定义确认这两个值，通常为 0x7B 和 0x7D
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')
        
        # 1. 串口配置 (根据 ELF 2 实际连接的端口修改，通常为 /dev/ttyUSB0)
        self.port_name = '/dev/ttyACM0'
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
            self.get_logger().info(f"✅ 成功连接 STM32 底盘串口: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            raise SystemExit

        # 2. ROS 2 话题与 TF 广播器
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 3. 里程计积分变量
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # 4. 开启定时器，100Hz 高频读取串口缓冲 (确保不漏帧)
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def cmd_callback(self, msg):
        """处理下发的 cmd_vel，组装 11 字节协议帧并发送给 STM32"""
        # 单位转换: m/s -> mm/s
        vx = int(msg.linear.x * 1000)
        vy = int(msg.linear.y * 1000)
        vz = int(msg.angular.z * 1000)
        
        # 转为 16位无符号整数 (处理负数补码)，便于拆分高低 8 位
        vx_u16 = vx & 0xFFFF
        vy_u16 = vy & 0xFFFF
        vz_u16 = vz & 0xFFFF

        # 组装数据负载: [控制模式(0=正常), 备用(0), Vx_H, Vx_L, Vy_H, Vy_L, Vz_H, Vz_L]
        payload = [
            0x00, 0x00, 
            (vx_u16 >> 8) & 0xFF, vx_u16 & 0xFF,
            (vy_u16 >> 8) & 0xFF, vy_u16 & 0xFF,
            (vz_u16 >> 8) & 0xFF, vz_u16 & 0xFF
        ]
        
        # 计算校验和 (帧头 + 负载的异或)
        checksum = FRAME_HEADER
        for b in payload:
            checksum ^= b
            
        # 组装完整 11 字节帧并发送
        frame = bytearray([FRAME_HEADER] + payload + [checksum, FRAME_TAIL])
        self.ser.write(frame)

    def read_serial_data(self):
        """读取并解析 STM32 发送的 24 字节数据帧"""
        while self.ser.in_waiting >= 24:
            # 寻找帧头
            if self.ser.read(1)[0] == FRAME_HEADER:
                # 读取后续的 23 字节
                rest_data = self.ser.read(23)
                if len(rest_data) == 23 and rest_data[-1] == FRAME_TAIL:
                    # 校验和验证 (Mode 1 是对发送数据的前 22 个字节求异或，这里用对应的接收端逻辑)
                    checksum_calc = FRAME_HEADER
                    for i in range(21):
                        checksum_calc ^= rest_data[i]
                        
                    if checksum_calc == rest_data[21]:
                        self.parse_sensor_data(rest_data)
                    else:
                        self.get_logger().warn("⚠️ 传感器帧校验和错误，已丢弃")

    def parse_sensor_data(self, data):
        """解析解包后的有效数据，计算里程计并发布"""
        # 解析速度 (大端模式，有符号 16 位)
        vx = int.from_bytes(data[1:3], byteorder='big', signed=True) / 1000.0  # m/s
        vy = int.from_bytes(data[3:5], byteorder='big', signed=True) / 1000.0  # m/s
        vz = int.from_bytes(data[5:7], byteorder='big', signed=True) / 1000.0  # rad/s 或 m/s (依据车型)

        # 这里还可以提取 IMU 和电压数据，按需补充
        # accel_x = int.from_bytes(data[7:9], byteorder='big', signed=True) ...
        # voltage = int.from_bytes(data[19:21], byteorder='big', signed=False) / 1000.0
        
        self.publish_odometry(vx, vy, vz)

    def publish_odometry(self, vx, vy, vth):
        """里程计运动学积分与 ROS 2 话题/TF 发布"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # 根据机器人中心速度计算各轴增量
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.last_time = current_time

        # 1. 发布 TF 变换 (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # 欧拉角转四元数 (仅 Yaw)
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # 2. 发布 /odom 话题
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

    def send_zero_velocity(self):
        """发送全零速度帧，用于紧急停止"""
        payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        checksum = FRAME_HEADER
        for b in payload:
            checksum ^= b
        frame = bytearray([FRAME_HEADER] + payload + [checksum, FRAME_TAIL])
        try:
            self.ser.write(frame)
            self.get_logger().info("🛑 已发送全零速度急停帧！")
        except Exception as e:
            self.get_logger().error(f"❌ 发送急停帧失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 当按下 Ctrl+C 时，会进入这里
        node.get_logger().info('检测到 Ctrl+C，正在紧急制动...')
        node.send_zero_velocity()
    finally:
        node.destroy_node()
        rclpy.shutdown()

# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


if __name__ == '__main__':
    main()