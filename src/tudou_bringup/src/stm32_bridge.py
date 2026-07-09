#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

# 确认串口帧头帧尾
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')
        
        # 1. 串口初始化配置
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
        vx = int(msg.linear.x * 1000)
        vy = int(msg.linear.y * 1000)
        vz = int(msg.angular.z * 1000)
        
        vx_u16 = vx & 0xFFFF
        vy_u16 = vy & 0xFFFF
        vz_u16 = vz & 0xFFFF

        payload = [
            0x00, 0x00, 
            (vx_u16 >> 8) & 0xFF, vx_u16 & 0xFF,
            (vy_u16 >> 8) & 0xFF, vy_u16 & 0xFF,
            (vz_u16 >> 8) & 0xFF, vz_u16 & 0xFF
        ]
        
        checksum = FRAME_HEADER
        for b in payload:
            checksum ^= b
            
        frame = bytearray([FRAME_HEADER] + payload + [checksum, FRAME_TAIL])
        try:
            if self.ser.is_open:
                self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f"❌ 发送控制帧失败: {e}")

    def read_serial_data(self):
        """读取并解析数据帧（增加了防崩溃的异常捕捉与高频自动重连机制）"""
        try:
            # 🌟 自动断线重连守护：如果串口关闭了，在后台默默尝试重新打开，直到成功
            if not self.ser.is_open:
                self.ser.open()
                self.get_logger().info("🔌 掉线守护：串口已自动成功恢复并重新打开！")
                return

            while self.ser.in_waiting >= 24:
                # 寻找帧头
                if self.ser.read(1)[0] == FRAME_HEADER:
                    # 读取后续的 23 字节
                    rest_data = self.ser.read(23)
                    if len(rest_data) == 23 and rest_data[-1] == FRAME_TAIL:
                        # 校验和验证
                        checksum_calc = FRAME_HEADER
                        for i in range(21):
                            checksum_calc ^= rest_data[i]
                            
                        if checksum_calc == rest_data[21]:
                            self.parse_sensor_data(rest_data)
                        else:
                            self.get_logger().warn("⚠️ 传感器帧校验和错误，已丢弃")
        except (serial.SerialException, OSError) as e:
            # 🌟 核心修改点 1：捕获大电流压降或物理震动导致的 Errno 5 物理掉线异常，绝不崩溃自杀！
            self.get_logger().error(f"⚠️ 串口通信突发异常: {e}，正在优雅重置串口，等待自动重连...")
            try:
                self.ser.close() # 释放被系统锁死的坏句柄
            except Exception:
                pass
            # 退出当前周期，把生存空间留给下一个 10ms 的定时器轮询去上面重新 open()

    def parse_sensor_data(self, data):
        """解析解包后的有效数据，计算里程计并发布"""
        vx = int.from_bytes(data[1:3], byteorder='big', signed=True) / 1000.0  # m/s
        vy = int.from_bytes(data[3:5], byteorder='big', signed=True) / 1000.0  # m/s
        
        # 🌟 核心修改点 2：强行加上负号进行航向镜像校正！
        # 彻底终结“真车右转90度，RViz里向左转90度”的神经分裂行为
        vz = -int.from_bytes(data[5:7], byteorder='big', signed=True) / 1000.0  # rad/s

        self.publish_odometry(vx, vy, vz)

    def publish_odometry(self, vx, vy, vth):
        """里程计运动学积分与 ROS 2 话题/TF 发布"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.last_time = current_time

        # 1. 发布 TF 变换 (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
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
            if self.ser.is_open:
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
        node.get_logger().info('检测到 Ctrl+C，正在紧急制动...')
        node.send_zero_velocity()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()