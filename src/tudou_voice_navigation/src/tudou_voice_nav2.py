#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import serial
import threading
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class HospitalVoiceNav(Node):
    def __init__(self):
        super().__init__('tudou_voice_nav2')
        
        # 1. 串口配置（Type-C 直连通常映射为 /dev/ttyUSB0 或 ttyUSB1）
        # ⚠️ 如果小车雷达已经占了 ttyUSB0，这里大概率是 /dev/ttyUSB1，请根据实际情况通过 ls /dev/ttyUSB* 确认
        self.declare_parameter('port', '/dev/ttyUSB0') 
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.5)
            self.get_logger().info(f"🎉 成功打通天问语音 Type-C 物理神经！监听端口: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ 无法打开语音串口 {port}: {e}。请检查线是否插紧或检查设备号！")
            return
            
        # 2. 建立 Nav2 Action 客户端（用于向导航栈下发病房目标点）
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. 【核心骨架】病房在静态地图上的高精绝对坐标配置字典
        # ⚠️ 这里的 X, Y, Z, W 坐标不能盲猜！必须是你用 slam_toolbox 建好医院地图后，
        # 在 Rviz2 里利用 "2D Nav Goal" 工具点选测试，并从终端日志里抄出来的数字（方法见第三部分）！
        self.room_goals = {
            0x01: { # 对应天问发送的 0x01 (1号病房)
                'name': '1号病房',
                'x': 4.0689,    # 离地图原点的横向距离(米)
                'y': -4.8141,    # 离地图原点的纵向距离(米)
                'z': 0.9985,     # 车头朝向旋转向量 Z
                'w': 0.0536      # 车头朝向旋转向量 W
            },
            0x02: { # 对应天问发送的 0x02 (2号病房)
                'name': '2号病房',
                'x': 3.45, 
                'y': -1.12, 
                'z': 0.707, 
                'w': 0.707
            }
        }
        
        # 4. 开启异步后台线程读取串口，防止阻塞 ROS 2 导致小车神经瘫痪
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_listener_loop)
        self.read_thread.start()

    def serial_listener_loop(self):
        while self.running and rclpy.ok():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(1) # 每次严格读取 1 字节
                if not data:
                    continue
                
                cmd = data[0] # 拿到字节数字
                self.get_logger().info(f"📡 串口收到天问模块代号: {hex(cmd)}")
                
                if cmd in self.room_goals:
                    room = self.room_goals[cmd]
                    self.get_logger().info(f"🚀 匹配成功！准备触发自主导航前往：【{room['name']}】")
                    self.execute_navigation(room)
                else:
                    self.get_logger().warn(f"⚠️ 收到未知代号: {hex(cmd)}，不予执行。")

    def execute_navigation(self, room_config):
        # 🎯 核心修改点：把 wait_for_action_server 改为 wait_for_server
        if not self.nav_client.wait_for_server(timeout_sec=4.0):
            self.get_logger().error("❌ Nav2 导航堆栈没有响应！请确保你已经成功运行了 Nav2 导航 launch！")
            return
            
        # 封装标准的 Nav2 目标点消息包
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = room_config['x']
        goal_msg.pose.pose.position.y = room_config['y']
        goal_msg.pose.pose.orientation.z = room_config['z']
        goal_msg.pose.pose.orientation.w = room_config['w']
        
        self.get_logger().info(f"🎯 正在向 Nav2 核心下发【{room_config['name']}】绝对坐标 -> X: {room_config['x']}, Y: {room_config['y']}")
        
        # 异步发送请求，机器人开始运动
        self.nav_client.send_goal_async(goal_msg)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HospitalVoiceNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()