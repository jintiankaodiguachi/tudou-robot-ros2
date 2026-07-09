#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import qos_profile_sensor_data

# 视觉与图像处理依赖
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 系统与网络通信依赖
import serial
import threading
import subprocess
import os
import requests
import base64
import sys 

class LingjingVlmAgent(Node):
    def __init__(self):
        super().__init__('lingjing_vlm_agent')
        
        self.bridge = CvBridge()
        self.current_room_name = "未知病房"
        self.is_navigating = False
        
        # 📂 核心工作路径参数配置
        self.qwen_cwd = "/home/elf/hoho/vlbot/qw2vl/"
        self.image_path = "/home/elf/hoho/vlbot/qw2vl/captured.jpg"
        self.nurse_server_url = "http://10.42.0.1:5000/api/report" # 笔记本有线网关 API
        
        # 串口物理绑定
        self.declare_parameter('port', '/dev/ttyUSB0') 
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.5)
            self.get_logger().info(f"🎉 天问语音桥物理打通！监听串口: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ 无法打开语音串口 {port}: {e}")
            return

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 坐标密码本（精准对接你日志中的一号病房真实坐标）
        self.room_goals = {
            0x01: {'name': '1号病房', 'x': 4.07, 'y': -4.81, 'z': 0.998, 'w': 0.053}, 
            0x02: {'name': '2号病房', 'x': 2.61, 'y': 0.87, 'z': 0.707, 'w': 0.707}
        }
        
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_listener_loop)
        self.read_thread.start()
        self.get_logger().info("🚀 灵境大模型巡检总控全线战备！")

    def serial_listener_loop(self):
        while self.running and rclpy.ok():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(1)
                if not data: continue
                cmd = data[0]
                if cmd in self.room_goals:
                    room = self.room_goals[cmd]
                    self.get_logger().info(f"🎯 语音匹配成功：【{room['name']}】")
                    self.start_navigation_task(room)

    def start_navigation_task(self, room_config):
        if self.is_navigating: return
        self.nav_client.wait_for_server()
        self.current_room_name = room_config['name']
        self.is_navigating = True
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = room_config['x']
        goal_msg.pose.pose.position.y = room_config['y']
        goal_msg.pose.pose.orientation.z = room_config['z']
        goal_msg.pose.pose.orientation.w = room_config['w']
        
        self.get_logger().info(f"📍 释放底盘制动，自主导航前往：【self.current_room_name】...")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_navigating = False
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        self.is_navigating = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🎉 成功合围目标点【{self.current_room_name}】！激活单帧高精捕获...")
            self.img_sub = self.create_subscription(
                Image, 
                '/camera/camera/color/image_raw', 
                self.camera_snapshot_callback, 
                qos_profile_sensor_data            
            )

    def camera_snapshot_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(self.image_path, cv_image)
            self.get_logger().info(f"📸 现场高精度图片数据落盘成型！")
            self.destroy_subscription(self.img_sub)
            threading.Thread(target=self.process_vlm_and_report).start()
        except Exception as e:
            self.get_logger().error(f"❌ 快门图像捕获异常: {e}")

    def process_vlm_and_report(self):
        """ 第四步：改用 unbuffered 字符流状态机，100% 免疫无换行挂起，彻底清洗脏标签 """
        self.get_logger().info("🧠 唤醒端侧嵌入式 NPU，Qwen2-VL 大脑多模态计算中...")
        cmd = ["./demo", self.image_path, "qwen2_vl_2b_vision_rk3588.rknn", "Qwen2-VL-2B-Instruct.rkllm", "128", "512"]
        prompt_input = "<image> 你现在是一个智能医疗巡检机器人。请仔细观察这张病房图片，告诉我床上的病人是否处于安全状态，是否有摔倒在地上、吊针快输完了或者急需医护人员处理的突发紧急事件。请用清晰简短的中文回答。\n"
        
        try:
            # 🛠️ 核心加固：将 bufsize 设为 0（无缓冲），强行进入实时字节流交互模式
            proc = subprocess.Popen(
                cmd, cwd=self.qwen_cwd, 
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, 
                text=True, bufsize=0
            )
            proc.stdin.write(prompt_input)
            proc.stdin.flush()
            
            full_buffer = ""
            qwen_output = ""
            capture_start = False
            
            # 🛠️ 核心加固：改用 read(1) 逐字啃骨头，不再依赖 \n 换行符
            while True:
                char = proc.stdout.read(1)
                if not char: break
                full_buffer += char
                
                # 1. 发现暗号 "robot:"，说明正文开始了，把前面无用的系统提示和 user: 标签全部扔掉！
                if "robot:" in full_buffer and not capture_start:
                    capture_start = True
                    full_buffer = "" # 清空蓄水池，从这里开始只存真正的话！
                    continue
                
                # 2. 一旦开启捕获，只要后面的字里拼出了下一个 "user:"（不管有没有换行符！）
                if capture_start and "user:" in full_buffer:
                    # 拦腰斩断！只要 user: 之前的内容，彻底拔掉末尾残留的脏后缀
                    qwen_output = full_buffer.split("user:")[0].strip()
                    self.get_logger().info("⚡ [流式中断] 成功抓获下一轮交互标识，0秒无缝强行破网！")
                    break
            
            # 拿到货了，立刻将 C++ 交互死循环暴力击杀
            proc.kill()
            
        except Exception as e:
            self.get_logger().error(f"❌ 边缘多模态交互异常: {str(e)}")
            return

        # 强行冲刷车机本地的 Debug 打印
        print("\n" + "="*50)
        print(f"💡 [Qwen2-VL 终极纯净中文回复]:\n{qwen_output}")
        print("="*50 + "\n")
        sys.stdout.flush() 

        if qwen_output and len(qwen_output) > 5:
            # 推送给笔记本 Flask 监护大屏
            self.upload_to_nurse_station(qwen_output)
        else:
            self.get_logger().error("❌ 清洗后发现空数据，取消网络上报。")

    def upload_to_nurse_station(self, analysis_text):
        try:
            with open(self.image_path, "rb") as img_file:
                base64_image = base64.b64encode(img_file.read()).decode('utf-8')
            payload = {
                "robot_id": "L_POTATO_01",
                "location": self.current_room_name,
                "vlm_analysis": analysis_text,
                "image_data": base64_image,
                "status_level": "EMERGENCY" if any(x in analysis_text for x in ["摔倒", "异常", "危险", "倒地"]) else "NORMAL"
            }
            headers = {'Content-Type': 'application/json'}
            response = requests.post(self.nurse_server_url, json=payload, headers=headers, timeout=5)
            if response.status_code == 200:
                print("✅ [NET_OK] 纯净无标签报告已自主送达护士站大屏！")
                sys.slate_window = None
                sys.stdout.flush()
        except Exception as e:
            print(f"❌ 跨机网络上报故障: {e}")
            sys.stdout.flush()

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LingjingVlmAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()