#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# 可用，总是报正常那个

# 改进了误报警，通过优化提示词工程

# 改进了误报警, 但是失败，模型卡死，按中止传图片，大模型未响应

# 可用，但是误报警

# 针对上一版的小改进

# 换图失败版本（加了wifi)

# 修复 全循环巡护不显示的问题，对应m.py

# 已完善单点巡护,进一步完善监护端显示（可用版本）

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

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
import time       
import traceback  

class LingjingVlmAgent(Node):
    def __init__(self):
        super().__init__('tudou_vlm_agent2')
        
        self.bridge = CvBridge()
        self.current_room_name = "未知病房"
        
        # 🛠️ 核心闭环控制状态机变量
        self.is_patrolling = False      
        self.patrol_sequence = []       
        self.current_step_index = 0     
        self.current_global_mission = "全线战备守候中" 
        
        # 📸 拍照控制旗帜 (防止动态创建/销毁订阅引发的死锁)
        self.need_snapshot = False
        
        # 🧭 终点母港坐标
        self.initial_pose = {
            'name': '初始点', 
            'x': -0.036,                  
            'y': 0.13,                 
            'z': 0.904,                 
            'w': 0.427                  
        }
        
        # 📂 核心工作路径参数配置
        self.qwen_cwd = "/home/elf/hoho/vlbot/qw2vl/"
        self.image_path = "/home/elf/hoho/vlbot/qw2vl/captured.jpg"
        
        # ⚠️ 请确保这里的 IP 和端口与你的前端后台完全一致（如：http://10.42.0.1:8000/api/report）
        self.nurse_server_url = "http://192.168.137.119:5000/api/report" 
        self.watchdog_server_url = "http://192.168.137.119:5000/api/status"
        
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
        
        # 📸 优化：改为静态订阅。10 代表默认 QoS (Reliable)，兼容性最高。
        self.img_sub = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.camera_snapshot_callback, 
            10          
        )
        
        # 坐标密码本
        self.room_goals = {
            0x01: {'name': '1号病房', 'x': 1.19, 'y': 1.57, 'z': 0.46, 'w': 0.89}, 
            0x02: {'name': '2号病房', 'x': 2.57, 'y': -1.68, 'z': 0.38, 'w': 0.93}
        }
        
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_listener_loop)
        self.read_thread.start()
        self.get_logger().info("🚀 灵境大模型多点双轨总控全线战备！")

    def report_status_to_watchdog(self, sub_task_text, global_mission_text=None):
        """双轨制报文发送（增加了超时容错与显式报错）"""
        try:
            if global_mission_text:
                self.current_global_mission = global_mission_text
                
            payload = {
                "global_task": self.current_global_mission, 
                "sub_task": sub_task_text
            }
            # 将 timeout 放宽至 1.0s，防止阻塞同时避免轻微网络抖动导致的丢包
            response = requests.post(self.watchdog_server_url, json=payload, headers={'Content-Type': 'application/json'}, timeout=1.0)
            if response.status_code != 200:
                self.get_logger().warn(f"⚠️ 状态上报成功但服务器返回异常状态码: {response.status_code}")
        except Exception as e:
            # 绝不盲目 release 异常，让操作人员在终端能看到网络问题
            self.get_logger().error(f"🌐 状态上报看护端失败，请检查网络或URL! 错误原因: {e}")

    def serial_listener_loop(self):
        while self.running and rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(1)
                    if not data: continue
                    cmd = data[0]
                    
                    if self.is_patrolling:
                        self.get_logger().warn("⚠️ 小车正处于流水线任务中，忽略此次语音。")
                        continue
                    
                    if cmd == 0x01:
                        self.get_logger().info("🎯 语音匹配：【1号房单点巡护】")
                        self.patrol_sequence = [self.room_goals[0x01], self.initial_pose]
                        self.current_step_index = 0
                        self.is_patrolling = True
                        self.report_status_to_watchdog("语音识别成功，正在组装链条...", "1号房单点闭环巡护")
                        self.start_next_sequence_target()
                        
                    elif cmd == 0x02:
                        self.get_logger().info("🎯 语音匹配：【2号房单点巡护】")
                        self.patrol_sequence = [self.room_goals[0x02], self.initial_pose]
                        self.current_step_index = 0
                        self.is_patrolling = True
                        self.report_status_to_watchdog("语音识别成功，正在组装链条...", "2号房单点闭环巡护")
                        self.start_next_sequence_target()
                        
                    elif cmd == 0x03: 
                        self.get_logger().info("🎯 语音匹配：【全流程大循环巡护】")
                        self.patrol_sequence = [self.room_goals[0x01], self.room_goals[0x02], self.initial_pose]
                        self.current_step_index = 0
                        self.is_patrolling = True
                        self.report_status_to_watchdog("下达全勤指令！正在初始化所有房间巡护链条...", "全流程大循环巡护")
                        time.sleep(0.5) 
                        self.start_next_sequence_target()
            except Exception as thread_err:
                self.get_logger().error(f"❌ 串口异常:\n{traceback.format_exc()}")
                time.sleep(1.0)

    def start_next_sequence_target(self):
        try:
            if self.current_step_index < len(self.patrol_sequence):
                room_config = self.patrol_sequence[self.current_step_index]
                self.current_room_name = room_config['name']
                
                self.report_status_to_watchdog(f"🚀 小车已释放轮系制动，正在导航奔赴: 【{self.current_room_name}】...")
                
                if not self.nav_client.wait_for_server(timeout_sec=2.0):
                    self.get_logger().error("❌ Nav2 导航核心未启动！强行跳过...")
                    self.emergency_skip_next()
                    return
                
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = float(room_config['x'])
                goal_msg.pose.pose.position.y = float(room_config['y'])
                goal_msg.pose.pose.orientation.z = float(room_config['z'])
                goal_msg.pose.pose.orientation.w = float(room_config['w'])
                
                send_goal_future = self.nav_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self.goal_response_callback)
            else:
                self.is_patrolling = False
                self.report_status_to_watchdog("就位初始点，等待语音唤醒", "全线战备守候中")
        except Exception as e:
            self.get_logger().error(f"调度层异常: {e}")
            self.emergency_skip_next()

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.emergency_skip_next()
                return
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.nav_result_callback)
        except Exception:
            self.emergency_skip_next()

    def nav_result_callback(self, future):
        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                if self.current_room_name == "初始点":
                    self.report_status_to_watchdog("🏠 土豆已回到监护站")
                    self.current_step_index += 1
                    time.sleep(2.0) 
                    self.start_next_sequence_target()
                else:
                    # 🎯 到达目标病房，举起截屏请求信号灯
                    self.get_logger().info(f"📍 已抵达【{self.current_room_name}】，等待相机送入新鲜帧...")
                    self.need_snapshot = True 
            else:
                self.emergency_skip_next()
        except Exception:
            self.emergency_skip_next()



    def camera_snapshot_callback(self, msg):
        # 如果当前不需要拍照，直接放行，不消耗算力
        if not self.need_snapshot:
            return
            
        try:
            self.need_snapshot = False # 立刻锁死标志位，防止单次到达触发多次拍照
            self.get_logger().info("📸 正在拦截视频流并固化为图像文件...")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(self.image_path, cv_image)
            
            # 开启异步线程处理大模型，不阻塞 ROS2 核心执行器
            threading.Thread(target=self.process_vlm_and_report).start()
        except Exception as e:
            self.get_logger().error(f"❌ 图像拦截或保存失败: {e}")
            self.emergency_skip_next()

    def process_vlm_and_report(self):
        try:
            self.report_status_to_watchdog(f"📌 小车已稳稳合围【{self.current_room_name}】目标位姿！\n🧠 边缘端 NPU 强力唤醒，Qwen2-VL 正在深度观察思考现场画面...")
            
            cmd = ["./demo", self.image_path, "qwen2_vl_2b_vision_rk3588.rknn", "Qwen2-VL-2B-Instruct.rkllm", "128", "512"]
            # 💡 优化后的高阶提示词，增加思维链约束
            prompt_input = """<image> 你是一名专业的病房医疗巡检机器人助手。请仔细观察图片，执行以下多步思维任务： 1. 【环境感知】：识别图中是否有患者，判断患者当前的姿态（如：站立、坐着、平躺、异常倒地）。 2. 【医疗监测】：检查输液状态（如：输液管内是否有液体流动，液位是否过低）。 3. 【逻辑判断】：仅当发现“有人摔倒在地上”或“吊针药水完全见底需要立即更换”时，才视为紧急情况。如果图片显示患者正常卧床、静坐，或者输液正常，即为正常状态。 4. 【否定过滤】：如果图片中包含“没有跌倒”、“没有异常”等否定含义的语境，请务必判定为正常。  请严格按照以下格式回答，不要输出任何额外的废话： [状态]: 正常 / 紧急 [原因]: (在此简短写出分析理由)"""
            
            qwen_output = "大模型未响应"
            proc = subprocess.Popen(cmd, cwd=self.qwen_cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=0)
            
            # 🛠️ 必须强制追加 "\n" 换行符！发送回车提交信号，瞬间激活 C++ 脑中枢！
            proc.stdin.write(prompt_input + "\n")
            proc.stdin.flush()
            
            full_buffer = ""
            capture_start = False
            
            import select
            while True:
                timeout_gate = 1.5 if (capture_start and "[原因]:" in full_buffer) else 25.0
                rlist, _, _ = select.select([proc.stdout], [], [], timeout_gate)
                
                if not rlist:
                    if capture_start and "[原因]:" in full_buffer:
                        qwen_output = full_buffer.strip()
                    break
                    
                char = proc.stdout.read(1)
                if not char: break
                full_buffer += char
                if "robot:" in full_buffer and not capture_start:
                    capture_start = True
                    full_buffer = "" 
                    continue
                if capture_start and "user:" in full_buffer:
                    qwen_output = full_buffer.split("user:")[0].strip()
                    break
            proc.kill()
            
            if qwen_output and len(qwen_output) > 5:
                self.upload_to_nurse_station(qwen_output)
                
            self.report_status_to_watchdog(f"✅ 【{self.current_room_name}】巡查完毕，成果已上传！正在解除底盘控制锁...")
            time.sleep(1.0)
        except Exception as e:
            self.get_gradient_logger().error(f"大模型流报错: {e}")

        self.current_step_index += 1
        self.start_next_sequence_target()

    def upload_to_nurse_station(self, analysis_text):
        try:
            with open(self.image_path, "rb") as img_file:
                base64_image = base64.b64encode(img_file.read()).decode('utf-8')
            
            # 💡 优化后的逻辑判断：严格匹配 Prompt 中的 [状态]: 紧急 格式
            status_level = "EMERGENCY" if "[状态]: 紧急" in analysis_text else "NORMAL"
            
            payload = {
                "robot_id": "L_POTATO_01",
                "location": self.current_room_name,
                "vlm_analysis": analysis_text,
                "image_data": base64_image,
                "status_level": status_level
            }
            requests.post(self.nurse_server_url, json=payload, headers={'Content-Type': 'application/json'}, timeout=5)
        except Exception:
            pass

    def process_vlm_and_report(self):
        try:
            self.report_status_to_watchdog(f"📌 小车已稳稳合围【{self.current_room_name}】目标位姿！\n🧠 边缘端 NPU 强力唤醒，Qwen2-VL 正在深度观察思考现场画面...")
            
            cmd = ["./demo", self.image_path, "qwen2_vl_2b_vision_rk3588.rknn", "Qwen2-VL-2B-Instruct.rkllm", "128", "512"]
            # 💡 优化后的高阶提示词，增加思维链约束
            prompt_input = """<image> 你是一名专业的病房医疗巡检机器人助手。请仔细观察图片，执行以下多步思维任务： 1. 【环境感知】：识别图中是否有患者，判断患者当前的姿态（如：站立、坐着、平躺、异常倒地）。 2. 【医疗监测】：检查输液状态（如：输液管内是否有液体流动，液位是否过低）。 3. 【逻辑判断】：仅当发现“有人摔倒在地上”或“吊针药水完全见底需要立即更换”时，才视为紧急情况。如果图片显示患者正常卧床、静坐，或者输液正常，即为正常状态。 4. 【否定过滤】：如果图片中包含“没有跌倒”、“没有异常”等否定含义的语境，请务必判定为正常。  请严格按照以下格式回答，不要输出任何额外的废话： [状态]: 正常 / 紧急 [原因]: (在此简短写出分析理由)"""
            
            qwen_output = "大模型未响应"
            proc = subprocess.Popen(cmd, cwd=self.qwen_cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=0)
            
            # 🛠️ 终极黄金修正：必须强制追加 "\n" 换行符！发送回车提交信号，瞬间激活 C++ 脑中枢！
            proc.stdin.write(prompt_input + "\n")
            proc.stdin.flush()
            
            full_buffer = ""
            capture_start = False
            
            import select
            while True:
                timeout_gate = 1.5 if (capture_start and "[原因]:" in full_buffer) else 25.0
                rlist, _, _ = select.select([proc.stdout], [], [], timeout_gate)
                
                if not rlist:
                    if capture_start and "[原因]:" in full_buffer:
                        qwen_output = full_buffer.strip()
                    break
                    
                char = proc.stdout.read(1)
                if not char: break
                full_buffer += char
                if "robot:" in full_buffer and not capture_start:
                    capture_start = True
                    full_buffer = "" 
                    continue
                if capture_start and "user:" in full_buffer:
                    qwen_output = full_buffer.split("user:")[0].strip()
                    break
            proc.kill()
            
            if qwen_output and len(qwen_output) > 5:
                self.upload_to_nurse_station(qwen_output)
                
            self.report_status_to_watchdog(f"✅ 【{self.current_room_name}】巡查完毕，成果已上传！正在解除底盘控制锁...")
            time.sleep(1.0)
        except Exception as e:
            self.get_gradient_logger().error(f"大模型流报错: {e}")

        self.current_step_index += 1
        self.start_next_sequence_target()

    def upload_to_nurse_station(self, analysis_text):
        try:
            with open(self.image_path, "rb") as img_file:
                base64_image = base64.b64encode(img_file.read()).decode('utf-8')
            
            # 💡 优化后的逻辑判断：严格匹配 Prompt 中的 [状态]: 紧急 格式
            status_level = "EMERGENCY" if "[状态]: 紧急" in analysis_text else "NORMAL"
            
            payload = {
                "robot_id": "L_POTATO_01",
                "location": self.current_room_name,
                "vlm_analysis": analysis_text,
                "image_data": base64_image,
                "status_level": status_level
            }
            requests.post(self.nurse_server_url, json=payload, headers={'Content-Type': 'application/json'}, timeout=5)
        except Exception:
            pass

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