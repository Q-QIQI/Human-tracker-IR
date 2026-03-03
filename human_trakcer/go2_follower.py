#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
import json

# 🟢 引入宇树 ROS2 专用的请求消息
from unitree_api.msg import Request

class Go2Follower(Node):
    def __init__(self):
        super().__init__('go2_follower')
        self.get_logger().info("🐶 Go2 Advanced Follower (unitree_ros2 API) Initialized!")

        # ================= ⚙️ 运动控制参数 =================
        self.MIN_FOLLOW_SPEED_X = 0.3    # 最小跟随速度 (m/s)
        self.MAX_FOLLOW_SPEED_X = 1.0    # 最大跟随速度 (m/s)
        self.TURN_SPEED_YAW = 0.6        # 标准转向角速度 (rad/s)
        self.SEARCH_TURN_SPEED = 0.5     # 丢失后的搜寻角速度

        # 距离映射参数
        self.PERSON_CLOSE_DIST = 1.0     # 停止距离 (米)
        self.PERSON_FAR_DIST = 2.5       # 加速的远距离 (米)
        self.HORIZONTAL_CENTER_TOL = 0.15 # 画面中心允许的偏差百分比
        
        self.IMG_WIDTH = 848.0 
        # ==================================================

        # 状态记录
        self.last_msg_time = time.time()
        self.last_known_x_percent = 0.5
        self.is_searching = False
        self.search_start_time = 0.0
        self.MAX_SEARCH_TIME = 8.0       # 原地转圈寻找的最长时间

        # 接收 Tracker 发来的目标位置
        self.sub_target = self.create_subscription(
            Point, '/human_tracker/target', self.target_callback, 10)
        
        # 🟢 发布控制指令给 Go2 高级运动服务话题
        self.pub_cmd = self.create_publisher(Request, '/api/sport/request', 10)

        # 50Hz 控制循环
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # 缓存目标状态
        self.target_locked = False
        self.target_z = 0.0
        self.target_x = 0.0

    def send_unitree_move_cmd(self, vx, vy, vyaw):
        """ 🟢 核心封装：向 Unitree ROS2 接口发送 Move 指令 """
        req = Request()
        # Header 设置
        req.header.identity.id = int(time.time() * 1000) # 生成时间戳 ID
        req.header.identity.api_id = 1008                # 1008 是 SportClient 的 Move API ID
        
        # Parameter 设置 (必须是严格格式的 JSON 字符串)
        # x: 前进/后退, y: 左横移/右横移, z: 左转/右转
        param_dict = {
            "x": float(vx), 
            "y": float(vy), 
            "z": float(vyaw)
        }
        req.parameter = json.dumps(param_dict)
        
        self.pub_cmd.publish(req)

    def target_callback(self, msg):
        self.last_msg_time = time.time()
        
        if msg.z > 0.5: # 1.0 表示锁定中
            self.target_locked = True
            self.target_z = msg.x
            self.target_x = msg.y
            self.last_known_x_percent = self.target_x / self.IMG_WIDTH
            self.is_searching = False
        else:
            self.target_locked = False

    def control_loop(self):
        now = time.time()
        time_since_last_msg = now - self.last_msg_time

        # 1. 致命超时保护 (视觉节点死机时急刹车)
        if time_since_last_msg > 1.0:
            self.stop_robot()
            return

        vx = 0.0
        vyaw = 0.0

        # 2. 正常跟随模式
        if self.target_locked:
            # --- 前后控制 ---
            speed_factor = (self.target_z - self.PERSON_CLOSE_DIST) / (self.PERSON_FAR_DIST - self.PERSON_CLOSE_DIST)
            speed_factor = max(0.0, min(1.0, speed_factor)) 
            
            if self.target_z < self.PERSON_CLOSE_DIST:
                vx = 0.0 # 离得太近，停下
            else:
                vx = self.MIN_FOLLOW_SPEED_X + (self.MAX_FOLLOW_SPEED_X - self.MIN_FOLLOW_SPEED_X) * speed_factor

            # --- 左右控制 ---
            norm_center_x = self.target_x / self.IMG_WIDTH
            if norm_center_x < (0.5 - self.HORIZONTAL_CENTER_TOL):
                vyaw = self.TURN_SPEED_YAW # 目标偏左，向左转
            elif norm_center_x > (0.5 + self.HORIZONTAL_CENTER_TOL):
                vyaw = -self.TURN_SPEED_YAW # 目标偏右，向右转

            # --- 减速过弯 ---
            if abs(vyaw) > 0.1 and abs(norm_center_x - 0.5) > 0.25:
                vx *= 0.3 

            self.send_unitree_move_cmd(vx, 0.0, vyaw)

        # 3. 丢失目标后的智能搜寻模式
        else:
            if not self.is_searching:
                self.is_searching = True
                self.search_start_time = now
                self.get_logger().warn("Target lost! Initiating smart search rotation...")

            search_duration = now - self.search_start_time
            if search_duration < self.MAX_SEARCH_TIME:
                # 往消失的方向原地转圈找人
                if self.last_known_x_percent < 0.5:
                    vyaw = self.SEARCH_TURN_SPEED
                else:
                    vyaw = -self.SEARCH_TURN_SPEED
                
                self.send_unitree_move_cmd(0.0, 0.0, vyaw)
            else:
                self.stop_robot()

    def stop_robot(self):
        """ 发送速度全为 0 的指令进行急刹车 """
        self.send_unitree_move_cmd(0.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = Go2Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()