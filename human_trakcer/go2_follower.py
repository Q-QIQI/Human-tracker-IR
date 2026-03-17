#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
import json

from unitree_api.msg import Request

class Go2Follower(Node):
    def __init__(self):
        super().__init__('go2_follower')
        self.get_logger().info("Go2 Advanced Follower (unitree_ros2 API) Initialized!")

       
        self.MIN_FOLLOW_SPEED_X = 0.3    
        self.MAX_FOLLOW_SPEED_X = 1.0    
        self.TURN_SPEED_YAW = 0.6       
        self.SEARCH_TURN_SPEED = 0.5     

        
        self.PERSON_CLOSE_DIST = 1.0    
        self.PERSON_FAR_DIST = 2.5       
        self.HORIZONTAL_CENTER_TOL = 0.15 
        
        self.IMG_WIDTH = 848.0 
       

      
        self.last_msg_time = time.time()
        self.last_known_x_percent = 0.5
        self.is_searching = False
        self.search_start_time = 0.0
        
        self.COASTING_TIME = 1.5        
        self.MAX_SEARCH_TIME = 8.0       

       
        self.sub_target = self.create_subscription(
            Point, '/human_tracker/target', self.target_callback, 10)
        
      
        self.pub_cmd = self.create_publisher(Request, '/api/sport/request', 10)

        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.target_locked = False
        self.target_z = 0.0
        self.target_x = 0.0

        self.last_vx = 0.0
        self.last_vyaw = 0.0

    def send_unitree_move_cmd(self, vx, vy, vyaw):
        
        req = Request()
        
        req.header.identity.id = int(time.time() * 1000) 
        req.header.identity.api_id = 1008    
        
        param_dict = {
            "x": float(vx), 
            "y": float(vy), 
            "z": float(vyaw)
        }
        req.parameter = json.dumps(param_dict)
        
        self.pub_cmd.publish(req)

    def target_callback(self, msg):
        self.last_msg_time = time.time()
        
        if msg.z > 0.5: 
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

        if time_since_last_msg > 1.0:
            self.stop_robot()
            return

        vx = 0.0
        vyaw = 0.0


        if self.target_locked:
         
            speed_factor = (self.target_z - self.PERSON_CLOSE_DIST) / (self.PERSON_FAR_DIST - self.PERSON_CLOSE_DIST)
            speed_factor = max(0.0, min(1.0, speed_factor)) 
            
            if self.target_z < self.PERSON_CLOSE_DIST:
                vx = 0.0 
            else:
                vx = self.MIN_FOLLOW_SPEED_X + (self.MAX_FOLLOW_SPEED_X - self.MIN_FOLLOW_SPEED_X) * speed_factor

          
            norm_center_x = self.target_x / self.IMG_WIDTH
            if norm_center_x < (0.5 - self.HORIZONTAL_CENTER_TOL):
                vyaw = self.TURN_SPEED_YAW # 目标偏左，向左转
            elif norm_center_x > (0.5 + self.HORIZONTAL_CENTER_TOL):
                vyaw = -self.TURN_SPEED_YAW # 目标偏右，向右转

            if abs(vyaw) > 0.1 and abs(norm_center_x - 0.5) > 0.25:
                vx *= 0.3 

            self.last_vx = vx
            self.last_vyaw = vyaw

            self.send_unitree_move_cmd(vx, 0.0, vyaw)

        else:
            if not self.is_searching:
                self.is_searching = True
                self.search_start_time = now
                self.get_logger().warn("Target occluded! Entering state machine...")

            search_duration = now - self.search_start_time

            if search_duration < self.COASTING_TIME:
                
                self.last_vx *= 0.95
                self.last_vyaw *= 0.95
                
                self.send_unitree_move_cmd(self.last_vx, 0.0, self.last_vyaw)

            elif search_duration < self.MAX_SEARCH_TIME:
              
                if self.last_known_x_percent < 0.5:
                    vyaw = self.SEARCH_TURN_SPEED
                else:
                    vyaw = -self.SEARCH_TURN_SPEED
                
                self.send_unitree_move_cmd(0.0, 0.0, vyaw)

           
            else:
                self.stop_robot()

    def stop_robot(self):
        
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
