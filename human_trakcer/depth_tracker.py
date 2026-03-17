#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
from ament_index_python.packages import get_package_share_directory

os.environ['YOLO_CHECKS'] = 'false'
from ultralytics import YOLO
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class DepthTracker(Node):

    def __init__(self):
        super().__init__('depth_tracker')
        self.bridge = CvBridge()
        self.get_logger().info("Initializing ROS2 2.5D Tracker with Anti-Occlusion (Single Model)...")

        # Load ONLY the fine-tuned custom model correctly from ROS2 package share
        pkg_share = get_package_share_directory('human_tracker')
        model_dir = os.path.join(pkg_share, 'models')

        custom_model_path = os.path.join(model_dir, 'best.pt')

        if not os.path.exists(custom_model_path):
            raise FileNotFoundError(f"Missing model: {custom_model_path}")

        # Initialize the single lightweight YOLOv8n model
        self.model_custom = YOLO(custom_model_path)

        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.frame_count = 0
        self.locked = False
        self.owner_pos = np.array([0.0, 0.0])
        self.owner_depth = 0.0
        self.owner_velocity = np.array([0.0, 0.0])
        self.owner_box = [0, 0, 0, 0]
        self.last_time = time.time()
        self.lost_frames = 0
        self.MAX_LOST_FRAMES = 60

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_depth = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_rect_raw', qos_profile=qos)

        self.sub_ir = message_filters.Subscriber(
            self, Image, '/camera/camera/infra1/image_rect_raw', qos_profile=qos)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_depth, self.sub_ir], queue_size=10, slop=0.1)

        self.ts.registerCallback(self.sync_callback)

        self.pub_annotated = self.create_publisher(
            Image, '/human_tracker/output', 10)

        self.pub_target = self.create_publisher(
            Point, '/human_tracker/target', 10)

    def get_iou(self, box1, box2):
        ix1 = max(box1[0], box2[0])
        iy1 = max(box1[1], box2[1])
        ix2 = min(box1[2], box2[2])
        iy2 = min(box1[3], box2[3])
        inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        a1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        a2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = a1 + a2 - inter
        return inter / union if union > 0 else 0.0

    def extract_robust_depth(self, depth_map, box):
        x1, y1, x2, y2 = map(int, box)
        h, w = depth_map.shape
        bw, bh = x2 - x1, y2 - y1

        rx1 = max(0, int(x1 + bw * 0.35))
        rx2 = min(w, int(x2 - bw * 0.35))
        ry1 = max(0, int(y1 + bh * 0.25))
        ry2 = min(h, int(y2 - bh * 0.25))

        roi = depth_map[ry1:ry2, rx1:rx2]
        valid = roi[roi > 0]

        if len(valid) > 10:
            return float(np.percentile(valid, 30)) / 1000.0
        return 0.0

    def check_physical_width(self, box, depth_m):
        if depth_m == 0.0:
            return False, 0.0
        pixel_w = box[2] - box[0]
        fx = 600.0
        real_width = (pixel_w * depth_m) / fx
        return (0.15 < real_width < 1.2), real_width

    def get_predictions(self, img):
        """
        Refactored: Pure single-model inference matching the paper's methodology.
        Extremely fast, low overhead.
        """
        res_custom = self.model_custom(img, imgsz=640, conf=0.40, classes=[0], verbose=False)[0]

        final_boxes = []
        if res_custom.boxes is not None:
            for box in res_custom.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                final_boxes.append([int(x1), int(y1), int(x2), int(y2)])

        return final_boxes

    def publish_target(self, is_locked):
        msg = Point()
        if is_locked:
            msg.x = float(self.owner_depth)
            msg.y = float(self.owner_pos[0])
            msg.z = 1.0
        else:
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
        self.pub_target.publish(msg)

    def sync_callback(self, depth_msg, ir_msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
            cv_ir = self.bridge.imgmsg_to_cv2(
                ir_msg, desired_encoding='mono8')

            self.frame_count += 1
            curr_time = time.time()
            dt = curr_time - self.last_time
            self.last_time = curr_time
            img_h, img_w = cv_ir.shape

            norm_ir = cv2.normalize(cv_ir, None, 0, 255, cv2.NORM_MINMAX)
            enhanced_ir = self.clahe.apply(norm_ir)
            cv_img = cv2.cvtColor(enhanced_ir, cv2.COLOR_GRAY2BGR)

            # Replaced ensemble call with single model prediction
            raw_boxes = self.get_predictions(cv_img)

            candidates = []
            for box in raw_boxes:
                depth_m = self.extract_robust_depth(cv_depth, box)
                is_valid, real_w = self.check_physical_width(box, depth_m)
                if is_valid and depth_m > 0:
                    cx = (box[0] + box[2]) / 2.0
                    cy = (box[1] + box[3]) / 2.0
                    candidates.append({
                        'box': box,
                        'pos': np.array([cx, cy]),
                        'depth': depth_m,
                        'width': real_w
                    })

            display_img = cv_img.copy()

            if not self.locked:
                self.publish_target(False)

                if self.frame_count > 30 and candidates:
                    img_center = np.array([img_w / 2.0, img_h / 2.0])
                    best_cand = min(
                        candidates,
                        key=lambda c: np.linalg.norm(c['pos'] - img_center))

                    if (np.linalg.norm(best_cand['pos'] - img_center) < 200
                            and best_cand['depth'] < 3.0):
                        self.locked = True
                        self.owner_pos = best_cand['pos']
                        self.owner_depth = best_cand['depth']
                        self.owner_box = best_cand['box']
                        self.owner_velocity = np.array([0.0, 0.0])
                        self.get_logger().info(
                            f"TARGET LOCKED at {self.owner_depth:.2f}m")

            else:
                if candidates:
                    predicted_pos = self.owner_pos + self.owner_velocity * dt
                    best_match = None
                    min_cost = float('inf')

                    for cand in candidates:
                        dist_2d = np.linalg.norm(
                            cand['pos'] - predicted_pos)
                        depth_diff = abs(
                            cand['depth'] - self.owner_depth)
                        depth_drop = self.owner_depth - cand['depth']
                        iou = self.get_iou(
                            cand['box'], self.owner_box)

                        if depth_drop > 0.25:
                            continue
                        if depth_diff > 0.35:
                            continue

                        max_dist_2d = 250 if iou > 0.1 else 120
                        if dist_2d > max_dist_2d:
                            continue

                        cost = dist_2d + (depth_diff * 1000) - (iou * 200)

                        if cost < min_cost:
                            min_cost = cost
                            best_match = cand

                    if best_match:
                        self.lost_frames = 0

                        if dt > 0:
                            new_vel = (
                                best_match['pos'] - self.owner_pos
                            ) / max(dt, 0.001)

                            speed = np.linalg.norm(new_vel)
                            if speed > 1000:
                                new_vel = (new_vel / speed) * 1000

                            self.owner_velocity = (
                                self.owner_velocity * 0.5 +
                                new_vel * 0.5)

                        self.owner_pos = best_match['pos']
                        self.owner_depth = (
                            self.owner_depth * 0.8 +
                            best_match['depth'] * 0.2)
                        self.owner_box = best_match['box']

                        self.publish_target(True)

                        bx1, by1, bx2, by2 = map(
                            int, best_match['box'])

                        cv2.rectangle(
                            display_img,
                            (bx1, by1),
                            (bx2, by2),
                            (0, 0, 255), 3)

                        cv2.putText(
                            display_img,
                            f"OWNER Z:{self.owner_depth:.2f}m",
                            (bx1, by1 - 10),
                            self.font,
                            0.6,
                            (0, 0, 255), 2)

                    else:
                        self.lost_frames += 1
                        self.publish_target(False)

                else:
                    self.lost_frames += 1
                    self.publish_target(False)

                if self.lost_frames > 0:
                    self.owner_velocity *= 0.8
                    self.owner_pos += self.owner_velocity * dt

                    cv2.circle(
                        display_img,
                        (int(self.owner_pos[0]),
                         int(self.owner_pos[1])),
                        15,
                        (0, 255, 255), 2)

                    if self.lost_frames > self.MAX_LOST_FRAMES:
                        self.locked = False
                        self.frame_count = 0
                        self.publish_target(False)
                        self.get_logger().warn("TARGET LOST")

            cv2.putText(
                display_img,
                "TRACKING: " +
                ("LOCKED" if self.locked else "SEARCHING"),
                (10, 30),
                self.font,
                0.6,
                (0, 0, 255) if self.locked else (0, 165, 255),
                2)

            self.pub_annotated.publish(
                self.bridge.cv2_to_imgmsg(
                    display_img, encoding="bgr8"))

        except Exception as e:
            import traceback
            self.get_logger().error(
                f"Error: {e}\n{traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_target(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
