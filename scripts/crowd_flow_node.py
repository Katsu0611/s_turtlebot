#!/usr/bin/env python3
"""
ROS2 群衆フロー検知ノード（v16: カメラ映像のみ・大画面版）

特徴:
1. オドメトリを使わず、カメラ映像とcmd_velのみで動作します。
2. 画面は2.5倍拡大＋高画質ログエリア表示（v14の機能継承）。
3. 実機/シミュレータの cmd_vel (または cmd_vel_joy) を使用可能。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from dataclasses import dataclass, replace
from collections import deque

# YOLOv8のインポート
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("警告: ultralytics がインストールされていません。")


@dataclass
class CameraParameters:
    """カメラの内部パラメータ"""
    fx: float = 525.0
    fy: float = 525.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    camera_height: float = 0.185
    camera_tilt: float = 0.1745


class EgoMotionCompensator:
    """エゴモーション補償クラス"""
    def __init__(self, camera_params: CameraParameters):
        self.cam = camera_params
        self.X = None
        self.Y = None
        # ★カメラのみの場合、少し強めに補正をかけます
        self.compensation_factor = 1.5 
        self.velocity_scale = 1.3
        self.focal_scale = 1.0
        self.height_scale = 1.0
        self.tilt_offset = 0.0
        self._update_grid(self.cam.width, self.cam.height)

    def update_camera_parameters(self, cam_params: CameraParameters):
        is_resized = (self.cam.width != cam_params.width or self.cam.height != cam_params.height)
        self.cam = cam_params
        if is_resized or self.X is None:
            self._update_grid(self.cam.width, self.cam.height)

    def _update_grid(self, w: int, h: int):
        self.X, self.Y = np.meshgrid(np.arange(w), np.arange(h))
        self.X = self.X.astype(np.float32)
        self.Y = self.Y.astype(np.float32)

    def _calculate_expected_radial_flow(self, linear_x: float, dt: float) -> np.ndarray:
        Vz = linear_x * self.velocity_scale
        if abs(Vz) < 0.001 or dt <= 0:
            return np.zeros((self.cam.height, self.cam.width, 2), dtype=np.float32)
        fx = self.cam.fx * self.focal_scale
        fy = self.cam.fy * self.focal_scale
        Z_avg = max(0.1, 1.0 * self.height_scale)
        delta_Z = Vz * dt
        scale_ratio = delta_Z / (Z_avg - self.tilt_offset)
        X_norm = (self.X - self.cam.cx) / fx
        flow_x = fx * X_norm * scale_ratio
        Y_norm = (self.Y - self.cam.cy) / fy
        flow_y = fy * Y_norm * scale_ratio
        return np.stack([flow_x, flow_y], axis=-1).astype(np.float32)

    def _calculate_expected_rotational_flow(self, angular_z: float, dt: float) -> np.ndarray:
        if abs(angular_z) < 0.001 or dt <= 0:
            return np.zeros((self.cam.height, self.cam.width, 2), dtype=np.float32)
        theta = self.cam.camera_tilt + self.tilt_offset 
        Wz = angular_z
        Wy_cam = -Wz * np.sin(theta)
        Wz_cam = Wz * np.cos(theta)
        fx = self.cam.fx * self.focal_scale
        fy = self.cam.fy * self.focal_scale
        u_cx = self.X - self.cam.cx
        v_cy = self.Y - self.cam.cy
        u_cx_fx = u_cx / fx
        v_cy_fy = v_cy / fy
        flow_x_per_sec = fx * (Wz_cam * v_cy_fy - Wy_cam - Wy_cam * u_cx_fx**2)
        flow_y_per_sec = fy * (-Wz_cam * u_cx_fx - Wy_cam * u_cx_fx * v_cy_fy)
        flow_x = flow_x_per_sec * dt
        flow_y = flow_y_per_sec * dt
        return np.stack([flow_x, flow_y], axis=-1).astype(np.float32)

    def compensate_flow(self, flow: np.ndarray, linear_x: float, angular_z: float, dt: float) -> np.ndarray:
        expected_radial_flow = self._calculate_expected_radial_flow(linear_x, dt)
        expected_rotational_flow = self._calculate_expected_rotational_flow(angular_z, dt)
        expected_flow = expected_radial_flow - expected_rotational_flow
        residual_flow = flow - (expected_flow * self.compensation_factor)
        return residual_flow


class CrowdFlowDetector:
    """群衆フロー検知クラス"""
    def __init__(self, grid_size=40, flow_threshold=1.5):
        self.grid_size = grid_size
        self.flow_threshold = flow_threshold
        
    def detect_crowd_patterns(self, flow, person_mask=None):
        if person_mask is not None:
            masked_flow = flow.copy()
            masked_flow[~person_mask] = [0, 0]
        else:
            masked_flow = flow
        
        results = {
            'main_direction': self._detect_main_flow(masked_flow, person_mask),
            'congestion_areas': [],
            'counter_flows': [],
        }
        return results
    
    def _detect_main_flow(self, flow, person_mask=None):
        h, w = flow.shape[:2]
        regions = []
        for y in range(0, h - self.grid_size, self.grid_size):
            for x in range(0, w - self.grid_size, self.grid_size):
                if person_mask is not None:
                    if not person_mask[y + self.grid_size//2, x + self.grid_size//2]:
                        continue
                region_flow = flow[y:y+self.grid_size, x:x+self.grid_size]
                avg_flow = np.mean(region_flow, axis=(0, 1))
                magnitude = np.linalg.norm(avg_flow)
                angle = np.arctan2(avg_flow[1], avg_flow[0])
                direction = self._classify_direction(angle, magnitude)
                if direction != 'STATIC/NO_FLOW':
                    regions.append({
                        'bbox': (x, y, self.grid_size, self.grid_size),
                        'magnitude': magnitude,
                        'direction': direction,
                        'angle': angle
                    })
        return regions
    
    def _classify_direction(self, angle, magnitude):
        STOP_THRESHOLD = self.flow_threshold * 0.5
        if magnitude < STOP_THRESHOLD: return 'STATIC/NO_FLOW'
        angle_deg = np.degrees(angle)
        if -135 <= angle_deg < -45: return 'APPROACH'
        elif -45 <= angle_deg < 45: return 'RIGHT'
        elif 45 <= angle_deg < 135: return 'RECEDE'
        else: return 'LEFT'


class CrowdFlowNode(Node):
    """ROS2 群衆フロー検知ノード（v16: カメラ映像のみ・大画面版）"""
    
    WINDOW_NAME = 'Crowd Flow Detection'
    
    # ▼▼▼ 表示設定 ▼▼▼
    DISPLAY_SCALE = 2.5      # 画面の拡大倍率
    LOG_AREA_HEIGHT = 200    # ログ表示エリアの高さ
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲
    
    def __init__(self):
        super().__init__('crowd_flow_detector')
        
        # === パラメータ ===
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        # ★ cmd_vel に戻しました
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_width', 240)
        self.declare_parameter('angular_filter_alpha', 0.05)
        self.declare_parameter('motion_threshold', 0.5)
        self.declare_parameter('grid_size', 20)
        self.declare_parameter('flow_threshold', 1.5)
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_confidence', 0.5)
        self.declare_parameter('yolo_interval', 5)
        self.declare_parameter('enable_ego_compensation', True)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('person_threshold', 1)
        
        self.declare_parameter('flow_pyr_scale', 0.5)
        self.declare_parameter('flow_levels', 1)
        self.declare_parameter('flow_winsize', 15)
        self.declare_parameter('flow_iterations', 1)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.target_width = self.get_parameter('target_width').value
        self.angular_filter_alpha = self.get_parameter('angular_filter_alpha').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        self.grid_size = self.get_parameter('grid_size').value
        self.flow_threshold = self.get_parameter('flow_threshold').value
        self.use_yolo = self.get_parameter('use_yolo').value
        self.yolo_interval = self.get_parameter('yolo_interval').value
        self.enable_ego_compensation = self.get_parameter('enable_ego_compensation').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.person_threshold = self.get_parameter('person_threshold').value
        
        self.flow_params = {
            'pyr_scale': self.get_parameter('flow_pyr_scale').value,
            'levels': self.get_parameter('flow_levels').value,
            'winsize': self.get_parameter('flow_winsize').value,
            'iterations': self.get_parameter('flow_iterations').value,
            'poly_n': 5,
            'poly_sigma': 1.2,
            'flags': 0
        }

        self.camera_params = CameraParameters()
        self.compensator = EgoMotionCompensator(self.camera_params)
        self.crowd_detector = CrowdFlowDetector(self.grid_size, self.flow_threshold)
        
        self.yolo_model = None
        self.cached_person_mask = None
        self.cached_yolo_boxes = []
        self.cached_person_count = 0

        # === ログ履歴バッファ ===
        self.log_buffer = deque(maxlen=8)

        if self.use_yolo and YOLO_AVAILABLE:
            try:
                self.yolo_model = YOLO(self.get_parameter('yolo_model').value)
                self.yolo_confidence = self.get_parameter('yolo_confidence').value
                self.log('✓ YOLOv8 Loaded')
            except Exception as e:
                self.log(f'❌ YOLO Error: {e}')
                self.yolo_model = None
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        self.bridge = CvBridge()
        
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile)
        
        # ★ cmd_vel の購読に戻しました
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos_profile)
        
        if "compressed" in self.camera_topic:
            self.log(f"Mode: Compressed ({self.camera_topic})")
            self.create_subscription(CompressedImage, self.camera_topic, self.image_callback, qos_profile)
        else:
            self.log(f"Mode: Raw ({self.camera_topic})")
            self.create_subscription(Image, self.camera_topic, self.image_callback, qos_profile)

        self.prev_gray = None
        self.prev_time = None
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.compensated_angular_z = 0.0
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        
        if self.enable_visualization:
            cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(self.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        self.log('=== Node Started (v16: Camera Only) ===')

    def log(self, message: str):
        self.get_logger().info(message)
        self.log_buffer.append(message)

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_params.width == msg.width: return
        scale = self.target_width / msg.width
        self.camera_params = replace(self.camera_params,
            fx=msg.k[0]*scale, fy=msg.k[4]*scale, cx=msg.k[2]*scale, cy=msg.k[5]*scale,
            width=int(msg.width*scale), height=int(msg.height*scale)
        )
        self.compensator.update_camera_parameters(self.camera_params)

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def image_callback(self, msg):
        current_time = self.get_clock().now()
        try:
            if hasattr(msg, 'format') and "jpeg" in msg.format:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            h, w = cv_image.shape[:2]
            if w != self.camera_params.width:
                scale = self.camera_params.width / w
                target_h = int(h * scale)
                if abs(target_h - self.camera_params.height) > 1:
                    self.camera_params.height = target_h
                    self.compensator.update_camera_parameters(self.camera_params)
                cv_image = cv2.resize(cv_image, (self.camera_params.width, self.camera_params.height))
            
            curr_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            if self.prev_gray is None:
                self.prev_gray = curr_gray
                self.prev_time = current_time
                return

            try:
                dt = (current_time.nanoseconds - self.prev_time.nanoseconds) / 1e9
                if dt <= 0.001: dt = 0.03
            except: dt = 0.03

            flow = cv2.calcOpticalFlowFarneback(self.prev_gray, curr_gray, None, **self.flow_params)
            
            if self.enable_ego_compensation:
                alpha = self.angular_filter_alpha
                self.compensated_angular_z = (self.compensated_angular_z * (1.0 - alpha) + self.angular_z * alpha)
                residual_flow = self.compensator.compensate_flow(flow, self.linear_x, self.compensated_angular_z, dt)
            else:
                residual_flow = flow
                self.compensated_angular_z = 0.0
            
            mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            residual_flow[mag < self.motion_threshold] = [0, 0]
            
            if self.use_yolo and self.yolo_model is not None:
                if (self.frame_count % self.yolo_interval == 0) or (self.cached_person_mask is None):
                    try:
                        results = self.yolo_model(cv_image, verbose=False, conf=self.yolo_confidence)
                        mask = np.zeros((cv_image.shape[0], cv_image.shape[1]), dtype=bool)
                        boxes = []
                        p_count = 0
                        for box in results[0].boxes:
                            if int(box.cls) == 0:
                                x1, y1, x2, y2 = map(int, box.xyxy[0])
                                mask[y1:y2, x1:x2] = True
                                boxes.append((x1, y1, x2, y2, float(box.conf)))
                                p_count += 1
                        self.cached_person_mask = mask
                        self.cached_yolo_boxes = boxes
                        self.cached_person_count = p_count
                    except Exception as e:
                        self.log(f'YOLO Error: {e}')
            
            person_mask = self.cached_person_mask
            yolo_boxes = self.cached_yolo_boxes
            person_count = self.cached_person_count
            
            dominant_direction_label = 'STATIC/NO_FLOW'
            dominant_angle_hsv = None
            
            if person_count >= self.person_threshold or not self.use_yolo:
                self.crowd_detector.detect_crowd_patterns(residual_flow, person_mask)
                if person_mask is not None and person_count > 0:
                    p_flow_x = residual_flow[person_mask, 0]
                    p_flow_y = residual_flow[person_mask, 1]
                    if p_flow_x.size > 0:
                        avg_x, avg_y = np.mean(p_flow_x), np.mean(p_flow_y)
                        mag = np.linalg.norm([avg_x, avg_y])
                        if mag > self.flow_threshold * 0.5:
                            ang = np.arctan2(avg_y, avg_x)
                            dominant_direction_label = self.crowd_detector._classify_direction(ang, mag)
                            dominant_angle_hsv = (ang * 180 / np.pi / 2) % 180

            self.log(f'[{self.frame_count}] Person: {person_count} | {dominant_direction_label}')

            if self.enable_visualization:
                left_frame = cv_image.copy()
                for x1, y1, x2, y2, _ in yolo_boxes:
                    cv2.rectangle(left_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                right_frame = self._create_hsv_flow_visualization(residual_flow, person_mask, cv_image.shape[:2], dominant_angle_hsv)
                
                self.fps = self.update_fps()
                cv2.putText(left_frame, f'FPS: {self.fps:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(right_frame, 'Flow (HSV)', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # === 大画面化＆結合処理 (v16) ===
                # 1. 映像部分の結合（低解像度）
                top_small = np.hstack([left_frame, right_frame])
                
                # 2. 映像部分の拡大（DISPLAY_SCALE倍）
                top_large = cv2.resize(top_small, None, fx=self.DISPLAY_SCALE, fy=self.DISPLAY_SCALE, interpolation=cv2.INTER_LINEAR)
                
                # 3. ログエリアの作成（拡大後の幅に合わせる）
                w_large = top_large.shape[1]
                log_area = np.zeros((self.LOG_AREA_HEIGHT, w_large, 3), dtype=np.uint8)
                
                # 4. ログの書き込み（高解像度エリアに描画するので文字がくっきり）
                font_scale = 0.8 
                line_height = 28
                margin_left = 15
                margin_top = 35
                
                for i, log_text in enumerate(self.log_buffer):
                    y_pos = margin_top + (i * line_height)
                    cv2.putText(log_area, f"> {log_text}", (margin_left, y_pos), 
                                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2, cv2.LINE_AA)

                # 5. 最終結合
                final_frame = np.vstack([top_large, log_area])

                cv2.imshow(self.WINDOW_NAME, final_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                raise KeyboardInterrupt
            elif key == ord('f'):
                if cv2.getWindowProperty(self.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN) == cv2.WINDOW_FULLSCREEN:
                    cv2.setWindowProperty(self.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                else:
                    cv2.setWindowProperty(self.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            
            self.prev_gray = curr_gray
            self.prev_time = current_time
            self.frame_count += 1

        except KeyboardInterrupt: raise
        except Exception as e:
            self.log(f'Callback Error: {e}')

    def _create_hsv_flow_visualization(self, flow, person_mask, image_shape, dominant_angle_hsv=None):
        h, w = image_shape
        hsv = np.zeros((h, w, 3), dtype=np.uint8)
        hsv[..., 1] = 255
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        
        base_hue = ang * 180 / np.pi / 2
        if dominant_angle_hsv is not None:
            hue_offset = dominant_angle_hsv - 90 
            hsv[..., 0] = (base_hue - hue_offset) % 180
        else:
            hsv[..., 0] = base_hue
            
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        if person_mask is not None:
            masked_bgr = np.zeros_like(bgr)
            masked_bgr[person_mask] = bgr[person_mask]
            return masked_bgr
        return bgr

    def update_fps(self):
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        if elapsed > 1.0:
            fps = self.frame_count / elapsed
            self.frame_count = 0
            self.start_time = time.time()
            return fps
        return self.fps

def main(args=None):
    rclpy.init(args=args)
    node = CrowdFlowNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()