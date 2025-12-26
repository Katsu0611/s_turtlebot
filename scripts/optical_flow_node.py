#!/usr/bin/env python3
"""
オプティカルフロー計算ノード (エゴモーション補償＆ベクトル可視化)

- .xacroファイルから算出した正確な物理パラメータで補償
- フローを矢印ベクトルで可視化
- BGR2GRAY のタイプミスを修正済み
- compute_ego_flow の並進フロー計算ロジックを物理的に正確なモデルに修正
- EgoMotionCompensator の初回初期化エラー (self.X is None) を修正
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from dataclasses import dataclass, replace

@dataclass
class CameraParameters:
    """カメラの内部パラメータ (camera_info から取得)"""
    fx: float = 525.0
    fy: float = 525.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    
    # --- .xacro から算出したファインチューニング済みの値 ---
    camera_height: float = 0.185  # [m] (床からレンズまでの高さ)
    camera_tilt: float = 0.1745   # [rad] (約10度のお辞儀)


class EgoMotionCompensator:
    """エゴモーション補償クラス"""
    
    def __init__(self, camera_params: CameraParameters):
        # self.X が None の状態で update_camera_parameters を呼ぶため、
        # 最初のメッシュ計算が必ず実行される
        self.cam = camera_params 
        self.X = None
        self.Y = None
        self.X_norm = None
        self.Y_norm = None
        self.update_camera_parameters(camera_params)
        
    def update_camera_parameters(self, camera_params: CameraParameters):
        """カメラパラメータが更新された場合（解像度変更など）にメッシュを再計算"""

        # ★★★ 修正点 ★★★
        # (self.X is not None) を追加。
        # self.X が None (初回起動時) の場合は、解像度が同じでも必ずメッシュ計算を実行する
        if (self.cam.width == camera_params.width and 
            self.cam.height == camera_params.height and 
            self.X is not None): # <--- self.X の None チェックを追加
            return
            
        self.cam = camera_params
        x_coords = np.arange(self.cam.width)
        y_coords = np.arange(self.cam.height)
        self.X, self.Y = np.meshgrid(x_coords, y_coords)
        # 正規化座標 (Normalized Coordinates)
        self.X_norm = (self.X - self.cam.cx) / self.cam.fx
        self.Y_norm = (self.Y - self.cam.cy) / self.cam.fy
        
    def compute_ego_flow(self, cmd_vel: Twist, dt: float) -> np.ndarray:
        """
        cmd_velから画像平面上の期待フロー（エゴフロー）を計算
        [修正版: 奥行き(Z)をピクセルごとに計算し、正しい並進フローモデルを使用]
        """
        vx = cmd_vel.linear.x
        vy = cmd_vel.linear.y # (注: vyによる横移動の並進フローは未実装)
        omega = cmd_vel.angular.z

        # --- 1. 並進フロー (主に vx: 前進 から) ---
        flow_translation = np.zeros((self.cam.height, self.cam.width, 2), dtype=np.float32)

        if abs(vx) > 0.01:
            # (a) 平坦な床を仮定し、ピクセルごとの前方奥行き Z を計算
            
            # self.Y_norm = (self.Y - self.cam.cy) / self.cam.fy
            # 各ピクセルのY座標 (Y_norm) から、カメラ中心軸からの角度(alpha_y)を計算
            alpha_y = np.arctan(self.Y_norm)
            
            # カメラの傾き(camera_tilt)とalpha_yを足して、地面への視線角度(total_tilt)を計算
            total_tilt = self.cam.camera_tilt + alpha_y
            
            # Z = 高さ H / tan(角度) により、前方奥行き Z を計算
            # (total_tilt が 0 やマイナスになる地平線より上は除く)
            Z = np.full_like(total_tilt, 1000.0) # デフォルトは無限遠(1000m)
            valid_pixels = total_tilt > 0.01 # 地平線より下
            
            Z[valid_pixels] = self.cam.camera_height / np.tan(total_tilt[valid_pixels])
            
            # 非常に近い値や遠い値をクリップ
            Z = np.clip(Z, 0.1, 1000.0)

            # (b) 放射状のフローを計算
            # u_flow = (u - cx) * vx * dt / Z
            # v_flow = (v - cy) * vx * dt / Z
            # (self.X - self.cam.cx) は (u - cx) に相当
            
            flow_translation[..., 0] = (self.X - self.cam.cx) * vx * dt / Z
            flow_translation[..., 1] = (self.Y - self.cam.cy) * vx * dt / Z
        
        # (注: ロボットの横移動 vy による並進フロー u = -fx * vy * dt / Z はここで追加可能)
        

        # --- 2. 回転フロー (omega: 旋回 から) ---
        flow_rotation = np.zeros_like(flow_translation)
        
        if abs(omega) > 0.01:
            # この計算は深度(Z)に依存しないため、以前のままで正しい
            dx = self.X - self.cam.cx
            dy = self.Y - self.cam.cy
            
            # (u_rot) = -omega * dt * dy
            # (v_rot) =  omega * dt * dx
            flow_rotation[..., 0] = -omega * dt * dy
            flow_rotation[..., 1] =  omega * dt * dx
        
        # --- 3. 合成 ---
        ego_flow = flow_translation + flow_rotation
        return ego_flow
    
    def compensate_flow(self, observed_flow: np.ndarray, cmd_vel: Twist, dt: float) -> np.ndarray:
        """観測フローからエゴモーションを差し引く"""
        ego_flow = self.compute_ego_flow(cmd_vel, dt)
        
        # フローの解像度が一致しているか最終確認
        if observed_flow.shape != ego_flow.shape:
             h, w = observed_flow.shape[:2]
             ego_flow = cv2.resize(ego_flow, (w, h), interpolation=cv2.INTER_LINEAR)

        compensated_flow = observed_flow - ego_flow
        return compensated_flow


def draw_flow_vectors(frame, flow, grid_size=16, magnitude_scale=1.5):
    """
    元画像 (frame) に、指定したグリッド間隔 (grid_size) で
    フローベクトル (flow) を矢印として重ねて描画する
    """
    h, w = frame.shape[:2]
    
    # グリッドの中心から描画を開始するためのオフセット
    y_offset, x_offset = grid_size // 2, grid_size // 2
    
    for y in range(y_offset, h, grid_size):
        for x in range(x_offset, w, grid_size):
            # (y, x) の位置のフローベクトル (fx, fy) を取得
            fx, fy = flow[y, x]
            
            # フローの大きさをスケーリング
            fx *= magnitude_scale
            fy *= magnitude_scale

            # フローが非常に小さい場合 (ノイズなど) は描画をスキップ
            if np.linalg.norm([fx, fy]) < 0.5:
                continue
                
            pt1 = (x, y) # 始点 (グリッドの中心)
            pt2 = (int(x + fx), int(y + fy)) # 終点 (フローの先)
            
            # 矢印を描画 (緑色, 太さ1, 矢じりのサイズ 0.3)
            cv2.arrowedLine(frame, pt1, pt2, (0, 255, 0), 1, tipLength=0.3)
    
    # frame は参照渡しなので、戻り値は不要だが、分かりやすさのため返す
    return frame


class OpticalFlowNode(Node):
    """
    オプティカルフロー計算ノード (エゴモーション補償付き)
    """
    
    def __init__(self):
        super().__init__('optical_flow_node')
        
        # パラメータ宣言
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info') 
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_joy')
        self.declare_parameter('target_width', 640)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('enable_ego_compensation', True)
        
        # パラメータ取得
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.target_width = self.get_parameter('target_width').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.enable_ego_compensation = self.get_parameter('enable_ego_compensation').value
        
        # QoS設定
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # サブスクライバー
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, qos
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )
        
        # 変数初期化
        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_time = None # dt計算用
        
        self.current_cmd_vel = Twist()
        # オリジナルのカメラパラメータを保持 (xacroの値で初期化)
        self.original_camera_params = CameraParameters()
        self.camera_info_received = False
        
        # エゴモーション補償器 (xacroの値で初期化)
        self.ego_compensator = EgoMotionCompensator(self.original_camera_params)
        # フロー計算用のスケーリングされたカメラパラメータ
        self.scaled_camera_params = replace(self.original_camera_params) # deep copy

        # 統計用
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        
        self.get_logger().info('=== オプティカルフローノード起動 (エゴモーション補償 / ベクトル可視化) ===')
        self.get_logger().info(f'カメラトピック: {camera_topic}')
        self.get_logger().info(f'エゴモーション補償: {"有効" if self.enable_ego_compensation else "無効"}')
        self.get_logger().info(f'物理パラメータ: Height={self.original_camera_params.camera_height:.3f}m, Tilt={self.original_camera_params.camera_tilt:.3f}rad')

    
    def camera_info_callback(self, msg: CameraInfo):
        """オリジナルのカメラ情報を取得・保存"""
        # camera_info から読み取った値で、xacroの物理値以外を上書きする
        if not self.camera_info_received:
            self.original_camera_params.fx = msg.k[0]
            self.original_camera_params.fy = msg.k[4]
            self.original_camera_params.cx = msg.k[2]
            self.original_camera_params.cy = msg.k[5]
            self.original_camera_params.width = msg.width
            self.original_camera_params.height = msg.height
            
            self.camera_info_received = True
            self.get_logger().info(f'カメラ内部パラメータ取得: {msg.width}x{msg.height}, fx={msg.k[0]:.1f}')
    
    def cmd_vel_callback(self, msg: Twist):
        """速度指令を保存"""
        self.current_cmd_vel = msg
    
    
    def image_callback(self, msg: Image):
        """メイン処理: フロー計算 → (エゴモーション補償) → 可視化"""
        
        current_time = self.get_clock().now()
        
        try:
            current_frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # --- 1. 解像度スケーリング ---
            if not self.camera_info_received:
                self.get_logger().warn('CameraInfo 未受信のため待機中...', throttle_duration_sec=5.0)
                return

            original_h, original_w = current_frame_bgr.shape[:2]
            scale_factor = original_w / self.target_width
            target_h = int(original_h / scale_factor)
            
            # フロー計算用のリサイズ画像
            curr_small_bgr = cv2.resize(current_frame_bgr, (self.target_width, target_h))
            
            # ★★★ BGR -> GRAY 修正済み ★★★
            curr_gray = cv2.cvtColor(curr_small_bgr, cv2.COLOR_BGR2GRAY)
            
            # --- 2. EgoCompensator のパラメータ更新 (解像度が変わった場合) ---
            if self.scaled_camera_params.width != self.target_width or \
               self.scaled_camera_params.height != target_h:
                
                self.get_logger().info(f'フロー計算解像度を {self.target_width}x{target_h} に設定中...')
                
                # dataclass.replace を使ってスケール済みの新しいパラメータセットを作成
                self.scaled_camera_params = replace(
                    self.original_camera_params,
                    width = self.target_width,
                    height = target_h,
                    fx = self.original_camera_params.fx / scale_factor,
                    fy = self.original_camera_params.fy / scale_factor,
                    cx = self.original_camera_params.cx / scale_factor,
                    cy = self.original_camera_params.cy / scale_factor
                )
                
                # 補償器の内部メッシュを新しい解像度で更新
                self.ego_compensator.update_camera_parameters(self.scaled_camera_params)
                self.get_logger().info('EgoCompensator を更新しました。')

            
            # --- 3. フロー計算 ---
            if self.prev_gray is None:
                self.prev_gray = curr_gray
                self.prev_time = current_time
                self.start_time = time.time()
                return
            
            # dt (前フレームからの経過時間)
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt < 0.001: dt = 0.033 # 異常値防止

            # (a) 純粋なオプティカルフロー
            observed_flow = cv2.calcOpticalFlowFarneback(
                self.prev_gray, curr_gray, None, 
                pyr_scale=0.5, levels=3, winsize=20,
                iterations=3, poly_n=7, poly_sigma=1.5, flags=0
            )
            
            # (b) エゴモーション補償
            flow_to_visualize = None
            
            if self.enable_ego_compensation:
                compensated_flow = self.ego_compensator.compensate_flow(
                    observed_flow, self.current_cmd_vel, dt
                )
                flow_to_visualize = compensated_flow
            else:
                flow_to_visualize = observed_flow
            
            
            # --- 4. 画面表示 ---
            if self.enable_visualization:
                self.fps = self.update_fps()
                
                # 元画像（リサイズ版）のコピーを作成
                vis_frame = curr_small_bgr.copy()
                
                # ベクトルを重ね書き
                draw_flow_vectors(vis_frame, flow_to_visualize, grid_size=16)
                
                # --- 情報描画 ---
                cv2.putText(vis_frame, f'FPS: {self.fps:.1f}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                v = self.current_cmd_vel.linear.x
                w = self.current_cmd_vel.angular.z
                cv2.putText(vis_frame, f'v={v:.2f} w={w:.2f}', (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                # エゴモーション補償ステータス
                status = "EGO-COMP: ON" if self.enable_ego_compensation else "EGO-COMP: OFF"
                color = (0, 255, 255) if self.enable_ego_compensation else (128, 128, 128)
                cv2.putText(vis_frame, status, (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                # ウィンドウ表示
                cv2.imshow('Optical Flow (Vectors)', vis_frame)
                cv2.waitKey(1)
            
            # 履歴を更新
            self.prev_gray = curr_gray
            self.prev_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'処理エラー: {e}')
            import traceback
            traceback.print_exc() # 詳細なエラー内容をターミナルに出力

    def update_fps(self):
        """FPS計算"""
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
    node = OpticalFlowNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('キーボード割り込みで終了')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()