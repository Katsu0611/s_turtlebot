from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # カメラデバイスのパス（基本は /dev/video0 ですが、認識しない場合は video2 等に変更してください）
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Path to video device'
    )

    return LaunchDescription([
        video_device_arg,

        # ==========================================
        # 1. USB Camera Node (カメラを起動する)
        # ==========================================
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': 640,   # カメラ自体の解像度
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'io_method': 'mmap',
            }],
            # トピック名を /camera/image_raw に変更して送信
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
            output='screen'
        ),

        # ==========================================
        # 2. Hybrid Crowd Flow Node (画像を受け取って処理する)
        # ==========================================
        Node(
            package='animove',
            executable='crowd_flow_node.py',
            name='hybrid_crowd_flow_detector',
            parameters=[{
                # ★ 受け取るトピック名 (カメラの設定に合わせる)
                'camera_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'cmd_vel_topic': '/cmd_vel',
                
                # 軽量化設定 (処理用にリサイズ)
                'target_width': 320,
                
                # YOLO設定
                'use_yolo': True,
                'yolo_model': 'yolov8n.pt',
                'yolo_confidence': 0.4,
                'yolo_interval': 5,
                'yolo_iou': 0.5,
                'yolo_max_det': 10,
                
                # YOLO検出フィルタ
                'min_bbox_area': 800,
                'max_bbox_area': 400000,
                'min_aspect_ratio': 0.5,
                'max_aspect_ratio': 6.0,
                
                # Optical Flow設定
                'flow_pyr_scale': 0.5,
                'flow_levels': 1,
                'flow_winsize': 15,
                'flow_iterations': 1,
                
                # 群衆検知設定
                'grid_size': 20,
                'flow_threshold': 1.5,
                'motion_threshold': 1.0,
                
                # 追跡設定
                'iou_threshold': 0.15,
                'max_lost_frames': 40,
                'duplicate_iou_threshold': 0.6,
                'out_of_frame_threshold': 0.15,
                'person_threshold': 1,
                
                # エゴモーション補償
                'enable_ego_compensation': True,
                
                # 可視化設定
                'enable_visualization': True,
            }],
            output='screen'
        ),
    ])