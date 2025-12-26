#!/usr/bin/env python3
"""
群衆フロー検知 Launchファイル（実機完全版）
USBカメラ起動 + エゴモーション補償対応
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

def find_script():
    """crowd_flow_node.pyを複数の場所から探す"""
    possible_paths = [
        Path.home() / 'animove_ws' / 'install' / 'animove' / 'lib' / 'animove' / 'crowd_flow_node.py',
        Path.home() / 'animove_ws' / 'src' / 'animove' / 'scripts' / 'crowd_flow_node.py',
    ]
    
    for path in possible_paths:
        if path.exists():
            print(f"[INFO] Found script at: {path}")
            return str(path)
    
    print(f"[ERROR] Script not found!")
    return str(possible_paths[0])

def generate_launch_description():
    
    script_path = find_script()
    
    # === Launch引数 ===
    
    # カメラデバイス
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2',
        description='ビデオデバイス番号 (例: /dev/video0, /dev/video2)'
    )
    
    # 速度コマンドトピック
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='速度コマンドトピック (例: /cmd_vel, /cmd_vel_joy)'
    )
    
    # 処理解像度
    target_width_arg = DeclareLaunchArgument(
        'target_width',
        default_value='320',
        description='処理する画像の幅 (低性能デバイスでは240や160推奨)'
    )
    
    # エゴモーション補償（実機では有効推奨）
    enable_ego_compensation_arg = DeclareLaunchArgument(
        'enable_ego_compensation',
        default_value='true',
        description='エゴモーション補償（移動ロボット用）'
    )
    
    # YOLO設定
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo',
        default_value='true',
        description='YOLOv8の有効化（低性能デバイスではfalse推奨）'
    )
    
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='YOLOモデル (n=nano/最軽量, s=small, m=medium)'
    )
    
    yolo_confidence_arg = DeclareLaunchArgument(
        'yolo_confidence',
        default_value='0.5',
        description='YOLO信頼度閾値'
    )
    
    person_threshold_arg = DeclareLaunchArgument(
        'person_threshold',
        default_value='3',
        description='検知開始人数'
    )
    
    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='30',
        description='グリッドサイズ'
    )
    
    flow_threshold_arg = DeclareLaunchArgument(
        'flow_threshold',
        default_value='1.5',
        description='フロー閾値'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='可視化（ディスプレイなしの場合はfalse）'
    )
    
    flow_winsize_arg = DeclareLaunchArgument(
        'flow_winsize',
        default_value='10',
        description='フローウィンドウサイズ'
    )
    
    flow_iterations_arg = DeclareLaunchArgument(
        'flow_iterations',
        default_value='2',
        description='フロー反復回数'
    )
    
    # === USBカメラノード ===
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap',
            'framerate': 30.0,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )
    
    # === 群流検知ノード ===
    crowd_flow_node = Node(
        package='animove',
        executable='crowd_flow_node.py',
        name='crowd_flow_detector',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'target_width': LaunchConfiguration('target_width'),
            'use_yolo': LaunchConfiguration('use_yolo'),
            'yolo_model': LaunchConfiguration('yolo_model'),
            'yolo_confidence': LaunchConfiguration('yolo_confidence'),
            'person_threshold': LaunchConfiguration('person_threshold'),
            'grid_size': LaunchConfiguration('grid_size'),
            'flow_threshold': LaunchConfiguration('flow_threshold'),
            'enable_ego_compensation': LaunchConfiguration('enable_ego_compensation'),
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'flow_winsize': LaunchConfiguration('flow_winsize'),
            'flow_iterations': LaunchConfiguration('flow_iterations'),
        }]
    )
    
    return LaunchDescription([
        # 引数
        video_device_arg,
        cmd_vel_topic_arg,
        target_width_arg,
        use_yolo_arg,
        yolo_model_arg,
        yolo_confidence_arg,
        person_threshold_arg,
        grid_size_arg,
        flow_threshold_arg,
        enable_ego_compensation_arg,
        enable_visualization_arg,
        flow_winsize_arg,
        flow_iterations_arg,
        
        # ノード
        usb_cam_node,
        crowd_flow_node,
    ])