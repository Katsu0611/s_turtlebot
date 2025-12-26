#!/usr/bin/env python3
"""
群衆フロー検知 Launchファイル（完全版）
setup.py不要、スクリプト直接実行方式
パス自動検出機能付き
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
from pathlib import Path

def find_script():
    """crowd_flow_node.pyを複数の場所から探す"""
    # 可能性のあるパスのリスト
    possible_paths = [
        # ソースディレクトリから
        Path.home() / 'animove_ws' / 'src' / 'animove' / 'scripts' / 'crowd_flow_node.py',
        # インストールディレクトリから
        Path.home() / 'animove_ws' / 'install' / 'animove' / 'share' / 'animove' / 'scripts' / 'crowd_flow_node.py',
        # launchファイルの相対位置から
        Path(__file__).parent.parent / 'scripts' / 'crowd_flow_node.py',
    ]
    
    for path in possible_paths:
        if path.exists():
            print(f"Found script at: {path}")
            return str(path)
    
    # 見つからない場合はエラーメッセージと最初のパスを返す
    print(f"ERROR: Script not found! Searched in:")
    for path in possible_paths:
        print(f"  - {path}")
    return str(possible_paths[0])  # デフォルトとして最初のパスを返す

def generate_launch_description():
    
    # スクリプトパスを自動検出
    script_path = find_script()
    
    # === Launch引数 ===
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2',
        description='ビデオデバイス番号'
    )
    
    target_width_arg = DeclareLaunchArgument(
        'target_width',
        default_value='320',
        description='処理する画像の幅'
    )
    
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo',
        default_value='true',
        description='YOLOv8の有効化'
    )
    
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='YOLOモデル'
    )
    
    yolo_confidence_arg = DeclareLaunchArgument(
        'yolo_confidence',
        default_value='0.5',
        description='YOLO信頼度閾値'
    )
    
    person_threshold_arg = DeclareLaunchArgument(
        'person_threshold',
        default_value='1',
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
    
    enable_ego_compensation_arg = DeclareLaunchArgument(
        'enable_ego_compensation',
        default_value='false',
        description='エゴモーション補償（移動ロボット用）'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='可視化'
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
    usb_cam_node = ExecuteProcess(
        cmd=['ros2', 'run', 'usb_cam', 'usb_cam_node_exe',
             '--ros-args',
             '-p', ['video_device:=', LaunchConfiguration('video_device')],
             '-p', 'image_width:=640',
             '-p', 'image_height:=480',
             '-p', 'pixel_format:=yuyv',
             '-p', 'camera_frame_id:=usb_cam',
             '-p', 'io_method:=mmap',
             '-p', 'framerate:=30.0',
             '-r', '/usb_cam/image_raw:=/camera/image_raw',
             '-r', '/usb_cam/camera_info:=/camera/camera_info',
        ],
        output='screen',
        shell=False
    )
    
    # === ダミーcmd_vel ===
    static_cmd_vel = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/cmd_vel', 
             'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}',
             '--rate', '10'],
        shell=False,
        output='log'
    )
    
    # === 群流検知ノード（Pythonスクリプト直接実行）===
    crowd_flow_node = ExecuteProcess(
        cmd=['python3', script_path,
             '--ros-args',
             '-p', 'camera_topic:=/camera/image_raw',
             '-p', 'camera_info_topic:=/camera/camera_info',
             '-p', 'cmd_vel_topic:=/cmd_vel',
             '-p', ['target_width:=', LaunchConfiguration('target_width')],
             '-p', ['use_yolo:=', LaunchConfiguration('use_yolo')],
             '-p', ['yolo_model:=', LaunchConfiguration('yolo_model')],
             '-p', ['yolo_confidence:=', LaunchConfiguration('yolo_confidence')],
             '-p', ['person_threshold:=', LaunchConfiguration('person_threshold')],
             '-p', ['grid_size:=', LaunchConfiguration('grid_size')],
             '-p', ['flow_threshold:=', LaunchConfiguration('flow_threshold')],
             '-p', ['enable_ego_compensation:=', LaunchConfiguration('enable_ego_compensation')],
             '-p', ['enable_visualization:=', LaunchConfiguration('enable_visualization')],
             '-p', ['flow_winsize:=', LaunchConfiguration('flow_winsize')],
             '-p', ['flow_iterations:=', LaunchConfiguration('flow_iterations')],
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        # 引数
        video_device_arg,
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
        
        # プロセス
        usb_cam_node,
        static_cmd_vel,
        crowd_flow_node,
    ])