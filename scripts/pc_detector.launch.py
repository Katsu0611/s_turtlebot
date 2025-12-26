#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# from pathlib import Path  <-- 不要なので削除

# find_script() 関数は削除しました（Nodeアクションが自動で探してくれるため）

def generate_launch_description():
    
    # script_path = find_script() <-- 不要なので削除
    
    # === Launch引数 ===
    # デフォルトを /cmd_vel_joy に設定（Good!）
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_joy')
    
    # 解像度やアルゴリズム設定
    target_width_arg = DeclareLaunchArgument('target_width', default_value='320')
    enable_ego_compensation_arg = DeclareLaunchArgument('enable_ego_compensation', default_value='true')
    use_yolo_arg = DeclareLaunchArgument('use_yolo', default_value='true')
    yolo_model_arg = DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt')
    yolo_confidence_arg = DeclareLaunchArgument('yolo_confidence', default_value='0.5')
    person_threshold_arg = DeclareLaunchArgument('person_threshold', default_value='2')
    grid_size_arg = DeclareLaunchArgument('grid_size', default_value='30')
    flow_threshold_arg = DeclareLaunchArgument('flow_threshold', default_value='1.0')
    enable_visualization_arg = DeclareLaunchArgument('enable_visualization', default_value='true')
    flow_winsize_arg = DeclareLaunchArgument('flow_winsize', default_value='10')
    flow_iterations_arg = DeclareLaunchArgument('flow_iterations', default_value='2')
    
    # === 群衆フロー検知ノード ===
    crowd_flow_node = Node(
        package='animove',
        executable='crowd_flow_node.py', # setup.py で scripts に入れているならこれでOK
        name='crowd_flow_detector',
        output='screen',
        parameters=[{
            # ★重要: 実機のカメラトピック名に合わせてください
            # 一般的には '/camera/image_raw/compressed' などの場合もありますが、
            # 実機で ros2 topic list をして '/image_raw/compressed' があるならこれで正解です。
            'camera_topic': '/image_raw/compressed', 
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
        # 引数登録
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
        
        # ノード起動
        crowd_flow_node,
    ])