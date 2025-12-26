import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- パラメータのLaunch Argument宣言 ---
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Input camera image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Input camera info topic'
    )

    # ★★★ ros2 topic list の結果に基づき、デフォルト値を修正 ★★★
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/diff_cont/cmd_vel_unstamped', 
        description='Input cmd_vel topic'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='True',
        description='Enable visualization window'
    )
    
    target_width_arg = DeclareLaunchArgument(
        'target_width',
        default_value='640',
        description='Width for flow calculation'
    )

    enable_ego_compensation_arg = DeclareLaunchArgument(
        'enable_ego_compensation',
        default_value='True',
        description='Enable ego-motion compensation'
    )

    
    # --- ノードの定義 ---
    
    optical_flow_node = Node(
        package='animove',
        executable='optical_flow_node.py', # 'lib' ディレクトリにインストールされた実行ファイルを指定
        name='optical_flow_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            # Launch Argumentの値をノードのパラメータとして渡す
            {'camera_topic': LaunchConfiguration('camera_topic')},
            {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
            {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')},
            {'enable_visualization': LaunchConfiguration('enable_visualization')},
            {'target_width': LaunchConfiguration('target_width')},
            {'enable_ego_compensation': LaunchConfiguration('enable_ego_compensation')},
        ]
    )
    
    return LaunchDescription([
        # Launch ArgumentをLaunchDescriptionに追加
        camera_topic_arg,
        camera_info_topic_arg,
        cmd_vel_topic_arg,
        enable_visualization_arg,
        target_width_arg,
        enable_ego_compensation_arg,
        
        # ノードをLaunchDescriptionに追加
        optical_flow_node
    ])