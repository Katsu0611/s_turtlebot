#!/usr/bin/env python3
"""
ロボット側 Launchファイル
USBカメラの起動と映像配信のみを行う
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # === Launch引数 ===
    
    # カメラデバイス
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2',
        description='ビデオデバイス番号 (例: /dev/video0, /dev/video2)'
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
            'pixel_format': 'mjpeg2rgb',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap',
            'framerate': 30.0,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        # 引数
        video_device_arg,
        
        # ノード
        usb_cam_node,
    ])