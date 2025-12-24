from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
<<<<<<< HEAD
                'serial_port': '/dev/ttyUSB1',
=======
                'serial_port': '/dev/ttyUSB0',
>>>>>>> acfb3d4 (Fixed Bugs)
                'serial_baudrate': 115200, 
                'frame_id': 'laser_frame',
                'scan_mode': 'Standard',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
