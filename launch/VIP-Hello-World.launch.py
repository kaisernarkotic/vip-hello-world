from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='VIP-Hello-World', executable='zed_camera_node', output='screen'),
        Node(package='VIP-Hello-World', executable='processing_node', output='screen'),
        Node(package='VIP-Hello-World', executable='control_node', output='screen'),
    ])
