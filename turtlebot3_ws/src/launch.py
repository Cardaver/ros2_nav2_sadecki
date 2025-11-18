from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wielokaty',
            executable='wiel',
            name='wiel',
            parameters=[{'use_sim_time':True}]
        ),
    ])