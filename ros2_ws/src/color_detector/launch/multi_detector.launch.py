from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='color_detector',
            executable='color_detector_node',
            name='detector_robot0',
            parameters=[
                {'camera_topic': '/robot_0/rgb_camera'},
                {'color': 'red'}
            ]
        ),
        Node(
            package='color_detector',
            executable='color_detector_node',
            name='detector_robot1',
            parameters=[
                {'camera_topic': '/robot_1/rgb_camera'},
                {'color': 'blue'}
            ]
        )
    ])