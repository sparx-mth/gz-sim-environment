from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_ids = [0, 1]  # Add more robot IDs here, e.g., [0, 1, 2]

    nodes = []
    for robot_id in robot_ids:
        nodes.append(
            Node(
                package='color_detector',  # 🛠️ Replace with your actual package name
                executable=f'color_finder_node_{robot_id}',  # if using .py directly
                name=f'color_finder_node_{robot_id}',
                parameters=[
                    {'camera_topic': f'/robot_{robot_id}/rgb_camera'},
                    {'color': 'blue' if robot_id == 0 else 'red'}  # Optional: set per-robot color
                ],
                output='screen'
            )
        )

    return LaunchDescription(nodes)
