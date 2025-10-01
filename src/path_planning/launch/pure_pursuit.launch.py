import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    path_planning_share_dir = get_package_share_directory('path_planning')

    # Define the path to the parameters file
    params_file = os.path.join(path_planning_share_dir, 'config', 'pure_pursuit.yaml')

    # Declare the node
    pure_pursuit_node = Node(
        package='path_planning',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        pure_pursuit_node
    ])
