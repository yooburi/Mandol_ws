import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    gps_to_utm_share_dir = get_package_share_directory('gps_to_utm')

    # Define the path to the parameters file
    params_file = os.path.join(gps_to_utm_share_dir, 'config', 'tf_gps_csv.yaml')

    # Path to the RViz config file
    rviz_config_file = os.path.join(gps_to_utm_share_dir, 'launch', 'tf.rviz')

    # Declare the node
    tf_gps_csv_node = Node(
        package='gps_to_utm',
        executable='tf_gps_csv_node',
        name='tf_gps_csv_node',
        output='screen',
        parameters=[params_file]
    )

    # Declare the rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        tf_gps_csv_node,
        rviz_node
    ])