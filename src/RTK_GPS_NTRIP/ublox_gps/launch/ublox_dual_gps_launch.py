"""Launch the ublox gps node for both f9p and f9r models with namespaces."""

import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')

    # Configuration for ZED-F9P
    f9p_params = os.path.join(config_directory, 'ublox_f9p.yaml')
    ublox_gps_f9p_node = launch_ros.actions.Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_f9p',
        namespace='f9p',
        output='screen',
        parameters=[f9p_params]
    )

    # Configuration for ZED-F9R
    f9r_params = os.path.join(config_directory, 'zed_f9r.yaml')
    ublox_gps_f9r_node = launch_ros.actions.Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_f9r',
        namespace='f9r',
        output='screen',
        parameters=[f9r_params]
    )

    # Shutdown event handler for the F9P node
    f9p_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ublox_gps_f9p_node,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    # Shutdown event handler for the F9R node
    f9r_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ublox_gps_f9r_node,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return launch.LaunchDescription([
        ublox_gps_f9p_node,
        ublox_gps_f9r_node,
        f9p_event_handler,
        f9r_event_handler
    ])
