import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()


    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': '/home/simon/ros2_crawlers_gm/src/crawlers_gm/maps/bureau.yaml'}])


    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])


    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld