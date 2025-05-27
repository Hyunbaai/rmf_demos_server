import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    nav_graph_file = LaunchConfiguration("nav_graph_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    server_uri = LaunchConfiguration("server_uri").perform(context)
    easy_fleet = LaunchConfiguration("easy_fleet").perform(context).lower() == "true"

    # YAML 파싱
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    robots_config = config['rmf_fleet']['robots']
    fleet_name = config['rmf_fleet']['name']
    fleet_responsive_wait = config['rmf_fleet'].get('responsive_wait', False)

    # 공통 파라미터
    param_dict = {
        'use_sim_time': use_sim_time.lower() == 'true',
        'server_uri': server_uri,
    }

    for robot_name in robots_config:
        responsive_wait = robots_config[robot_name].get('responsive_wait', fleet_responsive_wait)
        param_dict[f"responsive_wait_{robot_name}"] = responsive_wait

    # 선택한 노드 결정
    executable = "easy_fleet_adapter" if easy_fleet else "fleet_adapter"

    # -sim 옵션 필요 여부
    sim_args = ["-sim"] if use_sim_time.lower() == "true" else []

    fleet_node = Node(
        package='rmf_demos_fleet_adapter',
        executable=executable,
        output='screen',
        arguments=["-c", config_file, "-n", nav_graph_file] + sim_args,
        parameters=[param_dict]
    )

    return [fleet_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('nav_graph_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('server_uri', default_value=''),
        DeclareLaunchArgument('easy_fleet', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])
