import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

import subprocess 


def generate_launch_description():
    res=[]
    moveit_config = MoveItConfigsBuilder("m0609", package_name="m0609_moveit").to_moveit_configs()
    res.append(generate_demo_launch(moveit_config))

    control_node = Node(
        package="m0609_controller",
        executable="m0609_controller_server",
        name="m0609_controller_server",
        output="screen"
    )
    res.append(control_node)

    api_drfl_file = Node(
        package="m0609_controller",
        executable="m0609_controller_client",
        name="m0609_controller_client",
    )
    res.append(api_drfl_file)
    

    return LaunchDescription(res)

