from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    bag_record = LaunchConfiguration('bag_record')

    bag_record_arg = DeclareLaunchArgument(
            'bag_record',
            default_value='False'
        )

    bag_record_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/turtlebot3_world.launch.py'])
        )

    pkg_exe = Node(
            package='obstacle_avoidance',
            executable='walker',
        )

    execute_cmd = ExecuteProcess(
        condition=IfCondition(bag_record),
        cmd=[
            'ros2', 'bag', 'record', '-a', '-x "/camera.+"'
        ],
        shell=True
        )
    return LaunchDescription([
        bag_record_arg,
        bag_record_launch,
        pkg_exe,
        execute_cmd,
    ])