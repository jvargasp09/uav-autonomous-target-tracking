"""
Bridges the Gazebo clock to ROS 2 and runs the offboard mission on simulation time.
Requires the uXRCE-DDS agent and PX4 SITL with Gazebo to be running.

:author: Jorge Vargas
:date: November 16, 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Declare the use_sim_time argument and the clock bridge and offboard control nodes.
    """

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    offboard_control_node = Node(
        package='uatt_px4_interface',
        executable='offboard_control',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([declare_use_sim_time_cmd,
                              clock_bridge_node,
                              offboard_control_node])