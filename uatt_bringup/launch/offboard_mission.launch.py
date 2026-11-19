"""
Bridges the Gazebo clock and the camera image to ROS 2 and runs the offboard mission on simulation time.
Requires the uXRCE-DDS agent and PX4 SITL with Gazebo to be running.

:author: Jorge Vargas
:date: November 16, 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Declare the launch arguments, the Gazebo bridge (clock and camera),
    the offboard control node and an optional camera viewer.
    """

    use_sim_time = LaunchConfiguration('use_sim_time')
    view_camera = LaunchConfiguration('view_camera')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_view_camera_cmd = DeclareLaunchArgument(
        name='view_camera',
        default_value='false',
        description='Open rqt_image_view on the camera image if true'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        output='screen'
    )

    offboard_control_node = Node(
        package='uatt_px4_interface',
        executable='offboard_control',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    view_camera_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='screen',
        arguments=['/camera/image'],
        condition=IfCondition(view_camera)
    )

    return LaunchDescription([declare_use_sim_time_cmd,
                              declare_view_camera_cmd,
                              gz_bridge_node,
                              offboard_control_node,
                              view_camera_node])