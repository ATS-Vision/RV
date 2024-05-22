import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
    ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                'publish_frequency': 1000.0}]
)

node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

armor_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'buff_detector:='+launch_params['detector_log_level']],
    )

armor_tracker_node = Node(
    package='armor_tracker',
    executable='armor_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    ros_arguments=['--log-level', 'armor_tracker:='+launch_params['tracker_log_level']],
)

buff_detector_Node= Node(
        package='buff_detector',
        executable='buff_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'buff_detector:='+launch_params['detector_log_level']],
    )

buff_tracker_node = Node(
    package='buff_tracker',
    executable='buff_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    ros_arguments=['--log-level','buff_tracker:='+launch_params['tracker_log_level']],
)


