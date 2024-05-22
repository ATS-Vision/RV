import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import launch_params, robot_state_publisher, node_params 
    from common import  armor_tracker_node, buff_tracker_node
    from common import armor_detector_node, buff_detector_Node
    from launch_ros.actions import Node
    from launch import LaunchDescription

    return LaunchDescription([
        robot_state_publisher,
        armor_detector_node,
        armor_tracker_node,
        buff_detector_Node,
        buff_tracker_node,
    ])