#----------------------------------------------------------------------------------------
#================]  this is the launch file for micro ros nodes  [============================
#----------------------------------------------------------------------------------------
# run this node before connecting all the micro-controllers
#------------------------------------------------------------

import os
from ament_index_python.packages import get_package_share_directory
from concurrent.futures import Executor
from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from datetime import datetime


def generate_launch_description():
    ld = LaunchDescription()



    micro_ros_ex1 =launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/serial/by-id/usb-Teensyduino_USB_Serial_7637910-if00'],
            output='screen'
        )

    micro_ros_ex2 =launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/serial/by-id/usb-Teensyduino_USB_Serial_7556350-if00'],
            output='screen'
        )
    
    micro_ros_ex3 =launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev//serial/by-id/usb-Teensyduino_USB_Serial_7555460-if00'],
            output='screen'
        )
    
    ld.add_action(micro_ros_ex1)
    ld.add_action(micro_ros_ex2)
    ld.add_action(micro_ros_ex3)

    return ld
