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
    record = LaunchConfiguration('record')
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('package1'),
        'config',
        'node1_con.yaml'
        )

    node1 = Node(
        package="nodes_pkg",
        executable="node1",
        parameters=[config]
 
    )

# ////////////////////////////////////////////////////////////////////////
    now = datetime.now()
    dt_string = "bag_files/"+now.strftime("%d_%m_%Y %H:%M:%S")


# //////////////////////////////////////////////////////////////////

    bag_ex =launch.actions.ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                record])
        ),
            cmd=['ros2', 'bag', 'record', '/joint_val_topic', '-o', dt_string],
            output='screen'
        )
    
    bag_ex_launch_arg = launch.actions.DeclareLaunchArgument(
        'record',
        default_value='False'
    )

    ld.add_action(node1)
    ld.add_action(bag_ex_launch_arg)
    ld.add_action(bag_ex)

    return ld


