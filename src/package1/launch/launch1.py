#----------------------------------------------------------------------------------------
#          this is the launch file for my nodes
#----------------------------------------------------------------------------------------
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

from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    record = LaunchConfiguration('record')
    ld = LaunchDescription()

    config = os.path.join( # join the my config files so that package1 can acess
        get_package_share_directory('package1'),
        'config',
        'node1_con.yaml'
        )


    node1 = Node(
        package="package1",
        executable="node1",
        parameters=[config]
 
    )

    node2 = Node(
        package="package1",
        executable="node2" ,
        parameters=[config]
    )



# ////////////////////////////////////////////////////////////////////////
# this is for the bag files to be stored with the current time & date
    now = datetime.now()
    dt_string = "bag_files/"+now.strftime("%d_%m_%Y %H:%M:%S")
# //////////////////////////////////////////////////////////////////

# bag file recording : to record add an argument "record:=True" when using the launch file
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
    ld.add_action(node2)
    ld.add_action(bag_ex_launch_arg)
    ld.add_action(bag_ex)

    return ld


