import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # motor_config = os.path.join(
    #     get_package_share_directory('white_slot'),
    #     'config',
    #     'motor_config.yaml'
    # )

    joy = Node(
        package="joy",
        executable="joy_node",
        name="Joy_Node",
        remappings = [
            ('/joy', '/nana/joy')
        ]
    )

    joystick_control = Node(
        package="nana_core",   
        executable="joystick_node",
        name="Joystick_Node",
        namespace="",
    )
    
    
    drive_node = Node(
        package="nana_core",
        executable="driverpm_node",
        name="Drive_Node",
        namespace="",
    )

    arm_node = Node(
        package="nana_core",
        executable="armrpm_node",
        name="Arm_Node",
        namespace="",
    )

    # node_microros_1 = Node(
    #     package="micro_ros_agent",
    #     executable="micro_ros_agent",
    #     output="screen",
    #     arguments=["serial", "--dev", "/dev/ttyUSB0"],
    # )
    
    
    ld.add_action(joy)
    ld.add_action(joystick_control)
    # ld.add_action(node_microros_1)
    ld.add_action(drive_node)
    ld.add_action(arm_node)


    return ld
