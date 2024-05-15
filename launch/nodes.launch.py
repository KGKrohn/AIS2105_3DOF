from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ls = LaunchDescription()

    # Running gazebo 
    other_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('urdf_tutorial'),
                        'launch/display.launch.py')
        )
    )

    ball_tracking = Node(
        package="ais_3dof",
        executable="ball_tracking",
        name="Ball_Track_node",
    )
    Vizu_ball_path = Node(
        package="ais_3dof",
        executable="visualiser",
        name="Visual_Ball_Path",
    )
    Pixel_to_word = Node(
        package="ais_3dof",
        executable="convert_to_centi",
        name="Pixel_to_world",
    )
    vizualiser = Node(
        package="ais_3dof",
        executable="visualiser",
        name="visualize_ball_pos",
    )
    pid_controller = Node(
        package="ais_3dof",
        executable="pid",
        name="PID_node",
    )
    kinNode = Node(
        package="ais_3dof",
        executable="kin_calc",
        name="Kinematic_node",
    )
    kin_testNode = Node(
        package="ais_3dof",
        executable="test_prg",
        name="Test_kin_node",
    )
    Servo_angles = Node(
        package="ais_3dof",
        executable="pub_servo_ang",
        name="pub_to_arduino_node",
    )
    #ls.add_action(other_file) gazebo
    ls.add_action(ball_tracking)
    ls.add_action(Vizu_ball_path)
    ls.add_action(Pixel_to_word)
    ls.add_action(pid_controller)
    ls.add_action(kinNode)
    #ls.add_action(kin_testNode) test node for kinematic
    ls.add_action(Servo_angles)
    return ls
        
    