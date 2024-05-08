from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ls = LaunchDescription()

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
    talkerNode = Node(
        package="ais_3dof",
        executable="talker",
        name="pub_to_arduino_node",
    )
    ls.add_action(pid_controller)
    ls.add_action(kinNode)
    #ls.add_action(kin_testNode)
    ls.add_action(talkerNode)
    return ls
        
    