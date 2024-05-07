from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ls = LaunchDescription()
    #position_goals = PathJoinSubstitution(
    #    [FindPackageShare("ais_3dof"), "config", "goal_pos.yaml"])
   
    #listenerNode = Node(
    #            package="ais_3dof",
    #            executable="listener",
    #            #name="listen_camera_node",
    #        )
    computeNode = Node(
                package="ais_3dof",
                executable="compute",
                #name="talker_node",
            )
   
    kinNode = Node(
        package="ais_3dof",
        executable="kin_calc",
    )
    kin_testNode = Node(
        package="ais_3dof",
        executable="test_prg",
    )
    talkerNode = Node(
                package="ais_3dof",
                executable="talker",
                #name="talker_node",
            )
    #ls.add_action(listenerNode)
    #ls.add_action(computeNode)
    ls.add_action(kinNode)
    ls.add_action(kin_testNode)
    ls.add_action(talkerNode)
    return ls
        
    