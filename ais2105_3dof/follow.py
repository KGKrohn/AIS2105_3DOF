import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
import sys
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
class StowCommand(Node):
    def __init__(self):
        super().__init__('Pubish_joint_trajec')
        self.joint_state = JointState()
        self.trajectory_client = ActionClient(self, JointTrajectory, '/joint_trajectory_controller/joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 1)
        self.subscription
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
    def issue_stow_command(self):
        joint_state = self.joint_state
        if (joint_state is not None):
            self.get_logger().info('stowing...')
            stow_point1 = JointTrajectoryPoint()
            stow_point2 = JointTrajectoryPoint()
            duration1 = Duration(seconds=0.0)
            duration2 = Duration(seconds=4.0)
            stow_point1.time_from_start = duration1.to_msg()
            stow_point2.time_from_start = duration2.to_msg()
            joint_value1 = joint_state.position[1] # joint_lift is at index 1
            joint_value2 = joint_state.position[0] # wrist_extension is at index 0
            joint_value3 = joint_state.position[8] # joint_wrist_yaw is at index 8
            stow_point1.positions = [joint_value1, joint_value2, joint_value3]
            stow_point2.positions = [25.0, 30.0, 100.14]
            trajectory_goal = JointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = ['ard_motorA', 'ard_motorB', 'ard_motorC']
            trajectory_goal.trajectory.points = [stow_point2]
            trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory_goal.trajectory.header.frame_id = 'base_link'
            self.trajectory_client.send_goal_async(trajectory_goal)
            self.get_logger().info('Sent stow goal = {0}'.format(trajectory_goal))
def main(args=None):
    rclpy.init(args=args)

    stow_command = StowCommand()

    rclpy.spin_once(stow_command)
    stow_command.issue_stow_command()
    rclpy.spin(stow_command)
    stow_command.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()