#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK




class MoveClient(Node):
    def __init__(self, pose_input):
        super().__init__('ik_move_client')
        self.pose_input = pose_input

        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for /compute_ik service...')

        self.traj_pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)

    def call_and_move(self):
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = 'panda_arm'
        ik.avoid_collisions = True

        ps = PoseStamped()
        ps.header.frame_id = 'panda_link0'  # 或 "world"
        x, y, z, qx, qy, qz, qw = self.pose_input
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        ik.pose_stamped = ps
        ik.robot_state = RobotState()

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res and res.error_code.val == 1:
            js = res.solution.joint_state
            self.get_logger().info('✅ IK success')
            self.send_trajectory(js.name, js.position, seconds=3.0)
        else:
            code = res.error_code.val if res else 'None'
            self.get_logger().error(f'❌ IK failed,: {code}')

    def send_trajectory(self, joint_names, positions, seconds: float = 3.0):
        arm_joints = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
        ]
        name_pos_dict = dict(zip(joint_names, positions))
        filt_names = [j for j in arm_joints if j in name_pos_dict]
        filt_pos = [name_pos_dict[j] for j in filt_names]

        traj = JointTrajectory()
        traj.joint_names = filt_names

        pt = JointTrajectoryPoint()
        pt.positions = filt_pos
        pt.time_from_start = Duration(
            sec=int(seconds),
            nanosec=int((seconds % 1) * 1e9)
        )
        traj.points.append(pt)
        traj.header.stamp = self.get_clock().now().to_msg()

        self.traj_pub.publish(traj)
        self.get_logger().info('📤 Trajectory published')


def main():
    if len(sys.argv) != 8:
        print("❌ Usage：ros2 run action move_service_default x y z qx qy qz qw")
        return

    try:
        pose = [float(v) for v in sys.argv[1:]]
    except ValueError:
        print("❌ Input values must be float numbers.")
        return

    rclpy.init()
    node = MoveClient(pose)
    node.call_and_move()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
