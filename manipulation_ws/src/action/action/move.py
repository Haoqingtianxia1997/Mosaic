#!/usr/bin/env python3
"""
ik_service.py — Receive 6D pose -> Compute IK -> Publish trajectory
"""

import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from action_interfaces.srv import Move


def normalize_quat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return (qx/n, qy/n, qz/n, qw/n) if n else (1.0, 0.0, 0.0, 0.0)


class MoveService(Node):
    def __init__(self):
        super().__init__('move_service')

        self.srv_group = ReentrantCallbackGroup()

        self.create_service(
            Move, 'move_service',
            self.move_cb, callback_group=self.srv_group
        )

        # IK client
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.srv_group
        )
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wait for /compute_ik ...')

        self.current_js = JointState()
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb,
            10
        )

        # Trajectory publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/fr3_arm_controller/joint_trajectory',
            10
        )
    
    def joint_state_cb(self, msg: JointState):
        self.current_js = msg
        # print(f'Receive joint angle：{msg.name} = {msg.position}')


    # ------------------- Main callback -------------------
    def move_cb(self, req: Move.Request, res: Move.Response):
        if not self.current_js.name:             # Joint angles not received yet
            self.get_logger().warn('joint_states not ready, ignoring request')
            res.success = False
            return res

        qx, qy, qz, qw = normalize_quat(req.qx, req.qy, req.qz, req.qw)
        print(f'Receive request：x={req.x}, y={req.y}, z={req.z}, q=({qx}, {qy}, {qz}, {qw})')
        # Assemble IK request
        ik_req = GetPositionIK.Request()
        ik = ik_req.ik_request
        ik.group_name, ik.avoid_collisions = 'fr3_arm', True
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = 'fr3_link0'
        ik.pose_stamped.pose.position.x = req.x
        ik.pose_stamped.pose.position.y = req.y
        ik.pose_stamped.pose.position.z = req.z
        ik.pose_stamped.pose.orientation.x = qx
        ik.pose_stamped.pose.orientation.y = qy
        ik.pose_stamped.pose.orientation.z = qz
        ik.pose_stamped.pose.orientation.w = qw
        robot_state = RobotState()
        robot_state.joint_state = self.current_js
        ik.robot_state = robot_state
        
        ik_res = self.ik_client.call(ik_req)
        
        ok = ik_res and ik_res.error_code.val == 1
        if ok:
            js = ik_res.solution.joint_state
            # compensate
            js.position[6] += 0.785398
            print(js.position, "=================")
            print(js.name, js.position)
            self.publish_traj(js.name, js.position, 5.0)
            self.get_logger().info('✅ IK success, trajectory published.')

            # ===== New wait for actual reach logic =====
            target_name_list = js.name
            target_pos = np.array(js.position)
            pos_tol = 0.01   # Allowed position tolerance (radians)
            reached = False
            while True:
                # Get current joint angles from current_js
                name_to_idx = {name: i for i, name in enumerate(self.current_js.name)}
                try:
                    cur_pos = np.array([self.current_js.position[name_to_idx[n]] for n in target_name_list])
                except Exception as e:
                    self.get_logger().warn(f'Failed to get current joint angles: {e}')
                   
                    continue
                if np.all(np.abs(cur_pos - target_pos) < pos_tol):
                    reached = True
                    break
                
            if reached:
                self.get_logger().info('🎉 Robot arm reached target joint angles')
            else:
                self.get_logger().warn('⚠️ Wait timeout, robot arm not fully reached')

            ok = reached  # You can decide whether to base the response on whether it reached
        else:
            code = ik_res.error_code.val if ik_res else 'None'
            self.get_logger().error(f'❌ IK failed, code={code}')
            ok = False
        
        res.success = ok
        return res
    # ---------------------------------------------

    def publish_traj(self, joint_names, positions, seconds):
        arm = [f'fr3_joint{i}' for i in range(1, 8)]
        d = dict(zip(joint_names, positions))
        names = [j for j in arm if j in d]
        if not names:
            self.get_logger().error('IK solution does not contain arm joints!')
            return
        pos = [d[j] for j in names]

        traj           = JointTrajectory()
        traj.joint_names = names
        pt             = JointTrajectoryPoint()
        pt.positions   = pos
        pt.time_from_start = Duration(sec=int(seconds))
        traj.points.append(pt)
        traj.header.stamp = self.get_clock().now().to_msg()
        self.traj_pub.publish(traj)


def main():
    
    rclpy.init()
    node = MoveService()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
