#!/usr/bin/env python3
"""
ik_service.py — 接收 6D pose -> 计算 IK -> 发布轨迹
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

        # 轨迹发布器
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )
    
    def joint_state_cb(self, msg: JointState):
        self.current_js = msg
        # print(f'接收到关节角：{msg.name} = {msg.position}')


    # ------------------- 主回调 -------------------
    def move_cb(self, req: Move.Request, res: Move.Response):
        if not self.current_js.name:             # 还没收到关节角
            self.get_logger().warn('joint_states 未就绪，忽略本次请求')
            res.success = False
            return res

        qx, qy, qz, qw = normalize_quat(req.qx, req.qy, req.qz, req.qw)
        print(f'接收到请求：x={req.x}, y={req.y}, z={req.z}, q=({qx}, {qy}, {qz}, {qw})')
        # 组装 IK 请求
        ik_req = GetPositionIK.Request()
        ik = ik_req.ik_request
        ik.group_name, ik.avoid_collisions = 'panda_arm', True
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = 'panda_link0'
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
            print(js.name, js.position)
            self.publish_traj(js.name, js.position, 3.0)
            self.get_logger().info('✅ IK success，轨迹已发送')
            
            # ===== 新增等待实际到位逻辑 =====
            target_name_list = js.name
            target_pos = np.array(js.position)
            # 你可以适当调整阈值
            pos_tol = 0.01   # 允许的角度误差（弧度）
            reached = False
            while True:
                # 从 current_js 获取当前关节角
                name_to_idx = {name: i for i, name in enumerate(self.current_js.name)}
                try:
                    cur_pos = np.array([self.current_js.position[name_to_idx[n]] for n in target_name_list])
                except Exception as e:
                    self.get_logger().warn(f'当前关节角获取失败: {e}')
                   
                    continue
                if np.all(np.abs(cur_pos - target_pos) < pos_tol):
                    reached = True
                    break
                
            if reached:
                self.get_logger().info('🎉 机械臂已到达目标关节角')
            else:
                self.get_logger().warn('⚠️ 等待超时，机械臂未完全到位')

            ok = reached  # 你可以决定是否以是否到位来决定 response
        else:
            code = ik_res.error_code.val if ik_res else 'None'
            self.get_logger().error(f'❌ IK 失败，code={code}')
            ok = False
        
        res.success = ok
        return res
    # ---------------------------------------------

    def publish_traj(self, joint_names, positions, seconds):
        arm = [f'panda_joint{i}' for i in range(1, 8)]
        d = dict(zip(joint_names, positions))
        names = [j for j in arm if j in d]
        if not names:
            self.get_logger().error('IK 解未包含机械臂关节！')
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
