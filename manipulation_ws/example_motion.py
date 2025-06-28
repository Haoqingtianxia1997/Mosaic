#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, WorkspaceParameters, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

class PandaMotionPlanner(Node):
    def __init__(self):
        super().__init__('panda_motion_planner')
        
        # 创建服务客户端
        self.motion_plan_client = self.create_client(
            GetMotionPlan, 
            '/plan_kinematic_path'
        )
        
        while not self.motion_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待运动规划服务...')
        
        self.get_logger().info('Panda运动规划节点已启动!')

    def plan_to_joints(self, target_joints):
        """
        规划到目标关节位置
        target_joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7] - 7个关节角度
        """
        if len(target_joints) != 7:
            self.get_logger().error('Panda机械臂需要7个关节角度!')
            return False
            
        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()
        
        # 1. 设置工作空间参数（根据你的实际参数）
        workspace = WorkspaceParameters()
        workspace.header.frame_id = "panda_link0"
        workspace.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        workspace.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        motion_plan_request.workspace_parameters = workspace
        
        # 2. 设置起始状态（当前状态）
        start_state = RobotState()
        start_state.joint_state = JointState()
        start_state.joint_state.header.frame_id = "panda_link0"
        start_state.joint_state.name = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
            'panda_finger_joint1',
            'panda_finger_joint2'
        ]
        # 当前关节位置（你可以从实际机器人获取，这里使用默认值）
        current_positions = [
            0.9719375531084352,
            -0.47425465226857394,
            -0.8257876465417748,
            -1.2771662878630057,
            -0.2900058357911474,
            1.277321887441759,
            0.7195915649348764,
            0.03992697596549988,
            0.03992697596549988
        ]
        start_state.joint_state.position = current_positions
        start_state.joint_state.velocity = []
        start_state.joint_state.effort = []
        start_state.is_diff = False
        motion_plan_request.start_state = start_state
        
        # 3. 设置目标关节约束
        goal_constraints = Constraints()
        goal_constraints.name = ''
        
        # Panda机械臂关节名称
        panda_joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]
        
        # 为每个关节设置目标位置
        for joint_name, target_angle in zip(panda_joint_names, target_joints):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_angle
            joint_constraint.tolerance_above = 0.0001  # 与你的实际参数一致
            joint_constraint.tolerance_below = 0.0001
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        motion_plan_request.goal_constraints = [goal_constraints]
        
        # 4. 设置其他参数（根据你的实际参数）
        motion_plan_request.pipeline_id = "ompl"
        motion_plan_request.planner_id = ""  # 空字符串使用默认规划器
        motion_plan_request.group_name = "panda_arm"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.1
        motion_plan_request.max_acceleration_scaling_factor = 0.1
        motion_plan_request.cartesian_speed_limited_link = ""
        motion_plan_request.max_cartesian_speed = 0.0
        
        request.motion_plan_request = motion_plan_request
        
        # 调用服务
        self.get_logger().info(f'规划到目标位置: {target_joints}')
        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                self.get_logger().info('运动规划成功!')
                self.get_logger().info(f'规划时间: {response.motion_plan_response.planning_time}s')
                return True
            else:
                self.get_logger().error(f'规划失败，错误代码: {response.motion_plan_response.error_code.val}')
                return False
        else:
            self.get_logger().error('服务调用失败')
            return False

    def get_current_joint_positions(self):
        """
        获取当前关节位置（示例，实际中你可能需要从/joint_states话题获取）
        """
        return [
            0.9719375531084352,
            -0.47425465226857394,
            -0.8257876465417748,
            -1.2771662878630057,
            -0.2900058357911474,
            1.277321887441759,
            0.7195915649348764
        ]

def main():
    rclpy.init()
    planner = PandaMotionPlanner()
    
    # 示例1：使用你话题中的目标位置
    target_positions_1 = [
        0.91168879074049,
        -0.6356682041223909,
        -0.9177548153274062,
        -2.095857920213382,
        -0.4267367487926457,
        1.9671265833791591,
        0.8318783715901764
    ]
    
    # 示例2：另一个位置
    target_positions_2 = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]  # Home position
    
    # 执行规划
    print("规划到位置1...")
    if planner.plan_to_joints(target_positions_1):
        planner.get_logger().info('位置1规划成功!')
    
    # 等待一下
    rclpy.spin_once(planner, timeout_sec=2.0)
    
    print("规划到位置2...")
    if planner.plan_to_joints(target_positions_2):
        planner.get_logger().info('位置2规划成功!')
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()