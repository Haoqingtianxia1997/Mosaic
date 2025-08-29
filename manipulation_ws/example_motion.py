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

        # Create service client
        self.motion_plan_client = self.create_client(
            GetMotionPlan, 
            '/plan_kinematic_path'
        )
        
        while not self.motion_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for motion planning service...')

        self.get_logger().info('Panda motion planning node has started!')

    def plan_to_joints(self, target_joints):
        """
        Plan to the target joint positions
        target_joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7] - 7 joint angles
        """
        if len(target_joints) != 7:
            self.get_logger().error('Panda robot requires 7 joint angles!')
            return False
            
        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()

        # 1. Set workspace parameters (based on your actual parameters)
        workspace = WorkspaceParameters()
        workspace.header.frame_id = "panda_link0"
        workspace.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        workspace.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        motion_plan_request.workspace_parameters = workspace

        # 2. Set start state (current state)
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
        # Current joint positions (you can get this from the actual robot, using default values here)
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

        # 3. Set goal joint constraints
        goal_constraints = Constraints()
        goal_constraints.name = ''

        # Panda robot joint names
        panda_joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]

        # Set goal positions for each joint
        for joint_name, target_angle in zip(panda_joint_names, target_joints):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_angle
            joint_constraint.tolerance_above = 0.0001  # Consistent with your actual parameters
            joint_constraint.tolerance_below = 0.0001
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        motion_plan_request.goal_constraints = [goal_constraints]

        # 4. Set other parameters (based on your actual parameters)
        motion_plan_request.pipeline_id = "ompl"
        motion_plan_request.planner_id = ""  # Empty string uses default planner
        motion_plan_request.group_name = "panda_arm"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.1
        motion_plan_request.max_acceleration_scaling_factor = 0.1
        motion_plan_request.cartesian_speed_limited_link = ""
        motion_plan_request.max_cartesian_speed = 0.0
        
        request.motion_plan_request = motion_plan_request

        # Call service
        self.get_logger().info(f'Planning to target position: {target_joints}')
        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                self.get_logger().info('Motion planning succeeded!')
                self.get_logger().info(f'Planning time: {response.motion_plan_response.planning_time}s')
                return True
            else:
                self.get_logger().error(f'Planning failed, error code: {response.motion_plan_response.error_code.val}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False

    def get_current_joint_positions(self):
        """
        Get current joint positions (example, you may need to get this from /joint_states topic in practice)
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

    # Example 1: Use target positions from your topic
    target_positions_1 = [
        0.91168879074049,
        -0.6356682041223909,
        -0.9177548153274062,
        -2.095857920213382,
        -0.4267367487926457,
        1.9671265833791591,
        0.8318783715901764
    ]

    # Example 2: Another position
    target_positions_2 = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]  # Home position

    # Execute planning
    print("Planning to position 1...")
    if planner.plan_to_joints(target_positions_1):
        planner.get_logger().info('Position 1 planning succeeded!')

    # Wait a moment
    rclpy.spin_once(planner, timeout_sec=2.0)

    print("Planning to position 2...")
    if planner.plan_to_joints(target_positions_2):
        planner.get_logger().info('Position 2 planning succeeded!')
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()