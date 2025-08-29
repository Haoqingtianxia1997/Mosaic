import math
import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from action_interfaces.srv import Stir
from trajectory_msgs.msg import JointTrajectory
from action_interfaces.srv import Move

class StirService(Node):
    def __init__(self):
        super().__init__('stir_service')
        self.srv_group = ReentrantCallbackGroup()
        
        #service
        self.create_service(
            Stir, 'stir_service',
            self.stir_cb, callback_group=self.srv_group
        )
        
        #client
        self.move_client = self.create_client(
            Move, 
            'move_service', 
            callback_group=self.srv_group)
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wait for move_service ...')

        
        self.cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self.srv_group
        )
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_cartesian_path ...')
        
        #Topic
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb, 10
        )
        self.current_js = JointState()
        
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/fr3_arm_controller/joint_trajectory',
            10
        )



    def joint_state_cb(self, msg: JointState):
        self.current_js = msg


    def stir_cb(self, req: Stir.Request, res: Stir.Response):
        if not self.current_js.name:
            self.get_logger().warn('joint_states not ready, ignoring request')
            res.success = False
            return res

        # Insert to initial point
        move_req = Move.Request()
        move_req.x = req.center_x  
        move_req.y = req.center_y
        move_req.z = req.center_z - req.move_down_offset  # Insert depth
        move_req.qx = 1.0
        move_req.qy = 0.0
        move_req.qz = 0.0
        move_req.qw = 0.0

        move_res = self.move_client.call(move_req)  # Direct synchronous call until result is available

        if not move_res or not move_res.success:
            self.get_logger().error('Move Service call failed!')
            res.success = False
            return res 

        center_x = move_req.x 
        center_y = move_req.y 
        center_z = move_req.z 
        qx = move_req.qx
        qy = move_req.qy
        qz = move_req.qz
        qw = move_req.qw
        
        circle_points = 200
        waypoints = []
        for i in range(circle_points + 1):
            theta = math.radians(req.start_angle_deg) + 2 * math.pi * i / circle_points
            pose = Pose()
            pose.position.x = center_x + req.radius * math.cos(theta)
            pose.position.y = center_y + req.radius * math.sin(theta)
            pose.position.z = center_z
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            waypoints.append(pose)
        waypoints.append(waypoints[0])

        # Current accumulated stirring time
        accumulated_time = 0.0
        buffer = 0.05   # Buffer for each circle
        robot_state = RobotState()
        robot_state.joint_state = self.current_js

        while accumulated_time < req.stir_time:
            cartesian_req = GetCartesianPath.Request()
            cartesian_req.header.frame_id = 'fr3_link0'
            cartesian_req.start_state = robot_state
            cartesian_req.group_name = 'fr3_arm'
            cartesian_req.link_name = ''
            cartesian_req.waypoints = waypoints
            cartesian_req.max_step = 0.01
            cartesian_req.jump_threshold = 0.0
            cartesian_req.avoid_collisions = True
            # cartesian_req.max_cartesian_speed = req.speed
            # cartesian_req.max_acceleration_scaling_factor = 0.2
            # cartesian_req.max_velocity_scaling_factor = 0.2

            cartesian_res = self.cartesian_client.call(cartesian_req)
            ok = cartesian_res and cartesian_res.fraction > 0.99 and cartesian_res.error_code.val == 1
            if not ok:
                self.get_logger().error(f'Cartesian path planning failed, fraction={getattr(cartesian_res, "fraction", 0)}')
                res.success = False
                return res

            traj = cartesian_res.solution.joint_trajectory
            traj_duration = traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9
            # If executing this circle will time out, truncate the last circle
            if accumulated_time + traj_duration > req.stir_time:
                # Keep only the required part
                allowed_time = req.stir_time - accumulated_time
                new_points = []
                for pt in traj.points:
                    t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    if t <= allowed_time + 1e-4:
                        new_points.append(pt)
                    else:
                        break
                if len(new_points) < 2:
                    self.get_logger().error("stir_time too small, cannot execute even one point of the trajectory!")
                    res.success = False
                    return res
                traj.points = new_points
                traj_duration = allowed_time  # Last circle only executes to specified time

            self.traj_pub.publish(traj)
            self.get_logger().info(f"Wait for {traj_duration:.2f}s, executing stirring trajectory...")
            time.sleep(traj_duration + buffer)
            accumulated_time += traj_duration
            # Set as latest starting point
            robot_state.joint_state = self.current_js

        # Stirring complete, return to initial position
        move_req = Move.Request()
        move_req.x = req.center_x
        move_req.y = req.center_y
        move_req.z = req.center_z 
        move_req.qx = 1.0
        move_req.qy = 0.0
        move_req.qz = 0.0
        move_req.qw = 0.0

        move_res = self.move_client.call(move_req)
        if not move_res or not move_res.success:
            self.get_logger().error('Move Service call failed!')
            res.success = False
            return res 
        
        res.success = True
        self.get_logger().info('Stirring complete!')
        return res



def main():
    rclpy.init()
    node = StirService()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
