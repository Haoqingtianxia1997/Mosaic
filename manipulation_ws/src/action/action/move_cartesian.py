import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from action_interfaces.srv import Move
from trajectory_msgs.msg import JointTrajectory

def normalize_quat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return (qx/n, qy/n, qz/n, qw/n) if n else (1.0, 0.0, 0.0, 0.0)

class MoveCartesianService(Node):
    def __init__(self):
        super().__init__('move_cartesian_service')

        self.srv_group = ReentrantCallbackGroup()

        self.create_service(
            Move, 'move_cartesian_service',
            self.move_cb, callback_group=self.srv_group
        )

        self.cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self.srv_group
        )
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /compute_cartesian_path ...')

        self.current_js = JointState()
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb, 10
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )

    def joint_state_cb(self, msg: JointState):
        self.current_js = msg

    def move_cb(self, req: Move.Request, res: Move.Response):
        if not self.current_js.name:
            self.get_logger().warn('joint_states 未就绪，忽略本次请求')
            res.success = False
            return res

        # Prepare start state
        robot_state = RobotState()
        robot_state.joint_state = self.current_js

        # Prepare waypoints
        pose = Pose()
        pose.position.x = req.x
        pose.position.y = req.y
        pose.position.z = req.z
        qx, qy, qz, qw = normalize_quat(req.qx, req.qy, req.qz, req.qw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        cartesian_req = GetCartesianPath.Request()
        cartesian_req.header.frame_id = 'panda_link0'
        cartesian_req.start_state = robot_state
        cartesian_req.group_name = 'panda_arm'
        cartesian_req.link_name = ''  # usually leave blank for default tip
        cartesian_req.waypoints = [pose]
        cartesian_req.max_step = 0.01  # meters
        cartesian_req.jump_threshold = 0.0
        cartesian_req.avoid_collisions = True
        cartesian_req.max_cartesian_speed = 0.1
        cartesian_req.max_acceleration_scaling_factor = 0.2
        cartesian_req.max_velocity_scaling_factor = 0.2

        # Synchronous call for clarity (use async in prod)
        cartesian_res = self.cartesian_client.call(cartesian_req)
        
        ok = cartesian_res and cartesian_res.fraction > 0.99 and cartesian_res.error_code.val == 1
        if ok:
            traj = cartesian_res.solution.joint_trajectory
            self.traj_pub.publish(traj)
            
            while True:
                # Wait for the trajectory to be executed
                if self.current_js.name:
                    # Check if the robot is still moving
                    if all(abs(pos - self.current_js.position[i]) < 0.01 for i, pos in enumerate(traj.points[-1].positions)):
                        break

            self.get_logger().info('✅ Cartesian path sent, fraction=%.2f' % cartesian_res.fraction)
        else:
            self.get_logger().error(f'❌ Cartesian planning failed, fraction={getattr(cartesian_res, "fraction", 0)}')
        res.success = ok
        
        
        return res

def main():
    rclpy.init()
    node = MoveCartesianService()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
