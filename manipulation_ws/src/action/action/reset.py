import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.srv_group = ReentrantCallbackGroup()

        self.create_service(
            Trigger, 'reset_service',
            self.reset_cb, callback_group=self.srv_group
        )

        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb,
            10
        )
        self.current_js = JointState()
        
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )

        self.arm_joints = [f'panda_joint{i}' for i in range(1, 8)]
        self.default_joints = [
            0.0, 
            -1.1, 
            0.0,
            -2.6, 
            0.0, 
            1.6, 
            0.8]

    def joint_state_cb(self, msg: JointState):
        self.current_js = msg
        # Optionally, you can log the current joint states
        # self.get_logger().info(f'Current joint states: {self.current_js.position}')
        
    def reset_cb(self, request, response):
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints

        pt = JointTrajectoryPoint()
        pt.positions = self.default_joints
        pt.time_from_start = Duration(sec=5)
        traj.points.append(pt)
        traj.header.stamp = self.get_clock().now().to_msg()

        self.traj_pub.publish(traj)
        while True:
            if all(abs(pos - def_pos) < 0.01 for pos, def_pos in zip(self.current_js.position, self.default_joints)):
                break
        response.success = True
        response.message = "Robot reset to default position"
        return response

def main():
    rclpy.init()
    node = ResetService()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
