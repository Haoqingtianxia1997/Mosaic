import rclpy
from rclpy.action import ActionClient
from franka_msgs.action import Grasp
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup

class CloseService(Node):
    def __init__(self):
        super().__init__('close_service')

        self.srv_group = ReentrantCallbackGroup()

        # Action client for grasp
        self.grasp_client = ActionClient(self, Grasp, '/panda_gripper/grasp', callback_group=self.srv_group)

        self.place = self.create_service(
            Trigger, 'close_service',
            self.place_cb, callback_group=self.srv_group
        )
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb,
            10,
            callback_group=self.srv_group  # 可选，不写也行
        )
        self.current_js = JointState()
        self.current_grasp_width = 0.08

    def place_cb(self, request, response):
        self.send_grasp_goal(0.01, 0.1, 50.0, 0.04, 0.04)
        while True:
            if self.current_grasp_width < 0.05:
                response.success = True
                response.message = 'Gripper closed successfully'
                self.current_grasp_width = 0.08  # Reset current grasp width
                return response

    def joint_state_cb(self, msg: JointState):
        self.current_js = msg
        
        self.current_grasp_width = msg.position[-2] + msg.position[-1]
       
    def send_grasp_goal(self, width, speed, force, epsilon_inner, epsilon_outer):
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Grasp action server not available')
            return

        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon_inner
        goal_msg.epsilon.outer = epsilon_outer

        self.get_logger().info(
            f'Sending grasp goal: width={width}, speed={speed}, force={force}')
        self._send_goal(self.grasp_client, goal_msg)

    def _send_goal(self, client, goal_msg):
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info(f'Action completed: {result}')
        else:
            self.get_logger().warn('Action failed or was canceled')

def main(args=None):
    rclpy.init(args=args)
    node = CloseService()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
