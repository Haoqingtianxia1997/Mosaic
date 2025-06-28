import rclpy
from rclpy.action import ActionClient
from franka_msgs.action import Move, Grasp
from rclpy.node import Node
from rclpy.duration import Duration


class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        
        # Create action clients for 'move' and 'grasp' actions
        self.move_client = ActionClient(self, Move, '/panda_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/panda_gripper/grasp')

    def send_move_goal(self, width, speed):
        # Wait for the action server to be available
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Move action server not available')
            return
        
        # Create the goal message for the move action
        goal_msg = Move.Goal()
        goal_msg.width = width
        goal_msg.speed = speed

        # Send the goal and handle result when finished
        self.get_logger().info(f'Sending move goal: width={width}, speed={speed}')
        self._send_goal(self.move_client, goal_msg)

    def send_grasp_goal(self, width, speed, force, epsilon_inner, epsilon_outer):
        # Wait for the action server to be available
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Grasp action server not available')
            return

        # Create the goal message for the grasp action
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon_inner
        goal_msg.epsilon.outer = epsilon_outer

        # Send the goal and handle result when finished
        self.get_logger().info(f'Sending grasp goal: width={width}, speed={speed}, force={force}')
        self._send_goal(self.grasp_client, goal_msg)

    def _send_goal(self, client, goal_msg):
        # Send the goal asynchronously and handle feedback/results
        future = client.send_goal_async(goal_msg)

        # Check result after sending
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        # This is called when the action completes
        result = future.result()
        if result:
            self.get_logger().info(f'Action completed: {result}')
        else:
            self.get_logger().warn('Action failed or was canceled')


def main(args=None):
    rclpy.init(args=args)
    
    # Create the gripper control node
    gripper_control_node = GripperControlNode()

    # Send a 'move' goal
    gripper_control_node.send_move_goal(0.08, 0.1)  # Open gripper to 8 cm with 0.1 m/s speed
    
    # Send a 'grasp' goal (grip an object)
    gripper_control_node.send_grasp_goal(0.01, 0.1, 50.0, 0.04, 0.04)  # Close gripper to 1 cm with 20 N force

    # Spin the node to keep it active
    rclpy.spin(gripper_control_node)

    # Shut down ROS2
    gripper_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
