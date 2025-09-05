import rclpy, time
from rclpy.node import Node
from action_interfaces.srv import GraspDetect
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

class GraspDetectionService(Node):
    def __init__(self):
        super().__init__('grasp_detection_service')

        self.srv_group = ReentrantCallbackGroup()

        self.grasp_detection_service = self.create_service(
            GraspDetect, 'grasp_detection_service',
            self.grasp_detection_cb, callback_group=self.srv_group
        )
        
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb,
            10,
            callback_group=self.srv_group
        )
        
        self.current_js = JointState()
        self.current_grasp_width = 0.08
        
        self.min_grasp_width = 0.005
        self.max_empty_width = 0.076
        
        self.get_logger().info('Grasp Detection Service is ready.')

    def grasp_detection_cb(self, request, response):
        self.get_logger().info('Starting grasp detection...')

        # Wait a moment to ensure the latest sensor data is available
        time.sleep(0.1)

        # Check if grasping was successful
        grasp_success = self.detect_grasp_success()
        
        # 设置服务调用成功（总是True，表示服务运行成功）
        response.success = True
        # 设置抓取成功的布尔值
        response.grasp_success = grasp_success
        
        if grasp_success:
            response.message = f'Object grasped - Gripper width: {self.current_grasp_width:.4f}m'
            self.get_logger().info('✅ Grasp detected successfully!')
        else:
            response.message = f'No grasp detected - Gripper width: {self.current_grasp_width:.4f}m'
            self.get_logger().info('❌ No grasp detected')
        
        return response

    def detect_grasp_success(self):
        """
        position check to detect if an object is grasped.
        """
        width_indicates_grasp = (self.min_grasp_width < self.current_grasp_width < self.max_empty_width)

        self.get_logger().info(f'Gripper width: {self.current_grasp_width:.4f}m (Range: {self.min_grasp_width:.3f} - {self.max_empty_width:.3f})')
        self.get_logger().info(f'Width indicates grasp: {width_indicates_grasp}')

        # If the gripper width indicates an object is present, consider the grasp successful
        return width_indicates_grasp

    def joint_state_cb(self, msg: JointState):
        """Joint state callback to update the current gripper width"""
        self.current_js = msg
        # Gripper width = Left finger + Right finger position
        if len(msg.position) >= 2:
            self.current_grasp_width = msg.position[-2] + msg.position[-1]

def main(args=None):
    rclpy.init(args=args)
    node = GraspDetectionService()
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