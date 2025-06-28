import rclpy, math, time, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from action_interfaces.srv import Move
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_interfaces.srv import Grasp
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from action_interfaces.srv import ReturnBack

def normalize_quat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return (qx/n, qy/n, qz/n, qw/n) if n else (1.0, 0.0, 0.0, 0.0)

class ReturnBackService(Node):
    def __init__(self):
        super().__init__('return_back_service')
        
        self.srv_group = ReentrantCallbackGroup()
        
        # service
        self.add_service = self.create_service(
            ReturnBack, 'return_back_service',
            self.grasp_flavoring_cb, callback_group=self.srv_group
        )
        
        # client
        self.move_client = self.create_client(
            Move, 
            'move_service', 
            callback_group=self.srv_group
        )
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_service ...')
            
        self.cartesian_client = self.create_client(
            Move, 
            'move_cartesian_service',
            callback_group=self.srv_group
        )
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_cartesian_service ...')
        
        self.place_client = self.create_client(
            Trigger, 
            'open_service',
            callback_group=self.srv_group
        )
        while not self.place_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for open_service ...')
        
        # topic
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb, 10
        )
        self.current_js = JointState()
        
        # self.traj_pub = self.create_publisher(
        #     JointTrajectory,
        #     '/panda_arm_controller/joint_trajectory',
        #     10
        # )
    
    def joint_state_cb(self, msg: JointState):
        self.current_js = msg
        # self.get_logger().info(f'Current joint states: {self.current_js.position}')
        
    def grasp_flavoring_cb(self, request, response):
        if not self.current_js.name:
            self.get_logger().warn('joint_states 未就绪，忽略本次请求')
            response.success = False
            return response

        # 1. Move to preparation position
        move_prep_req = Move.Request()
        move_prep_req.x = request.x_prep
        move_prep_req.y = request.y_prep
        move_prep_req.z = request.z_prep
        move_prep_req.qx = request.qx_prep
        move_prep_req.qy = request.qy_prep
        move_prep_req.qz = request.qz_prep
        move_prep_req.qw = request.qw_prep

        move_prep_res = self.move_client.call(move_prep_req)
        
        if not move_prep_res or not move_prep_res.success:
            self.get_logger().error('Move Service 调用失败！')
            response.success = False
            return response
        
        # 2. Move to grasp position
        move_place_req = Move.Request()
        move_place_req.x = request.x_place
        move_place_req.y = request.y_place
        move_place_req.z = request.z_place
        move_place_req.qx = request.qx_place
        move_place_req.qy = request.qy_place
        move_place_req.qz = request.qz_place
        move_place_req.qw = request.qw_place

        move_place_res = self.cartesian_client.call(move_place_req)

        if not move_place_res or not move_place_res.success:
            self.get_logger().error('Move Cartesian Service 调用失败！')
            response.success = False
            return response
        
        # 3. Place object
        req = Trigger.Request()
        open_res = self.place_client.call(req)
        if open_res.success:
            self.get_logger().info(f"open_service success: {open_res.success}")
        else:
            self.get_logger().error("Service call failed")
            response.success = False
            return response
        time.sleep(1.5)
        
        # 4. Move back to preparation position
        move_back_req = Move.Request()
        move_back_req.x = request.x_prep
        move_back_req.y = request.y_prep
        move_back_req.z = request.z_prep
        move_back_req.qx = request.qx_prep
        move_back_req.qy = request.qy_prep
        move_back_req.qz = request.qz_prep
        move_back_req.qw = request.qw_prep

        move_back_res = self.cartesian_client.call(move_back_req)

        if not move_back_res or not move_back_res.success:
            self.get_logger().error('Move Cartesian Service 调用失败！')
            response.success = False
            return response
        
        response.success = True
        self.get_logger().info('Return back to home position completed successfully')
        return response
        
        
def main():
    rclpy.init()
    node = ReturnBackService()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()