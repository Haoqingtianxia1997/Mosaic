#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from action_interfaces.srv import Fk
import threading

class FKOnlyServer(Node):
    def __init__(self):
        super().__init__('fk_service')

        # MoveIt FK service client
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt FK service...')

        # Data and lock
        self.joint_names = [
            'fr3_joint1','fr3_joint2','fr3_joint3',
            'fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7'
        ]
        self._lock = threading.Lock()
        self.latest_angles = None

        # Separate callback groups to ensure subscription and service do not block each other
        sub_group = MutuallyExclusiveCallbackGroup()
        srv_group = MutuallyExclusiveCallbackGroup()

        # Subscribe to joint angles
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb, 10,
            callback_group=sub_group
        )

        # Create service
        self.create_service(
            Fk, '/fk_service',
            self.handle_request,
            callback_group=srv_group
        )

        self.get_logger().info('✅ /get_ee_position FK service is ready')

    # ---------- Callbacks ----------
    def joint_state_cb(self, msg: JointState):
        name2pos = dict(zip(msg.name, msg.position))
        try:
            with self._lock:
                self.latest_angles = [float(name2pos[n]) for n in self.joint_names]
        except KeyError:
            pass  # joints not complete, ignore

    def handle_request(self, req, res):
        # Get latest joint angles
        with self._lock:
            angles = None if self.latest_angles is None else self.latest_angles.copy()
        if angles is None:
            self.get_logger().warn('⏳ Haven\'t received joint_states, returning NaN')
            res.x = res.y = res.z = float('nan')
            return res

        # Call FK
        fk_req = GetPositionFK.Request()
        fk_req.robot_state.joint_state.name = self.joint_names
        fk_req.robot_state.joint_state.position = angles
        fk_req.fk_link_names = ['fr3_hand']
        fk_req.header.frame_id = 'fr3_link0'

        fut = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if fut.result() and fut.result().error_code.val == 1:
            pose = fut.result().pose_stamped[0].pose
            res.x, res.y, res.z = pose.position.x, pose.position.y, pose.position.z
            self.get_logger().info(
                f'📐 End effector position: {res.x:.3f}, {res.y:.3f}, {res.z:.3f}')
        else:
            self.get_logger().error('❌ FK computation failed')
            res.x = res.y = res.z = float('nan')
        return res

# ------------------- main -------------------
def main():
    rclpy.init()
    node = FKOnlyServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
