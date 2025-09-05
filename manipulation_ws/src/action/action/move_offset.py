#!/usr/bin/env python3
"""
move_offset.py — Receive delta distance and direction -> Get current position -> Calculate target position -> Move with cartesian path
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from action_interfaces.srv import MoveOffset, Fk, Move
import threading


class MoveOffsetService(Node):
    def __init__(self):
        super().__init__('move_offset_service')

        self.srv_group = ReentrantCallbackGroup()

        self.create_service(
            MoveOffset, 'move_offset_service',
            self.move_offset_cb, callback_group=self.srv_group
        )

        # Forward kinematics service client
        self.fk_client = self.create_client(
            Fk, '/fk_service',
            callback_group=self.srv_group
        )
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FK service...')

        # Cartesian move service client
        self.cartesian_client = self.create_client(
            Move, 'move_cartesian_service',
            callback_group=self.srv_group
        )
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Cartesian move service...')

        # Subscribe to joint states
        self._lock = threading.Lock()
        self.latest_joint_state = None
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb, 10
        )

        self.get_logger().info('✅ Move Offset service is ready')

    def joint_state_cb(self, msg: JointState):
        """Store the latest joint state"""
        with self._lock:
            self.latest_joint_state = msg

    def move_offset_cb(self, req: MoveOffset.Request, res: MoveOffset.Response):
        """
        Main callback for move offset service
        """
        self.get_logger().info(f'Received offset request: delta={req.delta}, direction={req.direction}')

        # Validate input
        if req.direction not in ['x', 'y', 'z']:
            self.get_logger().error(f'Invalid direction: {req.direction}. Must be x, y, or z')
            res.success = False
            return res

        # Check if joint states are available
        with self._lock:
            joint_state = self.latest_joint_state
        
        if joint_state is None:
            self.get_logger().error('No joint states available')
            res.success = False
            return res

        try:
            # Get current end effector position
            fk_req = Fk.Request()
            fk_future = self.fk_client.call_async(fk_req)
            
            # Wait for the future to complete without spinning
            import time
            timeout = 5.0
            start_time = time.time()
            while not fk_future.done() and (time.time() - start_time) < timeout:
                time.sleep(0.01)
            
            if not fk_future.done():
                self.get_logger().error('FK service call timed out')
                res.success = False
                return res
                
            if not fk_future.result():
                self.get_logger().error('FK service call failed')
                res.success = False
                return res
            
            fk_res = fk_future.result()
            current_x, current_y, current_z = fk_res.x, fk_res.y, fk_res.z
            self.get_logger().info(f'📐 Current position: ({current_x:.3f}, {current_y:.3f}, {current_z:.3f})')

            # Check for NaN values
            if any([x != x for x in [current_x, current_y, current_z]]):  # NaN check
                self.get_logger().error('FK service returned NaN values')
                res.success = False
                return res

            # Calculate target position based on direction and delta
            target_x, target_y, target_z = current_x, current_y, current_z
            
            if req.direction == 'x':
                target_x += req.delta
            elif req.direction == 'y':
                target_y += req.delta
            elif req.direction == 'z':
                target_z += req.delta

            self.get_logger().info(f'Target position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')

            # Step 3: Move to target position using cartesian move service
            move_req = Move.Request()
            move_req.x = target_x
            move_req.y = target_y
            move_req.z = target_z
            move_req.qx = 1.0
            move_req.qy = 0.0
            move_req.qz = 0.0
            move_req.qw = 0.0

            move_future = self.cartesian_client.call_async(move_req)
            
            # Wait for the future to complete without spinning
            timeout = 30.0
            start_time = time.time()
            while not move_future.done() and (time.time() - start_time) < timeout:
                time.sleep(0.01)
            
            if not move_future.done():
                self.get_logger().error('Cartesian move service call timed out')
                res.success = False
                return res
                
            if not move_future.result():
                self.get_logger().error('Cartesian move service call failed')
                res.success = False
                return res
            
            move_res = move_future.result()
            
            if move_res.success:
                self.get_logger().info(f'Successfully moved {req.delta}m in {req.direction} direction')
                res.success = True
            else:
                self.get_logger().error('Cartesian move failed')
                res.success = False

        except Exception as e:
            self.get_logger().error(f'Exception in move_offset_cb: {str(e)}')
            res.success = False

        return res


def main():
    rclpy.init()
    node = MoveOffsetService()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()