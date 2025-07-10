#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from action_interfaces.srv import Fk            # ← 替换为你的包名
import threading

class FKOnlyServer(Node):
    def __init__(self):
        super().__init__('fk_servive')

        # MoveIt FK 服务客户端
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 MoveIt FK 服务...')

        # 数据与锁
        self.joint_names = [
            'panda_joint1','panda_joint2','panda_joint3',
            'panda_joint4','panda_joint5','panda_joint6','panda_joint7'
        ]
        self._lock = threading.Lock()
        self.latest_angles = None

        # 分离的 callback group，确保订阅与服务互不阻塞
        sub_group = MutuallyExclusiveCallbackGroup()
        srv_group = MutuallyExclusiveCallbackGroup()

        # 订阅关节角
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb, 10,
            callback_group=sub_group
        )

        # 创建服务
        self.create_service(
            Fk, '/fk_service',
            self.handle_request,
            callback_group=srv_group
        )

        self.get_logger().info('✅ /get_ee_position FK 服务已就绪')

    # ---------- Callbacks ----------
    def joint_state_cb(self, msg: JointState):
        name2pos = dict(zip(msg.name, msg.position))
        try:
            with self._lock:
                self.latest_angles = [float(name2pos[n]) for n in self.joint_names]
        except KeyError:
            pass  # joints 未齐全，忽略

    def handle_request(self, req, res):
        # 取最新关节角
        with self._lock:
            angles = None if self.latest_angles is None else self.latest_angles.copy()
        if angles is None:
            self.get_logger().warn('⏳ 还未收到 joint_states，返回 NaN')
            res.x = res.y = res.z = float('nan')
            return res

        # 调用 FK
        fk_req = GetPositionFK.Request()
        fk_req.robot_state.joint_state.name = self.joint_names
        fk_req.robot_state.joint_state.position = angles
        fk_req.fk_link_names = ['panda_hand']
        fk_req.header.frame_id = 'panda_link0'

        fut = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if fut.result() and fut.result().error_code.val == 1:
            pose = fut.result().pose_stamped[0].pose
            res.x, res.y, res.z = pose.position.x, pose.position.y, pose.position.z
            self.get_logger().info(
                f'📐 末端位置: {res.x:.3f}, {res.y:.3f}, {res.z:.3f}')
        else:
            self.get_logger().error('❌ FK 计算失败')
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
