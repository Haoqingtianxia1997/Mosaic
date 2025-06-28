import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from action_interfaces.srv import Move
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from action_interfaces.srv import Add
from builtin_interfaces.msg import Duration

class AddService(Node):
    def __init__(self):
        super().__init__('add_service')
        
        self.srv_group = ReentrantCallbackGroup()
        
        # service
        self.add_service = self.create_service(
            Add, 'add_service',
            self.add_cb, callback_group=self.srv_group
        )
    
        
        # topic
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_cb, 10
        )
        self.current_js = JointState()
        
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )
        
    def joint_state_cb(self, msg: JointState):
        self.current_js = msg
        # self.get_logger().info(f'Current joint states: {self.current_js.position}')
        
    def add_cb(self, request, response):
        
        times = request.times if request.times > 0 else 1  # Default to 1 if times is not specified or is less than 1
    
        # 1. move robot to default "add" position
        seconds = 5.0
        joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        positions = [-0.48676195798999533, 0.5543532511386491, 0.03989929914025449, -2.2205283593397933, -0.4840910321121013, 4.345486343026757, 0.9207771633742318]
        self.action(seconds, joint_names, positions)
        
        # 2. rotate to add sauce into the soup 
        i= 0       
        while i < times:  # Adjust the number of iterations as needed
            seconds = 1.0
            joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
            positions = [-0.48676195798999533, 0.5543532511386491, 0.03989929914025449, -2.2205283593397933, -0.4840910321121013, 4.345486343026757, 2.7540284229521808]
            self.action(seconds, joint_names, positions)
            seconds = 1.0
            joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
            positions = [-0.48676195798999533, 0.5543532511386491, 0.03989929914025449, -2.2205283593397933, -0.4840910321121013, 4.345486343026757, 0.9207771633742318]
            self.action(seconds, joint_names, positions)
            i += 1
        
        
        response.success = True

        return response
            
        

    def action(self, seconds, joint_names, positions): 
        arm = [f'panda_joint{i}' for i in range(1, 8)]
        d = dict(zip(joint_names, positions))
        names = [j for j in arm if j in d]
        if not names:
            self.get_logger().error('IK 解未包含机械臂关节！')
            return
        pos = [d[j] for j in names]

        traj           = JointTrajectory()
        traj.joint_names = names
        pt             = JointTrajectoryPoint()
        pt.positions   = pos
        pt.time_from_start = Duration(sec=int(seconds))
        traj.points.append(pt)
        traj.header.stamp = self.get_clock().now().to_msg()
        self.traj_pub.publish(traj)
        
        while True:
            # 从 current_js 获取当前关节角
            name_to_idx = {name: i for i, name in enumerate(self.current_js.name)}
            try:
                cur_pos = [self.current_js.position[name_to_idx[n]] for n in names]
            except Exception as e:
                self.get_logger().warn(f'当前关节角获取失败: {e}')
                continue
            
            if all(abs(cp - p) < 0.05 for cp, p in zip(cur_pos, pos)):
                self.get_logger().info('🎉 机械臂已到达目标关节角')
                break
        
        
def main():
    rclpy.init()
    node = AddService()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()