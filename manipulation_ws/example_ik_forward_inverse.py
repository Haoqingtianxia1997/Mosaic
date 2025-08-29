# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Pose, Point, Quaternion
# from moveit_msgs.srv import GetPositionIK, GetPositionFK
# from moveit_msgs.msg import PositionIKRequest, PositionFKRequest, RobotState
# import numpy as np
# import time

# class MoveItIKFKSolver(Node):
#     def __init__(self):
#         super().__init__('moveit_ik_fk_solver')
        
#         # MoveIt IK服务客户端
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
#         # MoveIt FK服务客户端
#         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
#         # 等待服务可用
#         while not self.ik_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('等待MoveIt IK服务...')
        
#         while not self.fk_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('等待MoveIt FK服务...')
        
#         self.get_logger().info("✅ MoveIt IK/FK求解器已就绪")

#     def inverse_kinematics(self, world_position, world_orientation=None) -> list:
#         """
#         使用MoveIt计算逆运动学
        
#         Args:
#             world_position: [x, y, z] 世界坐标位置
#             world_orientation: [x, y, z, w] 四元数方向 (可选)
        
#         Returns:
#             list: 关节角度 [joint1, joint2, ..., joint7]
#         """
#         # 创建IK请求
#         request = GetPositionIK.Request()
        
#         # 设置目标位置和姿态
#         request.ik_request.group_name = "fr3_arm"  # 机械臂组名
#         request.ik_request.robot_state.joint_state.name = [
#             'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
#             'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
#         ]
        
#         # 当前关节状态（作为初始猜测）
#         request.ik_request.robot_state.joint_state.position = [
#             0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0
#         ]
        
#         # 设置末端执行器链接
#         request.ik_request.ik_link_name = "fr3_hand"
        
#         # 目标位置
#         target_pose = Pose()
#         target_pose.position = Point(
#             x=float(world_position[0]),
#             y=float(world_position[1]),
#             z=float(world_position[2])
#         )
        
#         # 目标方向
#         if world_orientation is None:
#             # 默认方向：末端执行器朝下
#             target_pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
#         else:
#             target_pose.orientation = Quaternion(
#                 x=float(world_orientation[0]),
#                 y=float(world_orientation[1]),
#                 z=float(world_orientation[2]),
#                 w=float(world_orientation[3])
#             )
        
#         request.ik_request.pose_stamped.header.frame_id = "fr3_link0"
#         request.ik_request.pose_stamped.pose = target_pose
        
#         # 设置超时和尝试次数
#         request.ik_request.timeout.sec = 5
#         # request.ik_request.attempts = 10
        
#         # 调用服务
#         try:
#             future = self.ik_client.call_async(request)
#             rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
#             if future.result() is not None:
#                 response = future.result()
#                 if response.error_code.val == 1:  # SUCCESS
#                     joint_angles = list(response.solution.joint_state.position)
#                     self.get_logger().info(f"✅ IK求解成功: {[f'{angle:.3f}' for angle in joint_angles]}")
#                     return joint_angles
#                 else:
#                     error_msg = f"IK求解失败，错误代码: {response.error_code.val}"
#                     self.get_logger().error(f"❌ {error_msg}")
#                     raise RuntimeError(error_msg)
#             else:
#                 raise TimeoutError("IK服务调用超时")
                
#         except Exception as e:
#             self.get_logger().error(f"❌ IK求解异常: {e}")
#             raise

#     def forward_kinematics(self, joint_angles, link_names=None) -> dict:
#         """
#         使用MoveIt计算正运动学
        
#         Args:
#             joint_angles: [joint1, joint2, ..., joint7] 关节角度列表
#             link_names: 要计算的链接名称列表，默认为末端执行器
        
#         Returns:
#             dict: {link_name: {'position': [x, y, z], 'orientation': [x, y, z, w]}}
#         """
#         if link_names is None:
#             link_names = ["fr3_hand"]  # 默认计算末端执行器位置
        
#         # 创建FK请求
#         request = GetPositionFK.Request()
        
#         # 设置机器人状态
#         request.robot_state.joint_state.name = [
#             'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
#             'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
#         ]
#         request.robot_state.joint_state.position = [float(angle) for angle in joint_angles]
        
#         # 设置要计算的链接
#         request.fk_link_names = link_names
        
#         # 设置参考坐标系
#         request.header.frame_id = "fr3_link0"
        
#         # 调用服务
#         try:
#             future = self.fk_client.call_async(request)
#             rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
#             if future.result() is not None:
#                 response = future.result()
#                 if response.error_code.val == 1:  # SUCCESS
#                     results = {}
#                     for i, link_name in enumerate(link_names):
#                         pose = response.pose_stamped[i].pose
#                         results[link_name] = {
#                             'position': [pose.position.x, pose.position.y, pose.position.z],
#                             'orientation': [pose.orientation.x, pose.orientation.y, 
#                                           pose.orientation.z, pose.orientation.w]
#                         }
                    
#                     self.get_logger().info(f"✅ FK求解成功")
#                     for link_name, data in results.items():
#                         pos = data['position']
#                         self.get_logger().info(f"  {link_name}: 位置=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                    
#                     return results
#                 else:
#                     error_msg = f"FK求解失败，错误代码: {response.error_code.val}"
#                     self.get_logger().error(f"❌ {error_msg}")
#                     raise RuntimeError(error_msg)
#             else:
#                 raise TimeoutError("FK服务调用超时")
                
#         except Exception as e:
#             self.get_logger().error(f"❌ FK求解异常: {e}")
#             raise

#     def get_end_effector_position(self, joint_angles) -> tuple:
#         """
#         获取末端执行器的世界坐标位置
        
#         Args:
#             joint_angles: 关节角度列表
            
#         Returns:
#             tuple: (position, orientation) 
#                    position: [x, y, z] 位置
#                    orientation: [x, y, z, w] 四元数方向
#         """
#         try:
#             result = self.forward_kinematics(joint_angles, ["fr3_hand"])
#             if "fr3_hand" in result:
#                 position = result["fr3_hand"]["position"]
#                 orientation = result["fr3_hand"]["orientation"]
#                 return position, orientation
#             else:
#                 raise RuntimeError("未能获取末端执行器位置")
#         except Exception as e:
#             self.get_logger().error(f"❌ 获取末端执行器位置失败: {e}")
#             raise

#     def get_multiple_link_positions(self, joint_angles) -> dict:
#         """
#         获取多个链接的世界坐标位置
        
#         Args:
#             joint_angles: 关节角度列表
            
#         Returns:
#             dict: 多个链接的位置信息
#         """
#         # 常用的链接名称
#         link_names = [
#             "fr3_link1",
#             "fr3_link2", 
#             "fr3_link3",
#             "fr3_link4",
#             "fr3_link5",
#             "fr3_link6",
#             "fr3_link7",
#             "fr3_hand"
#         ]
        
#         return self.forward_kinematics(joint_angles, link_names)

# # 测试函数
# def test_ik_fk_solver():
#     rclpy.init()
#     solver = MoveItIKFKSolver()
    
#     try:
#         print("=" * 60)
#         print("测试正运动学 (Forward Kinematics)")
#         print("=" * 60)
        
#         # 测试正运动学 - 使用一些典型的关节配置
#         test_joint_configs = [
#             [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0],  # 初始位置
#             [0.00017970632095981137,
#             0.07772538318352297,
#             -8.964913636211132e-05,
#             -2.7266952989127784,
#             0.0026599775134408766,
#             2.801719360336195,
#             0.7826196858485756,]
#         ]
        
#         for i, joints in enumerate(test_joint_configs):
#             print(f"\n--- 测试关节配置 {i+1} ---")
#             print(f"关节角度: {[f'{angle:.3f}' for angle in joints]}")
            
#             try:
#                 start_time = time.perf_counter()
#                 # 只计算末端执行器位置
#                 end_effector_pose = solver.forward_kinematics(joints)
#                 end_time = time.perf_counter()
#                 execution_time = end_time - start_time
                
#                 print(f"FK执行时间: {execution_time:.4f} 秒")
                
#                 if "fr3_hand" in end_effector_pose:
#                     pos = end_effector_pose["fr3_hand"]["position"]
#                     ori = end_effector_pose["fr3_hand"]["orientation"]
#                     print(f"末端执行器位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
#                     print(f"末端执行器方向: [{ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f}, {ori[3]:.3f}]")
                
#             except Exception as e:
#                 print(f"FK求解失败: {e}")
        
#         print("\n" + "=" * 60)
#         print("测试末端执行器位置获取")
#         print("=" * 60)
        
#         # 测试专门的末端执行器位置获取方法
#         for i, joints in enumerate(test_joint_configs):
#             print(f"\n--- 末端执行器位置测试 {i+1} ---")
#             print(f"关节角度: {[f'{angle:.3f}' for angle in joints]}")
            
#             try:
#                 # 获取完整的位置和方向
#                 start_time = time.perf_counter()
#                 ee_pos, ee_ori = solver.get_end_effector_position(joints)
#                 end_time = time.perf_counter()
#                 execution_time = end_time - start_time
                
#                 print(f"执行时间: {execution_time:.4f} 秒")
#                 print(f"末端执行器位置: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
#                 print(f"末端执行器方向: [{ee_ori[0]:.3f}, {ee_ori[1]:.3f}, {ee_ori[2]:.3f}, {ee_ori[3]:.3f}]")
                
#             except Exception as e:
#                 print(f"末端执行器位置获取失败: {e}")

#         print("\n" + "=" * 60)
#         print("测试逆运动学 (Inverse Kinematics)")
#         print("=" * 60)
        
#         # 测试逆运动学
#         test_positions = [
#             [0.4, 0, 0.2]  # 前方上方

#         ]
        
#         for i, pos in enumerate(test_positions):
#             print(f"\n--- 测试目标位置 {i+1} ---")
#             print(f"目标位置: {pos}")
#             try:
#                 start_time = time.perf_counter()
#                 joint_angles = solver.inverse_kinematics(pos)
#                 end_time = time.perf_counter()
#                 execution_time = end_time - start_time
                
#                 print(f"IK执行时间: {execution_time:.4f} 秒")
#                 print(f"求解的关节角度: {[f'{angle:.3f}' for angle in joint_angles]}")
                
#                 # 验证：使用求解的关节角度进行正运动学验证
#                 print("\n验证求解结果...")
#                 fk_result = solver.forward_kinematics(joint_angles)
#                 if "fr3_hand" in fk_result:
#                     calculated_pos = fk_result["fr3_hand"]["position"]
#                     error = np.linalg.norm(np.array(pos) - np.array(calculated_pos))
#                     print(f"计算得到的位置: [{calculated_pos[0]:.3f}, {calculated_pos[1]:.3f}, {calculated_pos[2]:.3f}]")
#                     print(f"位置误差: {error:.6f} m")
                    
#                     if error < 0.001:  # 1mm误差
#                         print("✅ 验证通过！")
#                     else:
#                         print("⚠️  验证失败，误差较大")
                
#             except Exception as e:
#                 print(f"求解失败: {e}")
        
#         print("\n" + "=" * 60)
#         print("测试多链接位置获取")
#         print("=" * 60)
        
#         # 测试获取多个链接位置
#         test_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
#         print(f"关节配置: {[f'{angle:.3f}' for angle in test_joints]}")
        
#         try:
#             all_links = solver.get_multiple_link_positions(test_joints)
#             print("\n所有链接位置:")
#             for link_name, data in all_links.items():
#                 pos = data['position']
#                 print(f"  {link_name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
#         except Exception as e:
#             print(f"多链接位置获取失败: {e}")
    
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     test_ik_fk_solver()





#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState
from std_msgs.msg import Header
import numpy as np
import time

class MoveItIKFKSolver(Node):
    def __init__(self):
        super().__init__('moveit_ik_fk_solver')
        
        # MoveIt IK Service Client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # MoveIt FK Service Client
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        # Wait for services to be available
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt IK service...')

        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt FK service...')
        
        self.get_logger().info("✅ MoveIt IK/FK Solver is ready")

    def inverse_kinematics(self, world_position, world_orientation=None) -> list:
        """
        Use MoveIt to compute inverse kinematics

        Args:
            world_position: [x, y, z] world position
            world_orientation: [x, y, z, w] quaternion orientation (optional)

        Returns:
            list: joint angles [joint1, joint2, ..., joint7]
        """
        # Create IK request
        request = GetPositionIK.Request()

        # Set target position and orientation
        request.ik_request.group_name = "fr3_arm"  # Robot arm group name
        request.ik_request.robot_state.joint_state.name = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]

        # Current joint state (as initial guess)
        request.ik_request.robot_state.joint_state.position = [
            0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0
        ]

        # Set end effector link
        request.ik_request.ik_link_name = "fr3_hand"

        # Target position
        target_pose = Pose()
        target_pose.position = Point(
            x=float(world_position[0]),
            y=float(world_position[1]),
            z=float(world_position[2])
        )

        # Target orientation
        if world_orientation is None:
            # Default orientation: end effector points down
            target_pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        else:
            target_pose.orientation = Quaternion(
                x=float(world_orientation[0]),
                y=float(world_orientation[1]),
                z=float(world_orientation[2]),
                w=float(world_orientation[3])
            )
        
        request.ik_request.pose_stamped.header.frame_id = "fr3_link0"
        request.ik_request.pose_stamped.pose = target_pose

        # Set timeout and attempts
        request.ik_request.timeout.sec = 5
        # request.ik_request.attempts = 10

        # Call service
        try:
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == 1:  # SUCCESS
                    joint_angles = list(response.solution.joint_state.position)
                    self.get_logger().info(f"✅ IK solution successful: {[f'{angle:.3f}' for angle in joint_angles]}")
                    return joint_angles
                else:
                    error_msg = f"IK solution failed, error code: {response.error_code.val}"
                    self.get_logger().error(f"❌ {error_msg}")
                    raise RuntimeError(error_msg)
            else:
                raise TimeoutError("IK service call timed out")

        except Exception as e:
            self.get_logger().error(f"❌ IK solution exception: {e}")
            raise

    def forward_kinematics(self, joint_angles, link_names=None) -> dict:
        """
        Use MoveIt to compute forward kinematics

        Args:
            joint_angles: [joint1, joint2, ..., joint7] joint angle list
            link_names: List of link names to compute, defaults to end effector

        Returns:
            dict: {link_name: {'position': [x, y, z], 'orientation': [x, y, z, w]}}
        """
        if link_names is None:
            link_names = ["fr3_hand"]  # Default to computing end effector position

        # Create FK request
        request = GetPositionFK.Request()

        # Set robot state
        request.robot_state.joint_state.name = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        request.robot_state.joint_state.position = [float(angle) for angle in joint_angles]

        # Set links to compute
        request.fk_link_names = link_names

        # Set reference frame
        request.header.frame_id = "fr3_link0"

        # Call service
        try:
            future = self.fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == 1:  # SUCCESS
                    results = {}
                    for i, link_name in enumerate(link_names):
                        pose = response.pose_stamped[i].pose
                        results[link_name] = {
                            'position': [pose.position.x, pose.position.y, pose.position.z],
                            'orientation': [pose.orientation.x, pose.orientation.y, 
                                          pose.orientation.z, pose.orientation.w]
                        }

                    self.get_logger().info(f"✅ FK solution successful")
                    for link_name, data in results.items():
                        pos = data['position']
                        self.get_logger().info(f"  {link_name}: position=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

                    return results
                else:
                    error_msg = f"FK solution failed, error code: {response.error_code.val}"
                    self.get_logger().error(f"❌ {error_msg}")
                    raise RuntimeError(error_msg)
            else:
                raise TimeoutError("FK service call timed out")

        except Exception as e:
            self.get_logger().error(f"❌ FK solution exception: {e}")
            raise

    def get_end_effector_position(self, joint_angles) -> tuple:
        """
        Get the world coordinates of the end effector

        Args:
            joint_angles: List of joint angles

        Returns:
            tuple: (position, orientation) 
                   position: [x, y, z] position
                   orientation: [x, y, z, w] quaternion orientation
        """
        try:
            result = self.forward_kinematics(joint_angles, ["fr3_hand"])
            if "fr3_hand" in result:
                position = result["fr3_hand"]["position"]
                orientation = result["fr3_hand"]["orientation"]
                return position, orientation
            else:
                raise RuntimeError("Unable to get end effector position")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to get end effector position: {e}")
            raise


    def get_multiple_link_positions(self, joint_angles) -> dict:
        """
        Get the world coordinates of multiple links

        Args:
            joint_angles: List of joint angles

        Returns:
            dict: Position information of multiple links
        """
        # Common link names
        link_names = [
            "fr3_link1",
            "fr3_link2", 
            "fr3_link3",
            "fr3_link4",
            "fr3_link5",
            "fr3_link6",
            "fr3_link7",
            "fr3_hand"
        ]
        
        return self.forward_kinematics(joint_angles, link_names)

def test_simple_ik_fk():
    rclpy.init()
    solver = MoveItIKFKSolver()

    try:
        print("=" * 30)
        print("👉 Forward Kinematics Test")
        print("=" * 30)

        # joint_angles = [-0.7687153380443489, 0.11247818654052139, 0.1119013495414711, -2.361459772011796, 0.031976763940068646, 2.5149397281779167, 0.05702809129333345]
        joint_angles = [0.7908531189025602, 0.7331863290614418, 0.14616466051488475, -1.5549292373432555, -0.12421715045696818, 2.3260297300625146, 0.24813900307069997]
        


        print(f"Joint angles: {[f'{a:.3f}' for a in joint_angles]}")
        fk_result = solver.forward_kinematics(joint_angles)

        pos = fk_result["fr3_hand"]["position"]
        ori = fk_result["fr3_hand"]["orientation"]
        print(f"End effector position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        print(f"End effector orientation: [{ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f}, {ori[3]:.3f}]")

        print("\n" + "=" * 30)
        print("👉 Inverse Kinematics Test")
        print("=" * 30)

        target_position = [0.554, -0.000, 0.625]
        print(f"Target position: {target_position}")
        ik_result = solver.inverse_kinematics(target_position)
        print(f"Joint angles: {[f'{a:.3f}' for a in ik_result]}")

    except Exception as e:
        print(f"Error occurred during testing: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    test_simple_ik_fk()