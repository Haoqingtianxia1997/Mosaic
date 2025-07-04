import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

def tf_transform_to_matrix(tf_transform):
    """将TransformStamped转换为4x4变换矩阵"""
    t = tf_transform.transform.translation
    q = tf_transform.transform.rotation
    translation = np.array([t.x, t.y, t.z])
    quat = np.array([q.x, q.y, q.z, q.w])
    rot = R.from_quat(quat).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = translation
    return T

class TfVisualizer(Node):
    def __init__(self, extrinsics_T, tf_parent, tf_child):
        super().__init__('tf_visualizer')
        self.extrinsics_T = extrinsics_T  # 外参矩阵
        self.tf_parent = tf_parent        # camera_link
        self.tf_child = tf_child          # camera_optical_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # tf: camera_link -> camera_optical_frame
            tf_transform = self.tf_buffer.lookup_transform(
                self.tf_parent,
                self.tf_child,
                rclpy.time.Time()
            )
            T_tf = tf_transform_to_matrix(tf_transform)

            T_tf_translation = [-0.01, 0.06, 0.0]
            T_tf[:3, 3] += np.array(T_tf_translation)  # 平移调整
            
            
            # 总变换: world->camera_link->camera_optical_frame
            T_total = self.extrinsics_T @ T_tf

            print("Transform matrix:\n", T_total)

            # 可视化
            world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
            cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
            cam_frame.transform(T_total)
            o3d.visualization.draw_geometries([world_frame, cam_frame])
        except Exception as e:
            print("Transform not available: ", e)

def main():
    rclpy.init()

    # # 这里手动填写 extrinsics 外参（world->camera_link）
    # translation = np.array([0.8595406021203789, 0.5037965577635406, 0.5703258113488032])
    # qx = 0.37301026915950647
    # qy = 0.06207915901055871
    # qz = -0.8924183137659721
    # qw = 0.24616878431920047
    # rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
    
    

    # zedl extrinsics
    translation = np.array([0.14287264529286604, -0.5160394374662502, 0.5285747070512808])
    roll = 0.4719635237773772
    pitch = 0.8017128013264066
    yaw = 1.0319360545389382
    rot = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    
    extrinsics_T = np.eye(4)
    extrinsics_T[:3, :3] = rot
    extrinsics_T[:3, 3] = translation

    tf_parent = "zedl_left_camera_frame"  # camera_link
    tf_child = "zedl_left_camera_optical_frame"
    

    node = TfVisualizer(extrinsics_T, tf_parent, tf_child)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
