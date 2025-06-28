import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import cv2

# 读取深度图（单位：米）
depth_img = np.load('./saved_images/l_depth.npy')
height, width = depth_img.shape

# 读取RGB图
rgb_img = cv2.cvtColor(cv2.imread('./saved_images/l_rgb.png'), cv2.COLOR_BGR2RGB)

# 相机内参
fx = 1060.0899658203125
fy = 1059.0899658203125
cx = 958.9099731445312
cy = 561.5670166015625

# 外参
translation = np.array([0.14287264529286604, -0.5160394374662502, 0.5285747070512808])
roll = 0.4719635237773772
pitch = 0.8017128013264066
yaw = 1.0319360545389382

# 生成像素网格
u, v = np.meshgrid(np.arange(width), np.arange(height))
z = depth_img
x = (u - cx) * z / fx
y = (v - cy) * z / fy

points_cam = np.stack((x, y, z), axis=-1).reshape(-1, 3)
z_flat = z.reshape(-1)
mask = (z_flat > 0) & (z_flat < 5)

points_cam = points_cam[mask]

# 变换到世界坐标系
rot = R.from_euler('xyz', [roll, pitch, yaw])
R_mat = rot.as_matrix()
points_world = np.dot(points_cam, R_mat.T) + translation

# 点云上色
rgb_flat = rgb_img.reshape(-1, 3)
colors = rgb_flat[mask] / 255.0

# 创建点云对象
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_world)
pcd.colors = o3d.utility.Vector3dVector(colors)

# 创建世界坐标系原点
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])

# 创建相机坐标系原点
camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
# 构造4x4变换矩阵
T = np.eye(4)
T[:3, :3] = R_mat
T[:3, 3] = translation
camera_frame.transform(T)

# 可视化
o3d.visualization.draw_geometries([pcd, world_frame, camera_frame],
                                  window_name="ZED2 PointCloud with World & Camera Frame")