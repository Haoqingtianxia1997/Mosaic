import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import cv2
from math import radians



# left image load
# depth_img = np.load('./saved_images/l_depth.npy')
# height, width = depth_img.shape
# rgb_img = cv2.cvtColor(cv2.imread('./saved_images/l_rgb.png'), cv2.COLOR_BGR2RGB)

# # zedl cam intrinsics
# fx = 1060.0899658203125
# fy = 1059.0899658203125
# cx = 958.9099731445312
# cy = 561.5670166015625
# # zedl extrinsics
# translation = np.array([0.11261126, -0.50195948, 0.55795671])
# qx = 0.81395177
# qy = -0.40028226
# qz = -0.07631803
# qw = -0.41404371
# roll = -2.38206340
# pitch = 0.47316643
# yaw = -0.72222115

# test_point = np.array([0.3, -0.2, 0.2])



# right image load
depth_img = np.load('./saved_images/r_depth.npy')
height, width = depth_img.shape
rgb_img = cv2.cvtColor(cv2.imread('./saved_images/r_rgb.png'), cv2.COLOR_BGR2RGB)

# zedr cam intrinsics
fx = 1059.9764404296875
fy = 1059.9764404296875
cx = 963.07568359375
cy = 522.3530883789062
# zedr extrinsics
translation = [0.903701253331141, 0.439249176547482, 0.598645500102408]
qx = -0.404974467935380
qy = -0.808551385290863
qz = 0.425767747250020
qw = 0.031018753461827
roll  = -2.298360156012407
pitch = 0.299130700579650
yaw   = 2.347883188101515

test_point = np.array([0.7, 0.0, 0.2])



rot = R.from_euler('xyz', [roll, pitch, yaw])
# rot = R.from_quat([qx, qy, qz, qw])
# 生成像素网格
u, v = np.meshgrid(np.arange(width), np.arange(height))
z = depth_img
x = (u - cx) * z / fx
y = (v - cy) * z / fy

points_cam = np.stack((x, y, z), axis=-1).reshape(-1, 3)
z_flat = z.reshape(-1)
mask = (z_flat > 0) & (z_flat < 5)

points_cam = points_cam[mask]



# extrinsics transformation
T_extrinsics = np.eye(4)
T_extrinsics[:3, :3] = rot.as_matrix()
T_extrinsics[:3, 3] = translation


T_total = T_extrinsics 

# 同质坐标
points_cam_hom = np.concatenate([points_cam, np.ones((points_cam.shape[0], 1))], axis=1)
points_world_hom = (T_total @ points_cam_hom.T).T


# rotation adjust 
# angle_deg = 5
# angle_rad = radians(angle_deg)
# R_y = R.from_euler('x', angle_rad).as_matrix()
# T_y = np.eye(4)
# T_y[:3, :3] = R_y
# points_world_hom = (T_y @ points_world_hom.T).T

# angle_deg = 5
# angle_rad = radians(angle_deg)
# R_y = R.from_euler('y', angle_rad).as_matrix()
# T_y = np.eye(4)
# T_y[:3, :3] = R_y
# points_world_hom = (T_y @ points_world_hom.T).T

# angle_deg = 5
# angle_rad = radians(angle_deg)
# R_y = R.from_euler('z', angle_rad).as_matrix()
# T_y = np.eye(4)
# T_y[:3, :3] = R_y
# points_world_hom = (T_y @ points_world_hom.T).T


points_world = points_world_hom[:, :3]


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
camera_frame.transform(T_total)

# test point position
sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
sphere.translate(test_point)
sphere.paint_uniform_color([1, 0, 0])  # 红色

# 可视化
o3d.visualization.draw_geometries([pcd, world_frame, camera_frame, sphere],
                                  window_name="ZED2 PointCloud with World & Camera Frame")