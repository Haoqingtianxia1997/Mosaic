import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import cv2

# 读取深度图（单位：米）
depth_img = np.load('./saved_images/r_depth.npy')
height, width = depth_img.shape

# 读取RGB图
rgb_img = cv2.cvtColor(cv2.imread('./saved_images/r_rgb.png'), cv2.COLOR_BGR2RGB)

# # zedl cam intrinsics
# fx = 1060.0899658203125
# fy = 1059.0899658203125
# cx = 958.9099731445312
# cy = 561.5670166015625
# # zedl extrinsics
# translation = np.array([0.14287264529286604, -0.5160394374662502, 0.5285747070512808])
# roll = 0.4719635237773772
# pitch = 0.8017128013264066
# yaw = 1.0319360545389382
# rot = R.from_euler('xyz', [roll, pitch, yaw])

# zedr cam intrinsics
fx = 1059.9764404296875
fy = 1059.9764404296875
cx = 963.07568359375
cy = 522.3530883789062
# zedr extrinsics
translation = np.array([0.8595406021203789, 0.5037965577635406, 0.5703258113488032])
qx = 0.37301026915950647
qy = 0.06207915901055871
qz = -0.8924183137659721
qw = 0.24616878431920047
rot = R.from_quat([qx, qy, qz, qw])

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


# link to center transformation
link_to_center_translation = np.array([0.0, 0.0, 0.015])
link_to_center_quaternion = [0.0, 0.024997395914712332, 0.0, 0.9996875162757025]  # [x, y, z, w]
link_to_center_rot = R.from_quat(link_to_center_quaternion)
link_to_center_rot_matrix = link_to_center_rot.as_matrix()  # 3x3
T_link_to_center = np.eye(4)
T_link_to_center[:3, :3] = link_to_center_rot_matrix
T_link_to_center[:3, 3] = link_to_center_translation


# center to left cam transformation
center_to_left_cam_translation = np.array([-0.01, 0.06, 0.0])
center_to_left_cam_quat = [0.0, 0.0, 0.0, 1.0]
center_to_left_cam_rot = R.from_quat(center_to_left_cam_quat)
T_center_to_left = np.eye(4)
T_center_to_left[:3, :3] = center_to_left_cam_rot.as_matrix()
T_center_to_left[:3, 3] = center_to_left_cam_translation


# left cam to optical frame transformation
left_cam_to_optical_translation = np.array([0.0, 0.0, 0.0])
left_cam_to_optical_quat = [0.5, -0.5, 0.5, -0.5]
left_cam_to_optical_rot = R.from_quat(left_cam_to_optical_quat)
T_left_cam_to_optical = np.eye(4)
T_left_cam_to_optical[:3, :3] = left_cam_to_optical_rot.as_matrix()
T_left_cam_to_optical[:3, 3] = left_cam_to_optical_translation


# 总变换：world <- camera_link <- camera_optical_frame
T_total = T_extrinsics @ T_link_to_center @ T_center_to_left @ T_left_cam_to_optical
# T_total = T_left_cam_to_optical @ T_center_to_left @ T_link_to_center @ T_extrinsics

# print(T_left_cam_to_optical)
# print(T_center_to_left)
# print(T_left_cam_to_optical @ T_center_to_left)
# print(T_left_cam_to_optical @ T_center_to_left @ T_link_to_center)

# print(T_link_to_center @ T_center_to_left @ T_left_cam_to_optical)
# print(T_link_to_center)
# print(T_extrinsics)
# print(T_total)

# # center to right cam transformation
# center_to_right_cam_translation = np.array([-0.01, -0.06, 0.0])
# center_to_right_cam_quat = [0.0, 0.0, 0.0, 1.0]
# center_to_right_cam_rot = R.from_quat(center_to_right_cam_quat)
# T_center_to_right = np.eye(4)
# T_center_to_right[:3, :3] = center_to_right_cam_rot.as_matrix()
# T_center_to_right[:3, 3] = center_to_right_cam_translation


# # right cam to optical frame transformation
# right_cam_to_optical_translation = np.array([0.0, 0.0, 0.0])
# right_cam_to_optical_quat = [0.5, -0.5, 0.5, -0.5]
# right_cam_to_optical_rot = R.from_quat(right_cam_to_optical_quat)
# T_right_cam_to_optical = np.eye(4)
# T_right_cam_to_optical[:3, :3] = right_cam_to_optical_rot.as_matrix()
# T_right_cam_to_optical[:3, 3] = right_cam_to_optical_translation


# # 总变换：world <- camera_link <- camera_optical_frame
# T_total = T_extrinsics @ T_link_to_center @ T_center_to_right @ T_right_cam_to_optical

# 同质坐标
points_cam_hom = np.concatenate([points_cam, np.ones((points_cam.shape[0], 1))], axis=1)
points_world_hom = (T_total @ points_cam_hom.T).T
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


camera_frame1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
camera_frame1.transform(T_extrinsics @ T_link_to_center @ T_center_to_left)
T_link_to_center @ T_center_to_left @ T_left_cam_to_optical

camera_frame2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
camera_frame2.transform(T_extrinsics @ T_link_to_center)

camera_frame3 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
camera_frame3.transform(T_extrinsics)

# 可视化
o3d.visualization.draw_geometries([pcd, world_frame, camera_frame, camera_frame1, camera_frame2, camera_frame3],
                                  window_name="ZED2 PointCloud with World & Camera Frame")