"""
reproject_to_world.py
=====================
读取 ZED2 的 depth.npy + rgb.png，
把整幅深度图投射成带颜色的点云并可视化。
"""

import numpy as np
import open3d as o3d
import cv2
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

# ---------- 通用相机模型 ----------
@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    R_wc: np.ndarray   # 3×3，摄像机→世界旋转
    t_wc: np.ndarray   # (3,)  摄像机→世界平移

#TODO extrinsics are wrong, need to be fixed
# ---------- 左 / 右 相机参数 ----------
left_cam = CameraModel(
    fx=1060.0899658203125,
    fy=1059.0899658203125,
    cx=958.9099731445312,
    cy=561.5670166015625,
    R_wc=R.from_euler(
        'xyz',
        [0.4719635237773772,
         0.8017128013264066,
         1.0319360545389382]).as_matrix(),
    t_wc=np.array([0.14287264529286604,
                   -0.5160394374662502,
                    0.5285747070512808])
)

right_cam = CameraModel(
    fx=1059.9764404296875,
    fy=1059.9764404296875,
    cx=963.07568359375,
    cy=522.3530883789062,
    R_wc=R.from_quat([
        0.37301026915950647,
        0.06207915901055871,
       -0.8924183137659721,
        0.24616878431920047]).as_matrix(),
    t_wc=np.array([0.8595406021203789,
                   0.5037965577635406,
                   0.5703258113488032])
)

# ---------- 像素 → 世界 ----------
def pixels_to_world(pixels, depths, cam: CameraModel, rgb_img=None):
    """
    pixels : (..., 2)  (u, v)
    depths : (...)     单位米
    rgb_img: H×W×3 (uint8, RGB)，若给则返回颜色
    """
    px = np.asarray(pixels)[..., 0]
    py = np.asarray(pixels)[..., 1]
    z  = np.asarray(depths)

    # 像素 → 相机坐标
    x = (px - cam.cx) * z / cam.fx
    y = (py - cam.cy) * z / cam.fy
    cam_pts = np.stack((x, y, z), axis=-1)       # (..., 3)

    # 相机 → 世界
    world_pts = cam_pts @ cam.R_wc.T + cam.t_wc   # (..., 3)

    if rgb_img is not None:
        h, w, _ = rgb_img.shape
        u = np.clip(px.astype(int), 0, w - 1)
        v = np.clip(py.astype(int), 0, h - 1)
        colors = rgb_img[v, u] / 255.0            # (..., 3)
        return world_pts, colors

    return world_pts, None



def world_to_pixels(points_world, cam: CameraModel, return_depth=False,
                    image_size: tuple[int, int] = None):
    """
    将世界坐标点投影回像素坐标

    参数:
        points_world : (..., 3) 世界坐标点
        cam : CameraModel
        return_depth : 是否返回相机坐标系下的 z 值
        image_size : (宽, 高)，用于检测越界。若为 None，则不检查

    返回:
        pixels : (..., 2) 像素坐标 (u, v)
        [可选] depth_z : (...) 相机坐标系下的深度
    """
    pw = np.asarray(points_world)
    pc = (pw - cam.t_wc) @ cam.R_wc  # 世界 → 相机

    x, y, z = pc[..., 0], pc[..., 1], pc[..., 2]
    z = np.where(z < 1e-6, 1e-6, z)

    u = cam.fx * x / z + cam.cx
    v = cam.fy * y / z + cam.cy
    pixels = np.stack((u, v), axis=-1)

    if image_size is not None:
        w, h = image_size
        if np.any((u < 0) | (u >= w) | (v < 0) | (v >= h)):
            return None

    if return_depth:
        return pixels, z
    return pixels, None


# 包装：左右相机
def pixels_to_world_left(pixels, depths, rgb_img=None):
    return pixels_to_world(pixels, depths, left_cam, rgb_img)

def pixels_to_world_right(pixels, depths, rgb_img=None):
    return pixels_to_world(pixels, depths, right_cam, rgb_img)

def world_to_pixels_left(points_world, return_depth=False):
    return world_to_pixels(points_world, left_cam, return_depth)

def world_to_pixels_right(points_world, return_depth=False):
    return world_to_pixels(points_world, right_cam, return_depth)