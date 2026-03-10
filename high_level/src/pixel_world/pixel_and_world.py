"""
reproject_to_world.py
=====================
Read depth.npy + rgb.png from ZED2,
project the entire depth image to a colored point cloud and visualize it.
"""

import numpy as np
import open3d as o3d
import cv2
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

# ---------- Camera intrinsics & extrinsics ----------
@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    R_wc: np.ndarray   # 3×3, camera→world rotation
    t_wc: np.ndarray   # (3,)  camera→world translation

# ---------- Left / Right Camera Parameters ----------
left_cam = CameraModel(
    fx=1060.0899658203125,
    fy=1059.0899658203125,
    cx=958.9099731445312,
    cy=561.5670166015625,
    R_wc=R.from_quat([0.83715593, -0.37772623, 0.15412896, -0.36433106]).as_matrix(),
    t_wc=np.array([0.08738127, -0.45562742,  0.55433431]),
)

right_cam = CameraModel(
    fx=1059.9764404296875,
    fy=1059.9764404296875,
    cx=963.07568359375,
    cy=522.3530883789062,
    R_wc=R.from_quat([0.55712176, 0.74418569, -0.29310264, -0.22336929]).as_matrix(),
    t_wc=np.array([0.862, 0.478207, 0.57065845]),
)

gaze_cam = CameraModel(
    fx = 910.5794677734375,
    fy = 910.3142700195312,
    cx = 643.673583984375,
    cy = 367.935546875,
    R_wc=R.from_quat([
        0.5, 
        -0.5, 
        0.5, 
        -0.5]).as_matrix(),
    t_wc=np.array([0.07261126, 
                   -0.54195948, 
                   0.82295671])
)

# ---------- Pixel to World ----------
def pixels_to_world(pixels, depths, cam: CameraModel, rgb_img=None):
    """
    pixels : (..., 2)  (u, v)
    depths : (...)  meters
    rgb_img: H×W×3 (uint8, RGB), return colors if given
    """
    px = np.asarray(pixels)[..., 0]
    py = np.asarray(pixels)[..., 1]
    z  = np.asarray(depths)

    # pixel → camera
    x = (px - cam.cx) * z / cam.fx
    y = (py - cam.cy) * z / cam.fy
    cam_pts = np.stack((x, y, z), axis=-1)       # (..., 3)

    # camera → world
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
    Project world coordinates back to pixel coordinates

    Args:
        points_world : (..., 3) world coordinates
        cam : CameraModel
        return_depth : whether to return the z value in camera coordinates
        image_size : (width, height), used for boundary checking. If None, no checking is performed

    Returns:
        pixels : (..., 2) pixel coordinates (u, v)
        [Optional] depth_z : (...) depth in camera coordinates
    """
    pw = np.asarray(points_world)
    pc = (pw - cam.t_wc) @ cam.R_wc  # world → camera

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


# Wrapper
def pixels_to_world_left(pixels, depths, rgb_img=None):
    return pixels_to_world(pixels, depths, left_cam, rgb_img)

def pixels_to_world_right(pixels, depths, rgb_img=None):
    return pixels_to_world(pixels, depths, right_cam, rgb_img)

def world_to_pixels_left(points_world, return_depth=False):
    return world_to_pixels(points_world, left_cam, return_depth)

def world_to_pixels_right(points_world, return_depth=False):
    return world_to_pixels(points_world, right_cam, return_depth)

def world_to_pixels_gaze(points_world, return_depth=False):
    return world_to_pixels(points_world, gaze_cam, return_depth)

def pixels_to_world_gaze(pixels, depths, rgb_img=None):
    return pixels_to_world(pixels, depths, gaze_cam, rgb_img)