#!/home/mosaic/miniconda3/envs/mosaic-ros/bin/python

import argparse
import copy
import cv2
import pathlib
import time
import torch
import torch.backends.cudnn as cudnn

import numpy as np
import open3d as o3d
import mediapipe as mp

from collections import deque, Counter
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO

import sys
import os

from src.intention.l2cs import select_device, Pipeline

# 添加 high_level/src 到 sys.path
HIGH_LEVEL_PATH = os.path.abspath(os.path.join(__file__, "../../high_level/src"))
if HIGH_LEVEL_PATH not in sys.path:
    sys.path.append(HIGH_LEVEL_PATH)
from pixel_world.pixel_and_world import left_cam, right_cam, pixels_to_world_left, pixels_to_world_right, world_to_pixels_left, world_to_pixels_right
from transcribe.tts import play_text_to_speech
from transcribe.stt import VoiceTranscriber



CWD = pathlib.Path.cwd()

def parse_args():
    parser = argparse.ArgumentParser(
        description='Gaze + head + finger direction 3D visualization.')
    parser.add_argument('--device', default="0", type=str)
    parser.add_argument('--snapshot', default='models/L2CSNet_gaze360.pkl', type=str)
    parser.add_argument('--cam', default=0, type=int)
    parser.add_argument('--arch', default='ResNet50', type=str)
    parser.add_argument('--save_dir', default='background', type=str, help="Directory to save images")
    parser.add_argument('--l_rgb', default='background/l_rgb.png', type=str)
    parser.add_argument('--l_depth', default='background/l_depth.npy', type=str)
    parser.add_argument('--r_rgb', default='background/r_rgb.png', type=str)
    parser.add_argument('--r_depth', default='background/r_depth.npy', type=str)
    parser.add_argument('--yolo_model_path', default='yolo_model/yolo11m.pt', type=str, help="Path to YOLO model file")
    return parser.parse_args()

    
def euler_to_3d_vector(pitch, yaw):
    x = -np.cos(pitch) * np.sin(yaw)
    y = -np.sin(pitch)
    z = -np.cos(pitch) * np.cos(yaw)
    return np.array([x, y, z])

def create_arrow_mesh(length=1.0):
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.01, cone_radius=0.02,
        cylinder_height=0.8 * length, cone_height=0.2 * length
    )
    return arrow

def align_arrow_to_vector(arrow, vec, origin=np.array([0, 0, 0])):
    vec = vec / np.linalg.norm(vec)
    z_axis = np.array([0, 0, 1])
    v = np.cross(z_axis, vec)
    s = np.linalg.norm(v)
    c = np.dot(z_axis, vec)
    if s < 1e-6:
        R_mat = np.eye(3)
    else:
        vx = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
        R_mat = np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = origin
    arrow.transform(T)
    return arrow

# TODO: replace this with actual camera extrinsics later
def cam_3_extrinsics():
    R_wc = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ])
    T_wc = np.array([0, 0.3, 0.5])
    return R_wc, T_wc

# TODO: replace this with pixels_to_world from pixel_world module later
def camera_to_world_point(origin_c, R_wc, T_wc):
    return R_wc @ origin_c + T_wc

def load_point_cloud(depth_path: str, rgb_path: str, intrinsics: tuple, extrinsics: dict) -> tuple[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh]:
    depth = np.load(depth_path)
    rgb   = cv2.cvtColor(cv2.imread(rgb_path), cv2.COLOR_BGR2RGB)

    fx, fy, cx, cy = intrinsics
    roll, pitch, yaw = extrinsics['euler']
    t = np.asarray(extrinsics['translation'])
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    pts = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    z_flat = z.reshape(-1)
    mask = (z_flat > 0) & (z_flat < 5)
    pts = pts[mask]

    T = np.eye(4)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3]  = t

    pts_h = np.concatenate([pts, np.ones((pts.shape[0], 1))], axis=1)
    pts_w = (T @ pts_h.T).T[:, :3]

    colours = rgb.reshape(-1, 3)[mask] / 255.0

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_w)
    pcd.colors = o3d.utility.Vector3dVector(colours)

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    frame.transform(T)

    return pcd, frame

def camera_params(cam_model):
    """Convert CameraModel to the format expected by load_point_cloud"""
    # Extract intrinsics
    intrinsics = (cam_model.fx, cam_model.fy, cam_model.cx, cam_model.cy)
    
    # Convert rotation matrix to euler angles and extract translation
    from scipy.spatial.transform import Rotation as R
    rot = R.from_matrix(cam_model.R_wc)
    euler_angles = rot.as_euler('xyz')
    
    extrinsics = {
        'euler': euler_angles,
        'translation': cam_model.t_wc.tolist(),
    }
    
    return intrinsics, extrinsics

def load_all_pointclouds(L_DEPTH_PATH, L_RGB_PATH, R_DEPTH_PATH, R_RGB_PATH):
    # ---------- Use camera parameters from pixel_and_world module ----------
    intr_left, extr_left = camera_params(left_cam)
    pcd_left, frame_left = load_point_cloud(
        L_DEPTH_PATH,
        L_RGB_PATH,
        intr_left,
        extr_left,
    )

    intr_right, extr_right = camera_params(right_cam)
    pcd_right, frame_right = load_point_cloud(
        R_DEPTH_PATH,
        R_RGB_PATH,
        intr_right,
        extr_right,
    )

    # ---------- ICP registration (left ➜ right) ----------
    voxel = 0.01  # voxel size for down‑sampling (metres)

    pcd_left_d  = pcd_left.voxel_down_sample(voxel)
    pcd_right_d = pcd_right.voxel_down_sample(voxel)

    pcd_left_d.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30)
    )
    pcd_right_d.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30)
    )

    reg = o3d.pipelines.registration.registration_icp(
        source=pcd_left_d,
        target=pcd_right_d,
        max_correspondence_distance=voxel * 2.0,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )

    # Apply ICP transform to high‑res left cloud & its frame
    pcd_left_icp = copy.deepcopy(pcd_left)
    pcd_left_icp.transform(reg.transformation)

    frame_left_icp = copy.deepcopy(frame_left)
    frame_left_icp.transform(reg.transformation)

    return pcd_right, frame_right, pcd_left_icp, frame_left_icp

def line_plane_intersect(origin, direction, z_plane=0.0):
    """calculate intersection of a line with xy plane"""
    if abs(direction[2]) < 1e-6:
        return None  # parallel, no intersection
    t = (z_plane - origin[2]) / direction[2]
    if t < 0:
        return None  # intersection point is behind the origin
    p = origin + t * direction
    return p

# desk area threshold
def in_valid_area(pt):
    return pt is not None and (0.0 <= pt[0] <= 1.0) and (0.0 <= pt[1] <= 0.7)
    
def draw_point_on_images(
        world_pt,
        tag: str = '',
        roi_size: float = 0.3,             # 正方形ROI的物理边长（米）
        center_color=(0, 0, 255),            # 中心点颜色
        edge_color=(0, 255, 255),            # ROI边框颜色
        save_dir: str = '',
        yolo_model: YOLO = None):
    """
    在左右图各画红点+正方形黄框；用正方形ROI跑YOLO。
    合并左右标签后打印并返回 (label, conf) 或 None。

    world_pt: 世界坐标中的中心点 (x, y, z)
    roi_size: 物理边长（米），正方形ROI的边长
    """

    # 原图复制
    l_detect = l_img_orig.copy()
    r_detect = r_img_orig.copy()

    # 世界 → 像素
    pix_left,  _ = world_to_pixels_left(world_pt)
    pix_right, _ = world_to_pixels_right(world_pt)

    # 用“边长的一半”在X轴上算像素间距，得到像素半边长
    half_size = roi_size / 2

    # 左图
    left_best = None
    if pix_left is not None:
        # 用世界坐标往X正方向偏移，测得像素“半边长”
        world_pt_offset = world_pt + np.array([half_size, 0, 0])
        pix_offset, _ = world_to_pixels_left(world_pt_offset)

        u, v = map(int, np.round(pix_left))
        if pix_offset is not None:
            half_size_px = int(np.linalg.norm(pix_offset - pix_left))
        else:
            half_size_px = 60   # fallback，默认60像素半边长

        # ROI坐标
        x1, y1 = u - half_size_px, v - half_size_px
        x2, y2 = u + half_size_px, v + half_size_px
        h, w = l_img_orig.shape[:2]
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)

        # 绘制中心点和正方形
        cv2.circle(l_detect, (u, v), 6, center_color, -1, cv2.LINE_AA)
        cv2.rectangle(l_detect, (x1, y1), (x2, y2), edge_color, 2)

        roi_l = l_img_orig[y1:y2, x1:x2]

        if roi_l.size:
            res = yolo_model(roi_l, verbose=False)[0]
            if res.boxes.shape[0]:
                idx = res.boxes.conf.cpu().numpy().argmax()
                left_best = (yolo_model.names[int(res.boxes.cls[idx])],
                             float(res.boxes.conf[idx]))
                # 还原bbox到整图坐标
                bx1, by1, bx2, by2 = res.boxes.xyxy[idx].cpu().numpy()
                cv2.rectangle(l_detect,
                              (int(bx1)+x1, int(by1)+y1),
                              (int(bx2)+x1, int(by2)+y1),
                              (0, 0, 255), 2)

    # 右图
    right_best = None
    if pix_right is not None:
        world_pt_offset = world_pt + np.array([half_size, 0, 0])
        pix_offset, _ = world_to_pixels_right(world_pt_offset)

        u, v = map(int, np.round(pix_right))
        if pix_offset is not None:
            half_size_px = int(np.linalg.norm(pix_offset - pix_right))
        else:
            half_size_px = 60

        x1, y1 = u - half_size_px, v - half_size_px
        x2, y2 = u + half_size_px, v + half_size_px
        h, w = r_img_orig.shape[:2]
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)

        cv2.circle(r_detect, (u, v), 6, center_color, -1, cv2.LINE_AA)
        cv2.rectangle(r_detect, (x1, y1), (x2, y2), edge_color, 2)

        roi_r = r_img_orig[y1:y2, x1:x2]

        if roi_r.size:
            res = yolo_model(roi_r, verbose=False)[0]
            if res.boxes.shape[0]:
                idx = res.boxes.conf.cpu().numpy().argmax()
                right_best = (yolo_model.names[int(res.boxes.cls[idx])],
                              float(res.boxes.conf[idx]))
                bx1, by1, bx2, by2 = res.boxes.xyxy[idx].cpu().numpy()
                cv2.rectangle(r_detect,
                              (int(bx1)+x1, int(by1)+y1),
                              (int(bx2)+x1, int(by2)+y2),
                              (0, 0, 255), 2)

    # 保存结果图片
    cv2.imwrite(f"{save_dir}/l_rgb_detect.png", l_detect)
    cv2.imwrite(f"{save_dir}/r_rgb_detect.png", r_detect)

    # 合并左右检测结果
    merged = None
    if left_best and right_best:
        if left_best[0] == right_best[0]:
            merged = (left_best[0], (left_best[1] + right_best[1]) / 2)
        else:
            merged = left_best if left_best[1] >= right_best[1] else right_best
    else:
        merged = left_best or right_best

    if merged:
        print(f"[{tag}] {merged[0]} {merged[1]:.2f}")
    return merged    

def process_gaze(
    frame, vis, gaze_pipeline, R_wc, T_wc, gaze_origin_c,
    gaze_ctx,
    alpha, SLIDING_WINDOW_SEC, OUTLIER_THRESHOLD, OUTLIER_COUNT, AVG_LAST_N,
    save_dir, yolo_model
):
    gaze_label = None

    results = gaze_pipeline.step(frame)
    gaze_intersect = None
    if results.pitch.shape[0] > 0:
        pitch = results.pitch[0]
        yaw = results.yaw[0]
        gaze_vec_c = euler_to_3d_vector(pitch, yaw)
        gaze_vec_w = R_wc @ gaze_vec_c
        gaze_origin_w = camera_to_world_point(gaze_origin_c, R_wc, T_wc)
        gaze_ctx['origin_w'] = gaze_origin_w
        
        # EMA更新
        if gaze_ctx['vec_ema'] is None:
            gaze_ctx['vec_ema'] = gaze_vec_w
        else:
            gaze_ctx['vec_ema'] = alpha * gaze_vec_w + (1 - alpha) * gaze_ctx['vec_ema']
            gaze_ctx['vec_ema'] = gaze_ctx['vec_ema'] / np.linalg.norm(gaze_ctx['vec_ema'])
        gaze_origin_w = camera_to_world_point(gaze_origin_c, R_wc, T_wc)

        new_arrow = create_arrow_mesh()
        new_arrow.paint_uniform_color([1.0, 0.0, 0.0])
        new_arrow = align_arrow_to_vector(new_arrow, gaze_ctx['vec_ema'], origin=gaze_origin_w)
        if gaze_ctx['arrow_visible']:
            vis.remove_geometry(gaze_ctx['arrow'], reset_bounding_box=False)
        vis.add_geometry(new_arrow, reset_bounding_box=False)
        gaze_ctx['arrow'] = new_arrow
        gaze_ctx['arrow_visible'] = True

        # 求交点
        intersect = line_plane_intersect(gaze_origin_w, gaze_ctx['vec_ema'])
        gaze_intersect = intersect

        if in_valid_area(intersect):
            # 球体可视化
            new_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025)
            new_sphere.paint_uniform_color([1.0, 0.0, 0.0])
            new_sphere.translate(intersect)
            if gaze_ctx['sphere'] is not None:
                vis.remove_geometry(gaze_ctx['sphere'], reset_bounding_box=False)
            vis.add_geometry(new_sphere, reset_bounding_box=False)
            gaze_ctx['sphere'] = new_sphere

            # 滑动窗口“稳定点”估计
            now = time.time()
            gaze_ctx['pts'].append((intersect.copy(), now))
            window_duration = now - gaze_ctx['pts'][0][1]
            if window_duration >= SLIDING_WINDOW_SEC:
                last_N = list(gaze_ctx['pts'])[-min(AVG_LAST_N, len(gaze_ctx['pts'])):]
                pos_arr = np.stack([pt for pt, _ in last_N])
                mean_pos = np.mean(pos_arr, axis=0)
                if gaze_ctx['last_output'] is None or np.linalg.norm(mean_pos - gaze_ctx['last_output']) > 1e-5:
                    print(f"[Stable gaze pos:] {mean_pos}")
                    merged = draw_point_on_images(
                        mean_pos, tag='gaze',
                        center_color=(0, 0, 255),
                        save_dir=save_dir, yolo_model=yolo_model
                    )
                    if merged is not None:
                        gaze_label = merged
                    gaze_ctx['last_output'] = mean_pos.copy()
                gaze_ctx['pts'].clear()
                gaze_ctx['base'] = None
            else:
                if gaze_ctx['base'] is None:
                    gaze_ctx['base'] = intersect.copy()
                outlier_count = sum(
                    np.linalg.norm(pt - gaze_ctx['base']) > OUTLIER_THRESHOLD
                    for pt, _ in gaze_ctx['pts']
                )
                if outlier_count >= OUTLIER_COUNT:
                    gaze_ctx['pts'].clear()
                    gaze_ctx['base'] = None
            # 清理过期点
            while gaze_ctx['pts'] and now - gaze_ctx['pts'][0][1] > SLIDING_WINDOW_SEC:
                gaze_ctx['pts'].popleft()
        elif gaze_ctx['sphere'] is not None:
            vis.remove_geometry(gaze_ctx['sphere'], reset_bounding_box=False)
            gaze_ctx['sphere'] = None
            gaze_ctx['pts'].clear()
            gaze_ctx['base'] = None
            gaze_ctx['last_output'] = None
        else:
            gaze_ctx['last_output'] = None
    else:
        # gaze丢失，清理可视化和状态
        if gaze_ctx['arrow_visible']:
            vis.remove_geometry(gaze_ctx['arrow'], reset_bounding_box=False)
            gaze_ctx['arrow_visible'] = False
            gaze_ctx['arrow'] = None
        if gaze_ctx['sphere'] is not None:
            vis.remove_geometry(gaze_ctx['sphere'], reset_bounding_box=False)
            gaze_ctx['sphere'] = None
        gaze_ctx['pts'].clear()
        gaze_ctx['base'] = None
        gaze_ctx['last_output'] = None
        gaze_ctx['vec_ema'] = None

    return gaze_label, gaze_ctx

def process_finger(
    frame, vis, hands, R_wc, T_wc,
    FX, FY, CX, CY,
    finger_ctx,
    alpha, SLIDING_WINDOW_SEC, OUTLIER_THRESHOLD, OUTLIER_COUNT, AVG_LAST_N,
    save_dir, yolo_model
):
    finger_label = None

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hands_results = hands.process(rgb)

    if hands_results.multi_hand_landmarks:
        lm = hands_results.multi_hand_landmarks[0].landmark
        h, w = frame.shape[:2]
        u0, v0 = int(lm[5].x * w), int(lm[5].y * h)
        u1, v1 = int(lm[8].x * w), int(lm[8].y * h)
        z0, z1 = 1.0, 0.95 # TODO: replace with actual depth values

        x0 = (u0 - CX) * z0 / FX
        y0 = (v0 - CY) * z0 / FY
        x1 = (u1 - CX) * z1 / FX
        y1 = (v1 - CY) * z1 / FY

        origin_c = np.array([x0, y0, z0])
        tip_c    = np.array([x1, y1, z1])

        origin_w = R_wc @ origin_c + T_wc
        tip_w    = R_wc @ tip_c    + T_wc
        vec_w    = tip_w - origin_w
        if np.linalg.norm(vec_w) < 1e-6:
            vec_w = np.array([0, 0, 1.0])
        vec_w = vec_w / np.linalg.norm(vec_w)

        # EMA
        if finger_ctx['vec_ema'] is None:
            finger_ctx['vec_ema']    = vec_w.copy()
            finger_ctx['origin_ema'] = origin_w.copy()
        else:
            finger_ctx['vec_ema']    = alpha * vec_w    + (1 - alpha) * finger_ctx['vec_ema']
            finger_ctx['vec_ema']   /= np.linalg.norm(finger_ctx['vec_ema'])
            finger_ctx['origin_ema'] = alpha * origin_w + (1 - alpha) * finger_ctx['origin_ema']

        finger_ctx['origin_w'] = finger_ctx['origin_ema']
        finger_ctx['vec_ema']  = finger_ctx['vec_ema']
        
        # 可视化
        new_ori = o3d.geometry.TriangleMesh.create_sphere(0.02)
        new_ori.paint_uniform_color([1, 0.4, 0])
        new_ori.translate(origin_w)
        if finger_ctx['sphere_ori'] is not None:
            vis.remove_geometry(finger_ctx['sphere_ori'], reset_bounding_box=False)
        vis.add_geometry(new_ori, reset_bounding_box=False)
        finger_ctx['sphere_ori'] = new_ori

        new_tip = o3d.geometry.TriangleMesh.create_sphere(0.02)
        new_tip.paint_uniform_color([0, 0.6, 1])
        new_tip.translate(tip_w)
        if finger_ctx['sphere_tip'] is not None:
            vis.remove_geometry(finger_ctx['sphere_tip'], reset_bounding_box=False)
        vis.add_geometry(new_tip, reset_bounding_box=False)
        finger_ctx['sphere_tip'] = new_tip

        new_arrow = create_arrow_mesh()
        new_arrow.paint_uniform_color([0.0, 0.7, 1.0])
        new_arrow = align_arrow_to_vector(new_arrow, finger_ctx['vec_ema'], origin=finger_ctx['origin_ema'])
        if finger_ctx['arrow_visible']:
            vis.remove_geometry(finger_ctx['arrow'], reset_bounding_box=False)
        vis.add_geometry(new_arrow, reset_bounding_box=False)
        finger_ctx['arrow'] = new_arrow
        finger_ctx['arrow_visible'] = True

        intersect = line_plane_intersect(finger_ctx['origin_ema'], finger_ctx['vec_ema'])
        if in_valid_area(intersect):
            new_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025)
            new_sphere.paint_uniform_color([0.0, 0.7, 1.0])
            new_sphere.translate(intersect)
            if finger_ctx['sphere'] is not None:
                vis.remove_geometry(finger_ctx['sphere'], reset_bounding_box=False)
            vis.add_geometry(new_sphere, reset_bounding_box=False)
            finger_ctx['sphere'] = new_sphere

            now = time.time()
            finger_ctx['pts'].append((intersect.copy(), now))
            window_duration = now - finger_ctx['pts'][0][1]
            if window_duration >= SLIDING_WINDOW_SEC:
                last_N = list(finger_ctx['pts'])[-min(AVG_LAST_N, len(finger_ctx['pts'])):]
                pos_arr = np.stack([pt for pt, _ in last_N])
                mean_pos = np.mean(pos_arr, axis=0)
                if finger_ctx['last_output'] is None or np.linalg.norm(mean_pos - finger_ctx['last_output']) > 1e-5:
                    print(f"[Stable finger pos:] {mean_pos}")
                    merged = draw_point_on_images(
                        mean_pos, tag='finger',
                        center_color=(255, 0, 0),
                        save_dir=save_dir, yolo_model=yolo_model
                    )
                    if merged is not None:
                        finger_label = merged
                    finger_ctx['last_output'] = mean_pos.copy()
                finger_ctx['pts'].clear()
                finger_ctx['base'] = None
            else:
                if finger_ctx['base'] is None:
                    finger_ctx['base'] = intersect.copy()
                outlier_count = sum(
                    np.linalg.norm(pt - finger_ctx['base']) > OUTLIER_THRESHOLD
                    for pt, _ in finger_ctx['pts']
                )
                if outlier_count >= OUTLIER_COUNT:
                    finger_ctx['pts'].clear()
                    finger_ctx['base'] = None
            while finger_ctx['pts'] and now - finger_ctx['pts'][0][1] > SLIDING_WINDOW_SEC:
                finger_ctx['pts'].popleft()
        else:
            if finger_ctx['sphere'] is not None:
                vis.remove_geometry(finger_ctx['sphere'], reset_bounding_box=False)
                finger_ctx['sphere'] = None
            finger_ctx['pts'].clear()
            finger_ctx['base'] = None
            finger_ctx['last_output'] = None
    else:
        # 无手或丢失，清理所有可视化和状态
        if finger_ctx['arrow_visible']:
            vis.remove_geometry(finger_ctx['arrow'], reset_bounding_box=False)
            finger_ctx['arrow_visible'] = False
            finger_ctx['arrow'] = None
        if finger_ctx['sphere'] is not None:
            vis.remove_geometry(finger_ctx['sphere'], reset_bounding_box=False)
            finger_ctx['sphere'] = None
        if finger_ctx['sphere_ori'] is not None: 
            vis.remove_geometry(finger_ctx['sphere_ori'], reset_bounding_box=False)
            finger_ctx['sphere_ori'] = None 
        if finger_ctx['sphere_tip'] is not None:
            vis.remove_geometry(finger_ctx['sphere_tip'], reset_bounding_box=False)
            finger_ctx['sphere_tip'] = None
        finger_ctx['pts'].clear()
        finger_ctx['base'] = None
        finger_ctx['vec_ema'] = None
        finger_ctx['origin_ema'] = None
        finger_ctx['last_output'] = None

    return finger_label, finger_ctx

def process_fusion(
    gaze_ctx, finger_ctx, vis, fusion_ctx,
    alpha, SLIDING_WINDOW_SEC, OUTLIER_THRESHOLD, OUTLIER_COUNT, AVG_LAST_N,
    save_dir, yolo_model
):
    fusion_label = None

    gaze_vec_ema   = gaze_ctx['vec_ema']
    gaze_origin_w  = gaze_ctx['origin_w']
    finger_vec_ema = finger_ctx['vec_ema']
    finger_origin_w= finger_ctx['origin_w']

    if (gaze_vec_ema is not None and finger_vec_ema is not None
        and gaze_origin_w is not None and finger_origin_w is not None):
        fusion_vec = gaze_vec_ema + finger_vec_ema
        fusion_vec = fusion_vec / np.linalg.norm(fusion_vec)
        fusion_origin = (gaze_origin_w + finger_origin_w) / 2

        # EMA更新
        if fusion_ctx['vec_ema'] is None:
            fusion_ctx['vec_ema'] = fusion_vec
            fusion_ctx['origin_ema'] = fusion_origin
        else:
            fusion_ctx['vec_ema']    = alpha * fusion_vec    + (1 - alpha) * fusion_ctx['vec_ema']
            fusion_ctx['vec_ema']   /= np.linalg.norm(fusion_ctx['vec_ema'])
            fusion_ctx['origin_ema'] = alpha * fusion_origin + (1 - alpha) * fusion_ctx['origin_ema']

        # 可视化
        if fusion_ctx['arrow'] is not None:
            vis.remove_geometry(fusion_ctx['arrow'], reset_bounding_box=False)
        new_arrow = create_arrow_mesh()
        new_arrow.paint_uniform_color([0.6, 0.1, 1.0])
        new_arrow = align_arrow_to_vector(new_arrow, fusion_ctx['vec_ema'], origin=fusion_ctx['origin_ema'])
        vis.add_geometry(new_arrow, reset_bounding_box=False)
        fusion_ctx['arrow'] = new_arrow

        fusion_intersect = line_plane_intersect(fusion_ctx['origin_ema'], fusion_ctx['vec_ema'])
        if in_valid_area(fusion_intersect):
            if fusion_ctx['sphere'] is not None:
                vis.remove_geometry(fusion_ctx['sphere'], reset_bounding_box=False)
            new_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025)
            new_sphere.paint_uniform_color([0.6, 0.1, 1.0])
            new_sphere.translate(fusion_intersect)
            vis.add_geometry(new_sphere, reset_bounding_box=False)
            fusion_ctx['sphere'] = new_sphere

            now = time.time()
            fusion_ctx['pts'].append((fusion_intersect.copy(), now))
            window_duration = now - fusion_ctx['pts'][0][1]
            if window_duration >= SLIDING_WINDOW_SEC:
                last_N = list(fusion_ctx['pts'])[-min(AVG_LAST_N, len(fusion_ctx['pts'])):]
                pos_arr = np.stack([pt for pt, _ in last_N])
                mean_pos = np.mean(pos_arr, axis=0)
                if fusion_ctx['last_output'] is None or np.linalg.norm(mean_pos - fusion_ctx['last_output']) > 1e-5:
                    print(f"[Stable fusion pos:] {mean_pos}")
                    merged = draw_point_on_images(
                        mean_pos, tag='fusion',
                        center_color=(180, 0, 255),
                        save_dir=save_dir, yolo_model=yolo_model
                    )
                    if merged is not None:
                        fusion_label = merged
                    fusion_ctx['last_output'] = mean_pos.copy()
                fusion_ctx['pts'].clear()
                fusion_ctx['base'] = None
            else:
                if fusion_ctx['base'] is None:
                    fusion_ctx['base'] = fusion_intersect.copy()
                outlier_count = sum(
                    np.linalg.norm(pt - fusion_ctx['base']) > OUTLIER_THRESHOLD
                    for pt, _ in fusion_ctx['pts']
                )
                if outlier_count >= OUTLIER_COUNT:
                    fusion_ctx['pts'].clear()
                    fusion_ctx['base'] = None
            while fusion_ctx['pts'] and now - fusion_ctx['pts'][0][1] > SLIDING_WINDOW_SEC:
                fusion_ctx['pts'].popleft()
        else:
            if fusion_ctx['sphere'] is not None:
                vis.remove_geometry(fusion_ctx['sphere'], reset_bounding_box=False)
                fusion_ctx['sphere'] = None
            fusion_ctx['pts'].clear()
            fusion_ctx['base'] = None
            fusion_ctx['last_output'] = None
    else:
        if fusion_ctx['arrow'] is not None:
            vis.remove_geometry(fusion_ctx['arrow'], reset_bounding_box=False)
            fusion_ctx['arrow'] = None
        if fusion_ctx['sphere'] is not None:
            vis.remove_geometry(fusion_ctx['sphere'], reset_bounding_box=False)
            fusion_ctx['sphere'] = None
        fusion_ctx['vec_ema'] = None
        fusion_ctx['origin_ema'] = None
        fusion_ctx['pts'].clear()
        fusion_ctx['base'] = None
        fusion_ctx['last_output'] = None

    return fusion_label, fusion_ctx

def process_window_summary(
    line_results, window_key_stat, window_label_stat,
    last_window_time, SLIDING_WINDOW_SEC, last_queried_label
):
    now = time.time()
    frame_keys = list(line_results.keys())
    window_key_stat.append(frame_keys)
    window_label_stat.append(line_results.copy())

    new_window_summary = False
    current_intention_type = None
    current_intention_label = None

    if now - last_window_time >= SLIDING_WINDOW_SEC:
        flat_keys = [k for keys in window_key_stat for k in keys]
        key_counts = Counter(flat_keys)
        total = len(window_key_stat)
        if key_counts and total > 0:
            most_common_key, count = key_counts.most_common(1)[0]
            percent = count / total
            label_counter = Counter([
                frame[most_common_key][0] for frame in window_label_stat if most_common_key in frame
            ])
            most_common_label = label_counter.most_common(1)[0][0] if label_counter else None

            if most_common_label is not None and most_common_label != last_queried_label:
                current_intention_type = most_common_key
                current_intention_label = most_common_label
                new_window_summary = True

            print(f"\n[Window Summary] Intention: {most_common_key} → {most_common_label}, 占比: {percent:.2%}")

        # 清空窗口
        window_key_stat.clear()
        window_label_stat.clear()
        last_window_time = now

    return (
        new_window_summary,
        current_intention_type,
        current_intention_label,
        window_key_stat,
        window_label_stat,
        last_window_time
    )

def process_intention_qa(
    line_results,
    play_text_to_speech,
    transcriber,
    transcription_file="src/transcribe/transcription.txt"
):
    last_query_result = ""
    if line_results:
        current_type, (current_label, current_conf) = list(line_results.items())[0]
        tts_text = f"Are you looking for {current_label}?"
        play_text_to_speech(tts_text, language='en')
        print(f"TTS asked: {tts_text}")

        # 录音 + 识别
        stt_text = transcriber.auto_record_and_transcribe(5)
        print(f"📝 STT Result: {stt_text}")

        if stt_text:
            stt_lower = stt_text.lower()
            if "yes" in stt_lower or current_label.lower() in stt_lower:
                play_text_to_speech("OK!", language='en')
                last_query_result = f'please give me \"{current_label}\"'
            else:
                play_text_to_speech("Please repeat.", language='en')
                last_query_result = ""
        else:
            play_text_to_speech("I didn't catch that. Please try again.", language='en')
            last_query_result = ""

        # 写文件
        with open(transcription_file, "w", encoding="utf-8") as f:
            f.write(last_query_result)
        print(f"last_query_result: {last_query_result}")

    return last_query_result



def main(args=None):
    cudnn.enabled = True
    
    gaze_ctx = dict(
        arrow=None,
        arrow_visible=False,
        sphere=None,
        origin_w=None,
        vec_ema=None,
        pts=deque(),
        base=None,
        last_output=None,
    )

    finger_ctx = dict(
        arrow=None,
        arrow_visible=False,
        sphere=None,
        sphere_ori=None,
        sphere_tip=None,
        origin_w=None,
        vec_ema=None,
        origin_ema=None,
        pts=deque(),
        base=None,
        last_output=None,
    )

    fusion_ctx = dict(
        arrow=None,
        sphere=None,
        vec_ema=None,
        origin_ema=None,
        pts=deque(),
        base=None,
        last_output=None,
    )

    args = parse_args()
    cam = args.cam
    save_dir = args.save_dir
    L_RGB_PATH = args.l_rgb
    L_DEPTH_PATH = args.l_depth
    R_RGB_PATH = args.r_rgb
    R_DEPTH_PATH = args.r_depth
    yolo_model = YOLO(args.yolo_model_path)
    
    l_img_orig = cv2.imread(L_RGB_PATH)
    r_img_orig = cv2.imread(R_RGB_PATH)
    l_detect   = l_img_orig.copy()  # for drawing ROI and bbox
    r_detect   = r_img_orig.copy()

    # ====== 加载点云和坐标系 ======
    pcd_right, frame_right, pcd_left_icp, frame_left_icp = load_all_pointclouds(L_DEPTH_PATH, L_RGB_PATH, R_DEPTH_PATH, R_RGB_PATH)

    gaze_pipeline = Pipeline(
        weights=CWD / 'src' / 'intention' / 'models' / 'L2CSNet_gaze360.pkl',
        arch=args.arch,
        device=select_device(args.device, batch_size=1)
    )

    cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    # === Open3D setup ===
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Intention 3D', width=800, height=600)
    # 世界坐标系（原点[0,0,0]）
    axis_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    vis.add_geometry(axis_world)

    # 两个点云和相机坐标系
    vis.add_geometry(pcd_right)
    vis.add_geometry(pcd_left_icp)
    vis.add_geometry(frame_right)
    vis.add_geometry(frame_left_icp)

    # 你的第三个相机坐标系（就是原来的axis_camera）
    R_wc, T_wc = cam_3_extrinsics()
    axis_camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.10)
    T = np.eye(4)
    T[:3,:3] = R_wc
    T[:3,3] = T_wc
    axis_camera.transform(T)
    vis.add_geometry(axis_camera)

    # 三个箭头在相机坐标系下的原点
    gaze_origin_c = np.array([0.0, 0.0, 1.0])
    head_origin_c = np.array([0.0, 0.1, 1.0])
    finger_origin_c = np.array([-0.2, 0.2, 0.8])

    # === Mediapipe setup ===
    mp_pose = mp.solutions.pose
    mp_hands = mp.solutions.hands
    pose = mp_pose.Pose(static_image_mode=False, model_complexity=1)
    hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1,
                           model_complexity=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)

    arrow_gaze = None
    arrow_gaze_visible = False
    sphere_gaze = None
    gaze_vec_ema = None # Exponential Moving Average for gaze vector
    gaze_pts = deque()  # (point, timestamp)
    gaze_base = None
    gaze_last_output = None
    
    arrow_finger = None
    arrow_finger_visible = False
    sphere_finger = None
    sphere_ori     = None       ### ← new 起点球
    sphere_tip     = None       ### ← new 终点球
    finger_vec_ema = None       # Exponential Moving Average for finger vector
    finger_pts = deque()
    finger_base = None
    finger_last_output = None
    finger_origin_ema = None
    
    arrow_fusion = None
    arrow_fusion_visible = False
    sphere_fusion = None
    fusion_vec_ema = None
    fusion_origin_ema = None
    fusion_pts = deque()
    fusion_base = None
    fusion_last_output = None
    
    alpha = 0.3  # EMA weight
    
    SLIDING_WINDOW_SEC = 2
    OUTLIER_THRESHOLD = 0.3
    OUTLIER_COUNT = 10
    AVG_LAST_N = 5
    
    line_results = {}
    window_key_stat = []    # 新增：存每帧的line_results.keys()
    window_label_stat = []  # 新增：存每帧line_results的value（label部分）
    last_window_time = time.time()  # 上一次统计的时间
    current_intention_label = None  # 你的输出变量
    current_intention_type = None
    
    # transcribe
    transcriber = VoiceTranscriber()
    last_tts_time = 0
    tts_interval = 10           # 两次TTS之间最小间隔（秒），可调节
    last_queried_label = None   # 记录上一次问过的label，防止重复问
    last_query_result = None    # 你的最终结果
    
    # laptop camera intrinsics (arbitrary)
    FX = 1370.0
    FY = 1370.0
    CX = 960.0
    CY = 540.0
    
    while True:
        success, frame = cap.read()
        if not success:
            print("Failed to obtain frame")
            time.sleep(0.1)
            continue
        # ========== 状态变量初始化 ==========
        fusion_label = None
        gaze_label = None
        finger_label = None

        # ========== 1. Gaze detection ==========
        gaze_label, gaze_ctx = process_gaze(
            frame, vis, gaze_pipeline, R_wc, T_wc, gaze_origin_c,
            gaze_ctx, alpha, SLIDING_WINDOW_SEC, OUTLIER_THRESHOLD, OUTLIER_COUNT, AVG_LAST_N,
            save_dir, yolo_model
        )

        # ========== 2. Finger direction ==========
        finger_label, finger_ctx = process_finger(
            frame, vis, hands, R_wc, T_wc,
            FX, FY, CX, CY,
            finger_ctx, alpha, SLIDING_WINDOW_SEC, OUTLIER_THRESHOLD, OUTLIER_COUNT, AVG_LAST_N,
            save_dir, yolo_model
        )

        # ========== 3. Draw direction fusion on images ==========
        fusion_label, fusion_ctx = process_fusion(
            gaze_ctx, finger_ctx, vis, fusion_ctx,
            alpha, SLIDING_WINDOW_SEC, OUTLIER_THRESHOLD, OUTLIER_COUNT, AVG_LAST_N,
            save_dir, yolo_model
        )

        # ===================== 只允许有一条label用于统计 =====================
        line_results.clear()
        if fusion_label is not None:
            line_results['fusion'] = fusion_label
        elif gaze_label is not None:
            line_results['gaze'] = gaze_label
        elif finger_label is not None:
            line_results['finger'] = finger_label

        
        vis.poll_events()

        last_query_result = process_intention_qa(
            line_results,
            play_text_to_speech,
            transcriber,
            transcription_file="src/transcribe/transcription.txt"
        )
        
        
        vis.update_renderer()


if __name__ == '__main__':
    main()


