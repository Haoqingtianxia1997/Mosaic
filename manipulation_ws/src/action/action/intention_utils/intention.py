from cv_bridge import CvBridge
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.tasks.python import BaseOptions
import time
from ultralytics import YOLO
import os
import sys
import json
import open3d as o3d
HIGH_LEVEL_PATH = os.path.abspath(os.path.join(__file__, "../../../../../../high_level/src"))
if HIGH_LEVEL_PATH not in sys.path:
    sys.path.append(HIGH_LEVEL_PATH)
from transcribe.tts import play_text_to_speech
from transcribe.stt import VoiceTranscriber
from pixel_world.pixel_and_world import left_cam, right_cam, gaze_cam, world_to_pixels_left, world_to_pixels_right, world_to_pixels_realsense, world_to_pixels_gaze, pixels_to_world_gaze
SRC_PATH = os.path.abspath(os.path.join(__file__, "../../../../"))
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)
from intention.l2cs import select_device, Pipeline



class Intention():
    def __init__(self):
        self.bridge = CvBridge()
        self.yolo_model = YOLO(os.path.join(os.path.dirname(__file__), '../../../../../high_level/yolo_model/mixed_zed_realsense.pt'))
        # self.transcriber = VoiceTranscriber()

        models_dir = os.path.join(os.path.dirname(__file__), 'models')
        hand_model_path = os.path.join(models_dir, 'hand_landmarker.task')
        face_model_path = os.path.join(models_dir, 'face_landmarker.task')

        hand_options = mp_vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=hand_model_path),
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.hands_detector = mp_vision.HandLandmarker.create_from_options(hand_options)

        face_options = mp_vision.FaceLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=face_model_path),
            running_mode=mp_vision.RunningMode.IMAGE,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.face_mesh = mp_vision.FaceLandmarker.create_from_options(face_options)
        
        self.gaze_pipeline = Pipeline(
            weights="src/intention/models/L2CSNet_gaze360.pkl",
            arch="ResNet50",
            device=select_device("0", batch_size=1)
        )

        self.ema = {}
        self.ema_alpha = 0.9
        self.first_gaze = True
        self.pitch = None
        self.yaw = None

        # stable point pos
        self.SLIDING_WINDOW_SEC = 0.5
        self.OUTLIER_THRESHOLD = 0.05
        self.OUTLIER_COUNT = 15
        self.AVG_LAST_N = 5

        #Yolo confidence
        self.yolo_conf = 0.80
        self.intention_yolo_conf = 0.80
        self.mask_labels = {"beer bottle", "mayonnaise bottle", "oil bottle", "water bottle"}
        self.gaussian_sigma_deg = 20.0   # half-cone width for Gaussian pointing score
        self.output_dir = './saved_images'
        os.makedirs(self.output_dir, exist_ok=True)

    def get_hand_pose(self, rgb_msg, depth_msg, T_wc, camera_intrinsics):
        """
        camera_intrinsics: (fx, fy, cx, cy)
        T_wc: 4x4 Transformation matrix (world ← camera)
        """
        fx, fy, cx, cy = camera_intrinsics

        from sensor_msgs.msg import CompressedImage as _CompressedImage
        if isinstance(rgb_msg, _CompressedImage):
            rgb = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
        else:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')  # Single-channel float32 (meters)

        h, w = depth.shape

        img_rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
        
        hands_result = self.hands_detector.detect(mp_image)
        drawn_img_bgr = None
        # Draw hand landmarks on the image for visualization
        if hands_result.hand_landmarks:
            hand = hands_result.hand_landmarks[0]
            try:
                # Convert normalized landmarks to pixel coordinates for drawing
                drawn_img = img_rgb.copy()
                for idx, landmark in enumerate(hand):
                    x_px = int(landmark.x * w)
                    y_px = int(landmark.y * h)
                    cv2.circle(drawn_img, (x_px, y_px), 4, (0, 255, 0), -1)
                # Optionally, draw connections (for MediaPipe Hands, see HAND_CONNECTIONS)
                HAND_CONNECTIONS = [
                    (0,1),(1,2),(2,3),(3,4),      # Thumb
                    (0,5),(5,6),(6,7),(7,8),      # Index
                    (5,9),(9,10),(10,11),(11,12), # Middle
                    (9,13),(13,14),(14,15),(15,16), # Ring
                    (13,17),(17,18),(18,19),(19,20), # Pinky
                    (0,17)
                ]
                for start, end in HAND_CONNECTIONS:
                    x1 = int(hand[start].x * w)
                    y1 = int(hand[start].y * h)
                    x2 = int(hand[end].x * w)
                    y2 = int(hand[end].y * h)
                    cv2.line(drawn_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                drawn_img_bgr = cv2.cvtColor(drawn_img, cv2.COLOR_RGB2BGR)
            except Exception as e:
                print(f"Hand landmark drawing error: {e}")
        else:
            return None, None, None, None, None

        # get pixel coordinates (INDEX_FINGER_MCP=5, INDEX_FINGER_TIP=8)
        index_mcp = hand[0]
        index_tip = hand[12]

        u1, v1 = int(index_mcp.x * w), int(index_mcp.y * h)
        u2, v2 = int(index_tip.x * w), int(index_tip.y * h)

        # Boundary check
        if not (0 <= u1 < w and 0 <= v1 < h and 0 <= u2 < w and 0 <= v2 < h):
            print(f"⚠️ finger points out of bounds")
            return None, None, None, None, None

        
        #  =======================for realsense
        z1 = float(depth[v1, u1]) * 0.001  # Convert mm to meters if needed
        z2 = float(depth[v2, u2]) * 0.001 # Convert mm to meters if depth is in millimeters
        # ========================for zed
        # z1 = float(depth[v1, u1])
        # z2 = float(depth[v2, u2])
        
        if z1 <= 0 or z2 <= 0:
            # print(f"⚠️ index finger depth abnormal: z1={z1}, z2={z2}")
            return None, None, None, None, None

        # Camera coordinate system back-projection
        x1 = (u1 - cx) * z1 / fx
        y1 = (v1 - cy) * z1 / fy
        p1_cam = np.array([x1, y1, z1, 1.0])

        x2 = (u2 - cx) * z2 / fx
        y2 = (v2 - cy) * z2 / fy
        p2_cam = np.array([x2, y2, z2, 1.0])

        # Transform to world coordinate system
        origin = (T_wc @ p1_cam)[:3]
        tip = (T_wc @ p2_cam)[:3]

        direction = tip - origin
        if np.linalg.norm(direction) < 1e-6:
            # print("⚠️ index finger direction length too short")
            return None, None, None

        direction /= np.linalg.norm(direction)
        # tip 作为射线起点传出，避免点云求交时打到手自身
        finger_tip_world = tip

        # 计算所有 21 个骨骼点的世界坐标（用于 RViz marker）
        hand_landmarks_world = []
        for lm in hand:
            u_lm = int(lm.x * w)
            v_lm = int(lm.y * h)
            if 0 <= u_lm < w and 0 <= v_lm < h:
                z_lm = float(depth[v_lm, u_lm]) * 0.001
                if z_lm > 0:
                    xc = (u_lm - cx) * z_lm / fx
                    yc = (v_lm - cy) * z_lm / fy
                    p_world = (T_wc @ np.array([xc, yc, z_lm, 1.0]))[:3]
                    hand_landmarks_world.append(p_world)
                else:
                    hand_landmarks_world.append(None)
            else:
                hand_landmarks_world.append(None)

        # print(f"origin:{origin}, direction: {direction}, tip:{finger_tip_world}")
        return direction, origin, drawn_img_bgr, hand_landmarks_world, finger_tip_world

    def compute_gaze_from_aria(self, pitch, yaw, cam_position, cam_quaternion):
        """
        Compute gaze direction and origin in world frame from Aria gaze euler angles and camera pose.

        Args:
            pitch: gaze pitch angle (radians), from /aria/gaze_euler msg.x
            yaw: gaze yaw angle (radians), from /aria/gaze_euler msg.y
            cam_position: np.array([x, y, z]) camera position in world frame
            cam_quaternion: np.array([x, y, z, w]) camera orientation quaternion

        Returns:
            (gaze_direction_world, gaze_origin_world) as numpy arrays
        """
        cos_pitch = math.cos(pitch)
        dir_cam = np.array([
            cos_pitch * math.sin(yaw),
            -math.sin(pitch),
            cos_pitch * math.cos(yaw),
        ])

        rot = R.from_quat(cam_quaternion).as_matrix()
        gaze_dir_world = rot @ dir_cam
        gaze_origin_world = cam_position.copy()

        # print(f"aria gaze_dir_world: {gaze_dir_world}, gaze_origin_world: {gaze_origin_world}")
        return gaze_dir_world, gaze_origin_world

    def line_plane_intersect(self, origin, direction, z_plane=0.0):
        """calculate intersection of a line with xy plane"""
        if abs(direction[2]) < 1e-6:
            return None  # parallel, no intersection
        t = (z_plane - origin[2]) / direction[2]
        if t < 0:
            return None  # intersection point is behind the origin
        p = origin + t * direction
        return p

    def _build_pointcloud_world(self, depth, intrinsics, T_wc, step=1):
        """深度图 → 世界坐标点云（step 降采样，跳过无效深度）。"""
        fx, fy, cx, cy = intrinsics
        h, w = depth.shape[:2]
        u = np.arange(0, w, step)
        v = np.arange(0, h, step)
        uu, vv = np.meshgrid(u, v)
        z = depth[vv, uu].astype(np.float32) * 0.001  # mm → m
        valid = z > 0.1
        if not valid.any():
            return None
        uu, vv, z = uu[valid], vv[valid], z[valid]
        xc = (uu - cx) * z / fx
        yc = (vv - cy) * z / fy
        pts_cam = np.stack([xc, yc, z, np.ones_like(z)], axis=1)  # (N,4)
        pts_world = (T_wc @ pts_cam.T).T[:, :3]                   # (N,3)
        return pts_world

    def ray_pointcloud_intersect(self, origin, direction, points, dist_thresh=0.02, min_t=0.05):
        """射线与世界坐标点云求交。
        origin 应传指尖世界坐标，避免打到手自身。
        min_t: 跳过起点前方 min_t 米内的点（过滤指尖残留噪声）。
        dist_thresh: 点到射线的最大垂直距离（米）。
        无有效点时退回 z=0 平面交点。
        """
        if points is None or len(points) == 0:
            return self.line_plane_intersect(origin, direction)

        diff = points - origin                              # (N,3)
        t = diff @ direction                               # (N,) 沿射线投影
        mask = t > min_t                                   # 跳过起点附近
        if not mask.any():
            return self.line_plane_intersect(origin, direction)

        t_f = t[mask]
        diff_f = diff[mask]
        proj = np.outer(t_f, direction)                    # (M,3)
        perp_dist = np.linalg.norm(diff_f - proj, axis=1) # (M,)

        close = perp_dist < dist_thresh
        if not close.any():
            return self.line_plane_intersect(origin, direction)

        best_t = t_f[close].min()
        return origin + best_t * direction

    def in_valid_area(self, pt):
        return pt is not None and (-1.0 <= pt[0] <= 1.0) and (-2.0 <= pt[1] <= 2.0)

    def _normalize_cos_scores(self, scores):
        """Min-max normalize current-frame cosine scores to [0, 1]."""
        valid_scores = [float(score) for score in scores if score is not None]
        if not valid_scores:
            return [0.0] * len(scores)

        min_score = min(valid_scores)
        max_score = max(valid_scores)
        score_range = max_score - min_score
        if score_range < 1e-6:
            return [1.0 if score is not None else 0.0 for score in scores]

        normalized = []
        for score in scores:
            if score is None:
                normalized.append(0.0)
            else:
                normalized.append(float((float(score) - min_score) / score_range))
        return normalized

    def draw_intersect(self, origin, direction, viewer):
        intersect = self.line_plane_intersect(origin, direction, z_plane=0.0)
        if self.in_valid_area(intersect):
            viewer.update_intersect_async(intersect)
        else:
            viewer.update_intersect_async(None)

    def ema_filter(self, value, ema, alpha):
        if ema is None:
            return value
        return alpha * value + (1 - alpha) * ema
    
    def get_ema(self, name):
        return self.ema.get(name, None)
    
    def update_ema(self, name, value, alpha, unit_vector=False):
        ema = self.ema.get(name)
        if ema is None or np.isnan(ema).any() or (unit_vector and np.linalg.norm(ema) < 1e-6):
            res = value.copy()
        else:
            res = alpha * value + (1 - alpha) * ema
            if unit_vector:
                norm = np.linalg.norm(res)
                if norm > 1e-6:
                    res = res / norm
        self.ema[name] = res
        return res

    def update_stable_point(self, pts_deque, intersect, window_sec, avg_last_n, outlier_thresh, outlier_count):
        """
        Stability queue: Pass in deque, current intersection point, window parameters, automatically update, return stable/last_output/queue/base, etc.
        """
        now = time.time()
        stable = None
        last_output = None
        base = None

        if intersect is not None and self.in_valid_area(intersect):
            pts_deque.append((intersect.copy(), now))
            window_duration = now - pts_deque[0][1]
            if window_duration >= window_sec:
                last_N = list(pts_deque)[-min(avg_last_n, len(pts_deque)):]
                mean_pos = np.mean([pt for pt, _ in last_N], axis=0)
                stable = mean_pos
                last_output = mean_pos.copy()
                base = None
            else:
                # Maintain base point and outlier detection
                if len(pts_deque) > 0:
                    base = pts_deque[0][0].copy()
                    outlier_cnt = sum(np.linalg.norm(pt - base) > outlier_thresh for pt, _ in pts_deque)
                    if outlier_cnt >= outlier_count:
                        base = None
            # Clean up expired points
            while pts_deque and now - pts_deque[0][1] > window_sec:
                pts_deque.popleft()
        else:
            pts_deque.clear()
            base = None
            last_output = None
            stable = None
        return stable, last_output, pts_deque, base
    
    # def detect_and_draw_yolo(self, img, u, v, yolo_model, output_path):
    #     """
    #     img: bgr8 (cv2)
    #     u, v: center point pixel of ROI
    #     yolo_model: 
    #     output_path: 
    #     """
    #     h, w = img.shape[:2]
    #     roi_size = 500
    #     half = roi_size // 2
    #     x1, y1 = max(u - half, 0), max(v - half, 0)
    #     x2, y2 = min(u + half, w-1), min(v + half, h-1)

    #     # Draw ROI box
    #     cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,255), 2)
    #     roi = img[y1:y2, x1:x2]
    #     labels = []
    #     if roi.size == 0 or roi.shape[0] < 5 or roi.shape[1] < 5:
    #         print("ROI empty, skip YOLO")
    #     else:
    #         result = yolo_model(roi, verbose=False,  conf=0.05)[0]
    #         if result.boxes.shape[0]:
    #             for box in result.boxes:
    #                 cls = int(box.cls[0].cpu().numpy())
    #                 label = yolo_model.names[cls]
    #                 conf = float(box.conf[0].cpu().numpy())
    #                 bx1, by1, bx2, by2 = map(int, box.xyxy[0].cpu().numpy())
    #                 cv2.rectangle(img, (bx1 + x1, by1 + y1), (bx2 + x1, by2 + y1), (0,0,255), 2)
    #                 cv2.putText(img, f"{label} {conf:.2f}", (bx1 + x1, by1 + y1 - 5),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
    #                 labels.append(label)
    #             print(f"YOLO detected: {', '.join(labels)}")
    #     cv2.imwrite(output_path, img)
    #     print(f"YOLO ROI & label image saved: {output_path}")
    #     return labels
    
    def _format_label_info(self, labels, scores=None):
        if scores is None:
            scores = [None] * len(labels)
        info = []
        for label, score in zip(labels, scores):
            item = {"label": label}
            if score is not None:
                item["score"] = float(score)
            info.append(item)
        info.sort(key=lambda item: item.get("score", float("-inf")), reverse=True)
        return json.dumps(info, ensure_ascii=False)
    
    def _format_label_info_last_time(self, labels, scores=None):
        if scores is None:
            scores = [None] * len(labels)
        info = []
        for label, score in zip(labels, scores):
            item = {"label": label}
            if score is not None:
                item["score"] = float(score)
            info.append(item)
        total = sum(item["score"] for item in info if "score" in item)
        if total > 0:
            for item in info:
                if "score" in item:
                    item["score"] = item["score"] / total
        info.sort(key=lambda item: item.get("score", float("-inf")), reverse=True)
        return json.dumps(info, ensure_ascii=False)
    
    def detect_and_draw_yolo(self, img, u, v, yolo_model, output_path, depth=None, camera_intrinsics=None, T_wc=None, ref_world_point=None, ray_mcp=None, ray_direction=None):
        """
        img: bgr8 (cv2)
        u, v: center point pixel of ROI
        yolo_model:
        output_path:
        depth: depth image (numpy, meters), optional
        camera_intrinsics: (fx, fy, cx, cy), optional
        T_wc: 4x4 world-from-camera transform, optional
        ref_world_point: np.array([x, y, z]) world coordinate of the pointing target (stable),
                         used to find the nearest detected object. If provided with depth/intrinsics/T_wc,
                         returns only the label and score of the closest bbox; otherwise returns all labels
                         with per-label scores. Also returns every valid bbox center in world coordinates.
        """
        h, w = img.shape[:2]
        roi_size = 200
        half = roi_size // 2
        x1, y1 = max(u - half, 0), max(v - half, 0)
        x2, y2 = min(u + half, w-1), min(v + half, h-1)

        roi = img[y1:y2, x1:x2]
        labels = []
        label_scores = []
        bbox_world_points = []
        if roi.size == 0 or roi.shape[0] < 5 or roi.shape[1] < 5:
            # print("ROI empty, skip YOLO")
            pass
        else:
            # Mask out pixels above the line y = 1.5x + 120 (origin upper-left, y downward)
            # and run YOLO only on the remaining area.
            img_for_yolo = img.copy()
            ys = np.arange(h, dtype=np.float32)[:, None]
            xs = np.arange(w, dtype=np.float32)[None, :]
            mask = ys > 1.4 * xs + 80.0
            img_for_yolo[mask] = 0
            # Draw the line on the output image.
            cv2.line(img, (0, 120), (w - 1, int(1.4 * (w - 1) + 80)), (0, 255, 0), 2)

            result = self.yolo_model(img_for_yolo, verbose=False, conf=self.intention_yolo_conf)[0]
            if result.boxes.shape[0]:
                # First pass: collect all bbox info and world XY
                detections = []  # list of (bx1,by1,bx2,by2, label, conf, world_xy)
                for box in result.boxes:
                    bx1, by1, bx2, by2 = map(int, box.xyxy[0].cpu().numpy())
                    center_x = (bx1 + bx2) // 2
                    center_y = (by1 + by2) // 2
                    cls = int(box.cls[0].cpu().numpy())
                    label = yolo_model.names[cls]
                    conf = float(box.conf[0].cpu().numpy())
                    world_xy = None
                    if depth is not None and camera_intrinsics is not None and T_wc is not None:
                        fx, fy, cx, cy = camera_intrinsics
                        dh, dw = depth.shape[:2]
                        if 0 <= center_x < dw and 0 <= center_y < dh:
                            z = float(depth[center_y, center_x]) * 0.001  # mm → m
                            if z > 0:
                                xc = (center_x - cx) * z / fx
                                yc = (center_y - cy) * z / fy
                                p_cam = np.array([xc, yc, z, 1.0])
                                xyz = (T_wc @ p_cam)[:3]
                                xyz[2] *= 0.6
                                world_xy = xyz
                                # print(f"  [{label}] center pixel ({center_x},{center_y}) -> world xyz: {xyz}")
                    detections.append((bx1, by1, bx2, by2, label, conf, world_xy))

                # Mask out unwanted classes and keep only the top-conf detection per label
                best = {}
                for det in detections:
                    lbl, cf = det[4], det[5]
                    if lbl.lower() in self.mask_labels:
                        continue
                    if lbl not in best or cf > best[lbl][5]:
                        best[lbl] = det
                detections = list(best.values())
                bbox_world_points = [d[6] for d in detections if d[6] is not None]

                # Find nearest bbox to ref_world_point
                if ref_world_point is not None:
                    # Project ref_world_point to pixel coords
                    ref_u, ref_v = None, None
                    if camera_intrinsics is not None and T_wc is not None:
                        fx, fy, cx, cy = camera_intrinsics
                        T_cw = np.linalg.inv(T_wc)
                        p_cam = (T_cw @ np.array([ref_world_point[0], ref_world_point[1], ref_world_point[2], 1.0]))[:3]
                        if p_cam[2] > 0:
                            ref_u = int(p_cam[0] * fx / p_cam[2] + cx)
                            ref_v = int(p_cam[1] * fy / p_cam[2] + cy)

                    # Draw all bboxes in blue
                    for bx1, by1, bx2, by2, lbl, cf, _ in detections:
                        cv2.rectangle(img, (bx1, by1), (bx2, by2), (255, 0, 0), 2)
                        cv2.putText(img, f"{lbl} {cf:.2f}", (bx1, by1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        cx_bb, cy_bb = (bx1 + bx2) // 2, (by1 + by2) // 2
                        cv2.circle(img, (cx_bb, cy_bb), 5, (255, 0, 0), -1)

                    # Draw ref point
                    if ref_u is not None and ref_v is not None:
                        cv2.circle(img, (ref_u, ref_v), 8, (0, 255, 0), -1)
                        cv2.putText(img, "ref", (ref_u + 10, ref_v),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # Calculate per-bbox pointing scores and publish all labels above threshold.
                    normalized_scores = [0.0] * len(detections)
                    if ref_u is not None and ref_v is not None:
                        ref_pixel = np.array([ref_u, ref_v])
                        ray_dir_norm = None
                        if ray_direction is not None:
                            nd = np.linalg.norm(ray_direction)
                            if nd > 1e-6:
                                ray_dir_norm = np.array(ray_direction) / nd
                        gaussian_scores = [0.0] * len(detections)
                        for i, (bx1, by1, bx2, by2, lbl, _, world_xyz) in enumerate(detections):
                            labels.append(lbl)
                            cx_bbox = (bx1 + bx2) // 2
                            cy_bbox = (by1 + by2) // 2
                            dist = float(np.linalg.norm(np.array([cx_bbox, cy_bbox]) - ref_pixel))
                            cos_a = None
                            angle_deg = None
                            if ray_dir_norm is not None and ray_mcp is not None and world_xyz is not None:
                                v_to_bbox = np.array(world_xyz) - np.array(ray_mcp)
                                nv = np.linalg.norm(v_to_bbox)
                                if nv > 1e-6:
                                    cos_a = float(np.clip(np.dot(v_to_bbox / nv, ray_dir_norm), -1.0, 1.0))
                                    angle_deg = float(np.degrees(np.arccos(cos_a)))
                                    gaussian_scores[i] = float(np.exp(
                                        -angle_deg ** 2 / (2 * self.gaussian_sigma_deg ** 2)
                                    ))
                        label_scores = gaussian_scores
                else:
                    # No ref point: draw only, return empty labels
                    for bx1, by1, bx2, by2, lbl, conf, _ in detections:
                        cv2.rectangle(img, (bx1, by1), (bx2, by2), (255, 0, 0), 2)
                        cv2.putText(img, f"{lbl} {conf:.2f}", (bx1, by1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        cx_bb, cy_bb = (bx1 + bx2) // 2, (by1 + by2) // 2
                        cv2.circle(img, (cx_bb, cy_bb), 5, (255, 0, 0), -1)

                # print(f"YOLO detected (ROI): {', '.join(labels)} with scores: {label_scores}")

        cv2.imwrite(output_path, img)
        # print(f"YOLO ROI & label image saved: {output_path}")
        self.last_yolo_img = img
        return labels, label_scores, bbox_world_points
    
    
    
    
    
    
    
    def get_scenario_yolo_labels(self, img, scenario_img_path=None):
        """
        img: bgr8 (cv2)
        u, v: center point pixel of ROI
        yolo_model: 
        output_path: 
        """
        labels = []
        h, w = img.shape[:2]
        img_for_yolo = img.copy()
        ys = np.arange(h, dtype=np.float32)[:, None]
        xs = np.arange(w, dtype=np.float32)[None, :]
        mask = ys > 1.4 * xs + 80.0
        img_for_yolo[mask] = 0
        cv2.line(img, (0, 120), (w - 1, int(1.4 * (w - 1) + 80)), (0, 255, 0), 2)
        result = self.yolo_model(img_for_yolo, verbose=False,  conf=self.yolo_conf)[0]
        if result.boxes.shape[0]:
            # Collect all detections, mask unwanted classes, keep top-conf per label
            raw = []
            for box in result.boxes:
                bx1, by1, bx2, by2 = map(int, box.xyxy[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                label = self.yolo_model.names[cls]
                conf = float(box.conf[0].cpu().numpy())
                raw.append((bx1, by1, bx2, by2, label, conf))
            best = {}
            for det in raw:
                lbl, cf = det[4], det[5]
                if lbl.lower() in self.mask_labels:
                    continue
                if lbl not in best or cf > best[lbl][5]:
                    best[lbl] = det
            for bx1, by1, bx2, by2, label, conf in best.values():
                cv2.rectangle(img, (bx1, by1), (bx2, by2), (0,0,255), 2)
                cv2.putText(img, f"{label} {conf:.2f}", (bx1, by1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                labels.append(label)
            # print(f" scenario YOLO detected: {', '.join(labels)}")
        cv2.imwrite(scenario_img_path, img)
        return labels
    
    # def ask_label_tts(self, labels):
    #     labels = list(set(labels))
    #     label_str = ", ".join(labels)
    #     if len(labels) == 0:
    #         last_query_result =""
    #     elif len(labels) == 1:
    #         tts_text = f"Are you looking for {label_str}?"
    #         play_text_to_speech(tts_text, language='en')
    #         stt_text = self.transcriber.auto_record_and_transcribe(5)
    #         print(f"📝 STT Result: {stt_text}")


    #         if stt_text and ("yes" in stt_text.lower() or
    #                         labels[0].lower() in stt_text.lower()):
    #             play_text_to_speech(
    #                 "OK！",
    #                 language='en'
    #             )
    #             last_query_result = f'please give me "{labels[0]}"'
    #         else:
    #             last_query_result =""

    #     else:
    #         tts_text = f"There are {label_str}. What do you want?"
    #         play_text_to_speech(tts_text, language='en')
    #         stt_text = self.transcriber.auto_record_and_transcribe(5)
    #         print(f"📝 STT Result: {stt_text}")

    #         found = ""
    #         if stt_text:
    #             stt_lower = stt_text.lower()
    #             for item in labels:
    #                 if item.lower() in stt_lower:
    #                     found = item
    #                     break
    #         if found:
    #             last_query_result = f'please give me "{found}"'
    #             play_text_to_speech(
    #                 "OK！",
    #                 language='en'
    #             )
    #         else:
    #             last_query_result = ""

    #     print(last_query_result)

    #     TRANS_FILE = os.path.abspath(
    #         os.path.join(os.path.dirname(__file__), '../../../../../high_level/src/transcribe/transcription.txt')
    #     )
    #     with open(TRANS_FILE, "w", encoding="utf-8") as f:
    #         f.write(last_query_result)

    def process_detection(self, direction=None, origin=None, rgb_msg=None, pts=None, direction_name=None, origin_name=None, image_name=None, camera_side=None, intersect=None, depth_msg=None, camera_intrinsics=None, T_wc=None, finger_tip_world=None):
        ray_origin = None
        ray_mcp = None

        if intersect is None:
            if direction is not None and origin is not None:
                # EMA filter
                self.update_ema(direction_name, direction, self.ema_alpha, unit_vector=True)
                self.update_ema(origin_name, origin, self.ema_alpha)
                finger_direction_ema = self.get_ema(direction_name)
                finger_origin_ema = self.get_ema(origin_name)
                ray_mcp = finger_origin_ema
                # 构建点云（支持单张或多张深度图，取第一张有效的）
                pcd_world = None
                dm_list = depth_msg if isinstance(depth_msg, list) else [depth_msg]
                intr_list = camera_intrinsics if isinstance(camera_intrinsics, list) else [camera_intrinsics]
                twc_list = T_wc if isinstance(T_wc, list) else [T_wc]
                for dm, intr, twc in zip(dm_list, intr_list, twc_list):
                    if dm is not None and intr is not None and twc is not None:
                        depth_np = self.bridge.imgmsg_to_cv2(dm, 'passthrough')
                        pcd_world = self._build_pointcloud_world(depth_np, intr, twc)
                        if pcd_world is not None:
                            break
                # 用指尖世界坐标作为射线起点，避免打到手自身点云
                ray_origin = finger_tip_world if finger_tip_world is not None else finger_origin_ema
                intersect = self.ray_pointcloud_intersect(ray_origin, finger_direction_ema, pcd_world)
            else:
                finger_direction_ema = None
                finger_origin_ema = None
                ray_origin = None
        else:
            finger_direction_ema = None
            finger_origin_ema = None
        
        # stable intersect
        stable, last_output, pts, finger_base = self.update_stable_point(
            pts, intersect, self.SLIDING_WINDOW_SEC, self.AVG_LAST_N,
            self.OUTLIER_THRESHOLD, self.OUTLIER_COUNT
        )
        if isinstance(rgb_msg, list) and isinstance(image_name, list) and isinstance(camera_side, list):
            all_labels = []
            all_label_scores = []
            all_bbox_world_points = []
            depth_list = depth_msg if isinstance(depth_msg, list) else [None] * len(rgb_msg)
            intrinsics_list = camera_intrinsics if isinstance(camera_intrinsics, list) else [camera_intrinsics] * len(rgb_msg)
            T_wc_list = T_wc if isinstance(T_wc, list) else [T_wc] * len(rgb_msg)
            for rgb, name, side, dm, intr, twc in zip(rgb_msg, image_name, camera_side, depth_list, intrinsics_list, T_wc_list):
                from sensor_msgs.msg import CompressedImage as _CI
                if isinstance(rgb, _CI):
                    img = self.bridge.compressed_imgmsg_to_cv2(rgb, 'bgr8')
                else:
                    img = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
                depth_np = self.bridge.imgmsg_to_cv2(dm, 'passthrough') if dm is not None else None
                ref_world = stable if stable is not None else intersect
                if ref_world is not None:
                    if side == "right":
                        # print(f"[Stable finger pos R:] {ref_world}")
                        pixel, _ = world_to_pixels_realsense(ref_world)
                    elif side == "left":
                        # print(f"[Stable finger pos L:] {ref_world}")
                        pixel, _ = world_to_pixels_left(ref_world)
                    u, v = int(round(pixel[0])), int(round(pixel[1]))
                    # print(f"Projected pixel: ({u}, {v})")
                    ref_pt = ref_world
                else:
                    h_img, w_img = img.shape[:2]
                    u, v = w_img // 2, h_img // 2
                    ref_pt = None
                labels, label_scores, bbox_world_points = self.detect_and_draw_yolo(
                    img, u, v, self.yolo_model, os.path.join(self.output_dir, name),
                    depth=depth_np, camera_intrinsics=intr, T_wc=twc,
                    ref_world_point=ref_pt, ray_mcp=ray_mcp, ray_direction=finger_direction_ema
                )
                all_labels.extend(labels)
                all_label_scores.extend(label_scores)
                all_bbox_world_points.extend(bbox_world_points)

            # # tts and stt
            # self.ask_label_tts(all_labels)
            label_output = all_labels
            score_output = all_label_scores
            bbox_world_output = all_bbox_world_points

        else:
            from sensor_msgs.msg import CompressedImage as _CI
            if isinstance(rgb_msg, _CI):
                img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
            else:
                img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_np = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough') if depth_msg is not None else None
            ref_world = stable if stable is not None else intersect
            if ref_world is not None:
                if camera_side == "right":
                    # print(f"[Stable finger pos R:] {ref_world}")
                    pixel, _ = world_to_pixels_realsense(ref_world)
                elif camera_side == "left":
                    # print(f"[Stable finger pos L:] {ref_world}")
                    pixel, _ = world_to_pixels_left(ref_world)
                elif camera_side == "realsense":
                    # print(f"[Stable finger pos RealSense:] {ref_world}")
                    pixel, _ = world_to_pixels_realsense(ref_world)
                u, v = int(round(pixel[0])), int(round(pixel[1]))
                # print(f"Projected pixel: ({u}, {v})")
                ref_pt = ref_world
            else:
                h_img, w_img = img.shape[:2]
                u, v = w_img // 2, h_img // 2
                ref_pt = None
            labels, label_scores, bbox_world_points = self.detect_and_draw_yolo(
                img, u, v, self.yolo_model, os.path.join(self.output_dir, image_name),
                depth=depth_np, camera_intrinsics=camera_intrinsics, T_wc=T_wc,
                ref_world_point=ref_pt, ray_mcp=ray_mcp, ray_direction=finger_direction_ema
            )
            label_output = labels
            score_output = label_scores
            bbox_world_output = bbox_world_points
            # # tts and stt
            # self.ask_label_tts(labels)
        
        return stable, last_output, pts, finger_base, finger_direction_ema, finger_origin_ema, intersect, label_output, score_output, bbox_world_output
