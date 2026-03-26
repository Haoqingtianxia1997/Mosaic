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
import open3d as o3d
HIGH_LEVEL_PATH = os.path.abspath(os.path.join(__file__, "../../../../../../high_level/src"))
if HIGH_LEVEL_PATH not in sys.path:
    sys.path.append(HIGH_LEVEL_PATH)
from transcribe.tts import play_text_to_speech
from transcribe.stt import VoiceTranscriber
from pixel_world.pixel_and_world import left_cam, right_cam, gaze_cam, world_to_pixels_left, world_to_pixels_right, world_to_pixels_gaze, pixels_to_world_gaze
SRC_PATH = os.path.abspath(os.path.join(__file__, "../../../../"))
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)
from intention.l2cs import select_device, Pipeline



class Intention():
    def __init__(self):
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolo_model/best.pt')
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
        self.SLIDING_WINDOW_SEC = 1.0
        self.OUTLIER_THRESHOLD = 0.05
        self.OUTLIER_COUNT = 15
        self.AVG_LAST_N = 5

        #Yolo confidence
        self.yolo_conf = 0.85
        self.intention_yolo_conf = 0.7
        self.output_dir = './saved_images'
        os.makedirs(self.output_dir, exist_ok=True)

    def get_hand_pose(self, rgb_msg, depth_msg, T_wc, camera_intrinsics):
        """
        camera_intrinsics: (fx, fy, cx, cy)
        T_wc: 4x4 Transformation matrix (world ← camera)
        """
        fx, fy, cx, cy = camera_intrinsics

        rgb = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')  # Single-channel float32 (meters)

        h, w = depth.shape

        img_rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        img_small = cv2.resize(img_rgb, (640, 360))
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_small)
        hands_result = self.hands_detector.detect(mp_image)

        if hands_result.hand_landmarks:
            hand = hands_result.hand_landmarks[0]
        else:
            return None, None

        # get pixel coordinates (MIDDLE_FINGER_MCP=9 as palm center, MIDDLE_FINGER_PIP=10)
        index_mcp = hand[9]
        index_tip = hand[10]

        u1, v1 = int(index_mcp.x * w), int(index_mcp.y * h)
        u2, v2 = int(index_tip.x * w), int(index_tip.y * h)

        # Boundary check
        if not (0 <= u1 < w and 0 <= v1 < h and 0 <= u2 < w and 0 <= v2 < h):
            print(f"⚠️ finger points out of bounds")
            return None, None 

        z1 = float(depth[v1, u1])
        z2 = float(depth[v2, u2])

        if z1 <= 0 or z2 <= 0:
            print(f"⚠️ index finger depth abnormal: z1={z1}, z2={z2}")
            return None, None

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
            print("⚠️ index finger direction length too short")
            return None, None

        direction /= np.linalg.norm(direction)
        print(f"origin:{origin}, direction: {direction}")
        return direction, origin

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

        print(f"aria gaze_dir_world: {gaze_dir_world}, gaze_origin_world: {gaze_origin_world}")
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

    def in_valid_area(self, pt):
        return pt is not None and (0.0 <= pt[0] <= 1.0) and (-0.7 <= pt[1] <= 0.7)

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
    
    def detect_and_draw_yolo(self, img, u, v, yolo_model, output_path, depth=None, camera_intrinsics=None, T_wc=None, ref_world_point=None):
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
                         returns only the label of the closest bbox; otherwise returns all labels.
        """
        h, w = img.shape[:2]
        roi_size = 500
        half = roi_size // 2
        x1, y1 = max(u - half, 0), max(v - half, 0)
        x2, y2 = min(u + half, w-1), min(v + half, h-1)

        # Draw ROI box
        cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,255), 2)
        roi = img[y1:y2, x1:x2]
        labels = []
        if roi.size == 0 or roi.shape[0] < 5 or roi.shape[1] < 5:
            print("ROI empty, skip YOLO")
        else:
            result = self.yolo_model(img, verbose=False, conf=self.intention_yolo_conf)[0]
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
                            z = float(depth[center_y, center_x])
                            if z > 0:
                                xc = (center_x - cx) * z / fx
                                yc = (center_y - cy) * z / fy
                                p_cam = np.array([xc, yc, z, 1.0])
                                xyz = (T_wc @ p_cam)[:3]
                                world_xy = xyz[:2]
                                print(f"  [{label}] center pixel ({center_x},{center_y}) -> world xyz: {xyz}")
                    detections.append((bx1, by1, bx2, by2, label, conf, world_xy))

                # Find nearest bbox to ref_world_point
                if ref_world_point is not None:
                    ref_xy = np.array(ref_world_point[:2])
                    best_idx = None
                    best_dist = float('inf')
                    for i, (_, _, _, _, lbl, _, wxy) in enumerate(detections):
                        if wxy is not None:
                            dist = float(np.linalg.norm(wxy - ref_xy))
                            print(f"  [{lbl}] world xy: {wxy}, dist to ref: {dist:.4f}")
                            if dist < best_dist:
                                best_dist = dist
                                best_idx = i
                    if best_idx is not None:
                        bx1, by1, bx2, by2, best_label, best_conf, _ = detections[best_idx]
                        cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 0, 255), 2)
                        cv2.putText(img, f"{best_label} {best_conf:.2f}", (bx1, by1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        labels = [best_label]
                        print(f"  -> nearest bbox: [{best_label}] (dist={best_dist:.4f})")
                    else:
                        labels = [d[4] for d in detections]
                else:
                    # No ref point: draw and return all
                    for bx1, by1, bx2, by2, lbl, conf, _ in detections:
                        cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 0, 255), 2)
                        cv2.putText(img, f"{lbl} {conf:.2f}", (bx1, by1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        labels.append(lbl)

                print(f"YOLO detected (ROI): {', '.join(labels)}")

        cv2.imwrite(output_path, img)
        print(f"YOLO ROI & label image saved: {output_path}")
        return labels
    
    
    
    
    
    
    
    def get_scenario_yolo_labels(self, img, scenario_img_path=None):
        """
        img: bgr8 (cv2)
        u, v: center point pixel of ROI
        yolo_model: 
        output_path: 
        """
        labels = []
        result = self.yolo_model(img, verbose=False,  conf=self.yolo_conf)[0]
        if result.boxes.shape[0]:
            for box in result.boxes:
                bx1, by1, bx2, by2 = map(int, box.xyxy[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                label = self.yolo_model.names[cls]
                conf = float(box.conf[0].cpu().numpy())
                cv2.rectangle(img, (bx1, by1), (bx2, by2), (0,0,255), 2)
                cv2.putText(img, f"{label} {conf:.2f}", (bx1, by1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                labels.append(label)
            print(f" scenario YOLO detected: {', '.join(labels)}")
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

    def process_detection(self, direction=None, origin=None, rgb_msg=None, pts=None, direction_name=None, origin_name=None, image_name=None, camera_side=None, intersect=None, depth_msg=None, camera_intrinsics=None, T_wc=None):
        
        if intersect is None :     
            # EMA filter
            self.update_ema(direction_name, direction, self.ema_alpha, unit_vector=True)
            self.update_ema(origin_name, origin, self.ema_alpha)
            finger_direction_ema = self.get_ema(direction_name)
            finger_origin_ema = self.get_ema(origin_name)                                      
            # Calculate intersect
            intersect = self.line_plane_intersect(finger_origin_ema, finger_direction_ema, z_plane=0.0)
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
            depth_list = depth_msg if isinstance(depth_msg, list) else [None] * len(rgb_msg)
            intrinsics_list = camera_intrinsics if isinstance(camera_intrinsics, list) else [camera_intrinsics] * len(rgb_msg)
            T_wc_list = T_wc if isinstance(T_wc, list) else [T_wc] * len(rgb_msg)
            if stable is not None:
                for rgb, name, side, dm, intr, twc in zip(rgb_msg, image_name, camera_side, depth_list, intrinsics_list, T_wc_list):
                    if side == "right":
                        print(f"[Stable finger pos R:] {stable}")
                        # 3d to 2d projection
                        pixel, _ = world_to_pixels_right(stable)

                    elif side == "left":
                        print(f"[Stable finger pos L:] {stable}")
                        # 3d to 2d projection
                        pixel, _ = world_to_pixels_left(stable)

                    # elif camera_side == "middle":
                    #     print(f"[Stable finger pos Middle:] {stable}")
                    #     # 3d to 2d projection
                    #     pixel, _ = world_to_pixels_third(stable)

                    u, v = int(round(pixel[0])), int(round(pixel[1]))
                    print(f"Projected pixel: ({u}, {v})")

                    # yolo
                    img = self.bridge.compressed_imgmsg_to_cv2(rgb, 'bgr8')
                    depth_np = self.bridge.imgmsg_to_cv2(dm, 'passthrough') if dm is not None else None
                    labels = self.detect_and_draw_yolo(
                        img, u, v, self.yolo_model, os.path.join(self.output_dir, name),
                        depth=depth_np, camera_intrinsics=intr, T_wc=twc, ref_world_point=stable
                    )
                    all_labels.extend(labels)

                all_labels = list(set(all_labels))
                # # tts and stt
                # self.ask_label_tts(all_labels)
                label_output = all_labels
            else:
                label_output = []

        else:
            if stable is not None:
                if camera_side == "right":
                    print(f"[Stable finger pos R:] {stable}")
                    # 3d to 2d projection
                    pixel, _ = world_to_pixels_right(stable)
                
                elif camera_side == "left":
                    print(f"[Stable finger pos L:] {stable}")
                    # 3d to 2d projection
                    pixel, _ = world_to_pixels_left(stable)
                # elif camera_side == "middle":
                #     print(f"[Stable finger pos Middle:] {stable}")
                #     # 3d to 2d projection
                #     pixel, _ = world_to_pixels_third(stable)

                u, v = int(round(pixel[0])), int(round(pixel[1]))
                print(f"Projected pixel: ({u}, {v})")

                # yolo
                img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, 'bgr8')
                depth_np = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough') if depth_msg is not None else None
                labels = self.detect_and_draw_yolo(
                    img, u, v, self.yolo_model, os.path.join(self.output_dir, image_name),
                    depth=depth_np, camera_intrinsics=camera_intrinsics, T_wc=T_wc, ref_world_point=stable
                )
                label_output = labels
                # # tts and stt
                # self.ask_label_tts(labels)
            else:
                label_output = []
        
        return stable, last_output, pts, finger_base, finger_direction_ema, finger_origin_ema, intersect, label_output

