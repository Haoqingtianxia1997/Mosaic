from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import mediapipe as mp
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

        self.mp_hands = mp.solutions.hands
        self.hands_detector = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        
        # self.play_text_to_speech = play_text_to_speech

        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,  # 更精准瞳孔等
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.gaze_pipeline = Pipeline(
            weights="src/intention/models/L2CSNet_gaze360.pkl",
            arch="ResNet50",
            device=select_device("0", batch_size=1)
        )

        self.ema = {}
        self.ema_alpha = 0.3
        self.first_gaze = True
        self.pitch = None
        self.yaw = None

        # stable point pos
        self.SLIDING_WINDOW_SEC = 3.0
        self.OUTLIER_THRESHOLD = 0.1
        self.OUTLIER_COUNT = 5
        self.AVG_LAST_N = 5

        self.output_dir = './saved_images'
        os.makedirs(self.output_dir, exist_ok=True)

    def get_hand_pose(self, rgb_msg, depth_msg, T_wc, camera_intrinsics):
        """
        camera_intrinsics: (fx, fy, cx, cy)
        T_wc: 4x4 世界坐标系变换矩阵（world ← camera）
        """
        fx, fy, cx, cy = camera_intrinsics

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')  # 单通道 float32 (米)

        h, w = depth.shape

        img_rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        hands_result = self.hands_detector.process(img_rgb)

        if hands_result.multi_hand_landmarks:
            hand = hands_result.multi_hand_landmarks[0]
        else:
            return None, None 

        # get pixel coordinates
        index_mcp = hand.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP]
        index_tip = hand.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]

        u1, v1 = int(index_mcp.x * w), int(index_mcp.y * h)
        u2, v2 = int(index_tip.x * w), int(index_tip.y * h)

        # 边界判断
        if not (0 <= u1 < w and 0 <= v1 < h and 0 <= u2 < w and 0 <= v2 < h):
            print(f"⚠️ finger points out of bounds")
            return None, None 

        z1 = float(depth[v1, u1])
        z2 = float(depth[v2, u2])

        if z1 <= 0 or z2 <= 0:
            print(f"⚠️ 食指关键点深度异常: z1={z1}, z2={z2}")
            return None, None 

        # 相机坐标系中反投影
        x1 = (u1 - cx) * z1 / fx
        y1 = (v1 - cy) * z1 / fy
        p1_cam = np.array([x1, y1, z1, 1.0])

        x2 = (u2 - cx) * z2 / fx
        y2 = (v2 - cy) * z2 / fy
        p2_cam = np.array([x2, y2, z2, 1.0])

        # 变换到世界坐标系
        origin = (T_wc @ p1_cam)[:3]
        tip = (T_wc @ p2_cam)[:3]

        direction = tip - origin
        if np.linalg.norm(direction) < 1e-6:
            print("⚠️ 食指方向长度过短")
            return None, None

        direction /= np.linalg.norm(direction)
        # print(f"origin:{origin}, direction: {direction}")
        return direction, origin

    def get_gaze_direction(self, rgb_msg, depth_msg, T_wc, cam_intrinsics):

        img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w = img.shape[:2]
        fx, fy, cx, cy = cam_intrinsics

        # step1: face mesh检测
        face_result = self.face_mesh.process(img_rgb)
        if not face_result.multi_face_landmarks:
            return None, None
        face_landmarks = face_result.multi_face_landmarks[0]

        # step2: 双眼中心关键点（33, 263为左右眼球中心）
        left_eye = face_landmarks.landmark[33]
        right_eye = face_landmarks.landmark[263]
      
        u = int((left_eye.x + right_eye.x) / 2 * w)
        v = int((left_eye.y + right_eye.y) / 2 * h)


        # step3: 反投影Z
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1] and not np.isnan(depth[v, u]).any():
            z = float(depth[v, u])/1000.0
        else:
            return None , None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        cam_origin_c = np.array([x, y, z])
        print(f"cam_origin_c: {cam_origin_c}")
        # u, v = np.meshgrid(np.arange(w), np.arange(h))
        # z = depth.astype(np.float32) /1000.0

        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy

        # cam_points_c = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        
        
        # rgb_flat = img_rgb.reshape(-1, 3)
        # colors = rgb_flat / 255.0
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(cam_points_c)
        # pcd.colors = o3d.utility.Vector3dVector(colors)

        # world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])

        # # test point position
        # sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
        # sphere.translate(test_point := cam_origin_c)
        # sphere.paint_uniform_color([1, 0, 0])  # 红色

        # # 可视化
        # o3d.visualization.draw_geometries([world_frame, sphere, pcd],
        #                                 window_name="ZED2 PointCloud with World & Camera Frame")

        # cv2.circle(img, (u, v), 5, (0, 255, 0), -1)
        # cv2.imshow("Gaze Point", img)
        # cv2.waitKey(1)



        # step4: L2CSNet gaze方向
        results = self.gaze_pipeline.step(img)
        if results.pitch.shape[0] == 0:
            return None, None
        pitch = results.pitch[0]
        yaw = results.yaw[0]

        if np.isnan(pitch) or np.isnan(yaw):
            self.first_gaze = True
            self.pitch = None
            self.yaw = None

        else:
            if self.first_gaze:
                self.first_gaze = False
                self.pitch = pitch
                self.yaw = yaw
            else:
                self.pitch = self.ema_alpha * pitch + (1 - self.ema_alpha) * self.pitch
                self.yaw=  self.ema_alpha * yaw + (1 - self.ema_alpha) * self.yaw
            
        # 欧拉角→相机系单位向量
        gx = -np.cos(self.pitch) * np.sin(self.yaw)
        gy = - np.sin(self.pitch)
        gz = -np.cos(self.pitch) * np.cos(self.yaw)
        gaze_vec_c = np.array([gx, gy, gz])

        # step5: 转到世界系
        R_wc = T_wc[:3, :3]
        t_wc = T_wc[:3, 3]
        gaze_vec_w = R_wc @ gaze_vec_c
        gaze_origin_w = R_wc @ cam_origin_c + t_wc
        print(f"gaze_vec_w: {gaze_vec_w}, gaze_origin_w: {gaze_origin_w}")
        return gaze_vec_w, gaze_origin_w
    
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
        return pt is not None and (0.0 <= pt[0] <= 1.0) and (0.0 <= pt[1] <= 0.7) # right side of desk

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
        判稳队列: 传入 deque、当前交点、窗口参数，自动更新，返回 stable/last_output/队列/基准等
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
                # 维护基准点和离群检测
                if len(pts_deque) > 0:
                    base = pts_deque[0][0].copy()
                    outlier_cnt = sum(np.linalg.norm(pt - base) > outlier_thresh for pt, _ in pts_deque)
                    if outlier_cnt >= outlier_count:
                        base = None
            # 清理过期点
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

    #     # 画 ROI 框
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
    
    def detect_and_draw_yolo(self, img, u, v, yolo_model, output_path):
        """
        img: bgr8 (cv2)
        u, v: center point pixel of ROI
        yolo_model: 
        output_path: 
        """
        h, w = img.shape[:2]
        roi_size = 500
        half = roi_size // 2
        x1, y1 = max(u - half, 0), max(v - half, 0)
        x2, y2 = min(u + half, w-1), min(v + half, h-1)

        # 画 ROI 框
        cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,255), 2)
        roi = img[y1:y2, x1:x2]
        labels = []
        if roi.size == 0 or roi.shape[0] < 5 or roi.shape[1] < 5:
            print("ROI empty, skip YOLO")
        else:
            result = yolo_model(img, verbose=False,  conf=0.7)[0]
            if result.boxes.shape[0]:
                for box in result.boxes:
                    bx1, by1, bx2, by2 = map(int, box.xyxy[0].cpu().numpy())
                    # 检查框的中心点是否在ROI内
                    center_x = (bx1 + bx2) // 2
                    center_y = (by1 + by2) // 2
                    if x1 <= center_x <= x2 and y1 <= center_y <= y2:
                        cls = int(box.cls[0].cpu().numpy())
                        label = yolo_model.names[cls]
                        conf = float(box.conf[0].cpu().numpy())
                        cv2.rectangle(img, (bx1, by1), (bx2, by2), (0,0,255), 2)
                        cv2.putText(img, f"{label} {conf:.2f}", (bx1, by1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                        labels.append(label)
                print(f"YOLO detected (ROI): {', '.join(labels)}")
        cv2.imwrite(output_path, img)
        print(f"YOLO ROI & label image saved: {output_path}")
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

    def process_detection(self, direction, origin, rgb_msg, finger_pts, direction_name, origin_name, image_name, camera_side):

        # EMA filter
        self.update_ema(direction_name, direction, self.ema_alpha, unit_vector=True)
        self.update_ema(origin_name, origin, self.ema_alpha)
        finger_direction_ema = self.get_ema(direction_name)
        finger_origin_ema = self.get_ema(origin_name)

        # Calculate intersect
        intersect = self.line_plane_intersect(finger_origin_ema, finger_direction_ema, z_plane=0.0)

        # stable intersect
        stable, last_output, finger_pts, finger_base = self.update_stable_point(
            finger_pts, intersect, self.SLIDING_WINDOW_SEC, self.AVG_LAST_N,
            self.OUTLIER_THRESHOLD, self.OUTLIER_COUNT
        )
        if isinstance(rgb_msg, list) and isinstance(image_name, list) and isinstance(camera_side, list):
            all_labels = []
            if stable is not None:
                for rgb, name, side in zip(rgb_msg , image_name, camera_side):
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
                    img = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
                    labels = self.detect_and_draw_yolo(
                        img, u, v, self.yolo_model, os.path.join(self.output_dir, name)
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
                img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
                labels = self.detect_and_draw_yolo(
                    img, u, v, self.yolo_model, os.path.join(self.output_dir, image_name)
                )
                label_output = labels
                # # tts and stt
                # self.ask_label_tts(labels)
            else:
                label_output = []
        
        return stable, last_output, finger_pts, finger_base, finger_direction_ema, finger_origin_ema, intersect, label_output

