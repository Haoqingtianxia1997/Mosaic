# -*- coding: utf-8 -*-
"""
intention_predict.py
实时视频流：人体姿态 + 头部朝向 ROI + YOLO 目标检测 + 语音交互
"""
import cv2                      # OpenCV 图像处理与摄像头
import time                     # 时间函数
import mediapipe as mp          # Google MediaPipe 姿态/分割
import numpy as np              # 数值运算
import sys, os                  # 系统相关
from ultralytics import YOLO    # Ultralytics YOLOv8
# from src.mistral_ai.label_analyzer import LabelAnalyzer  # 可选：LLM 标签分析
from src.transcribe.tts import play_text_to_speech        # 文本转语音
from src.transcribe.stt import VoiceTranscriber           # 语音转文本


def intention_predict(model_path) -> None:
    """
    主入口：打开摄像头，实时推理人体姿势、头部方向、目标检测并语音交互
    """
    # 初始化语音转写器和 YOLOv8 模型
    model_path = model_path if model_path else 'yolo_model/yolo11m.pt'  # 默认权重路径
    transcriber = VoiceTranscriber()
    model = YOLO(model_path)               # 自己训练好的权重
    label_times = {}                         # {label: last_seen_time}
    window_seconds = 15                       # 标签保留时间窗口 (秒)
    tts_interval = 30                        # 每次 TTS/STT 间隔 (秒)
    detected_labels = set()                  # 当前窗口内检测到的标签
    last_query_result = None                 # 上一次 STT 解析结果

    # 初始化 MediaPipe Pose 与 SelfieSegmentation
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        enable_segmentation=False
    )
    mp_selfie_segmentation = mp.solutions.selfie_segmentation
    selfie_segmentation = mp_selfie_segmentation.SelfieSegmentation(
        model_selection=1      # 1=精度高但较慢；0=速度快鲁棒性高
    )

    # 打开默认摄像头 (索引 0)
    capture = cv2.VideoCapture(0)
    # label_analyzer = LabelAnalyzer()       # 若需要 LLM 进一步分析

    tts_stt_in_progress = False              # 是否正在执行 TTS/STT
    last_tts_time = 0                        # 上次 TTS 时间戳

    # ===== 主循环：逐帧处理 =====
    while capture.isOpened():
        ret, frame = capture.read()          # 读取一帧
        if not ret:                          # 读取失败则退出
            break

        current_time = time.time()           # 当前时间戳
        h, w, _ = frame.shape                # 图像高、宽、通道

        # 实时显示原始帧
        cv2.imshow('Head Pose + Local YOLO Detection', frame)

        # 如果正在语音交互，暂停检测，只显示提示
        if tts_stt_in_progress:
            cv2.putText(
                frame, "Paused for TTS/STT...", (50, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2
            )
            cv2.imshow('Head Pose + Local YOLO Detection', frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC 退出
                break
            continue                         # 跳到下一帧

        # ---------- 1. 姿态与分割 ----------
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = pose.process(rgb_frame)                # 姿态推理
        seg_results = selfie_segmentation.process(rgb_frame)  # 人体分割
        mask = seg_results.segmentation_mask > 0.5            # True=人体像素
        if not pose_results.pose_landmarks:
            print("No pose landmarks detected.")
        # ---------- 2. 仅在检测到姿态时继续 ----------
        if pose_results.pose_landmarks:
            # --------  关键点坐标 --------
            landmarks = pose_results.pose_landmarks.landmark
            nose      = landmarks[mp_pose.PoseLandmark.NOSE]
            left_ear  = landmarks[mp_pose.PoseLandmark.LEFT_EAR]
            right_ear = landmarks[mp_pose.PoseLandmark.RIGHT_EAR]

            nose_coords      = (int(nose.x * w),      int(nose.y * h))
            left_ear_coords  = (int(left_ear.x * w),  int(left_ear.y * h))
            right_ear_coords = (int(right_ear.x * w), int(right_ear.y * h))
            

            # --------  头部方向向量 --------
            ear_midpoint = (                           # 两耳中点
                (left_ear_coords[0] + right_ear_coords[0]) // 2,
                (left_ear_coords[1] + right_ear_coords[1]) // 2
            )
            direction_vector = (                       # 鼻子指向向量
                nose_coords[0] - ear_midpoint[0],
                nose_coords[1] - ear_midpoint[1]
            )

            # -------- 向量归一化 --------
            norm = np.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
            vec_length_threshold = 15                  # 正视阈值 (像素)
            if norm == 0:                              # 避免除零
                continue
            unit_vector = (direction_vector[0] / norm,
                           direction_vector[1] / norm)
            
            # ---------- 3. 计算 ROI（头部朝向矩形） ----------
            roi_length = 4000                        # 采样线长度 (像素)
            samples = 60                               # 采样点数
            inside = 0  
            roi_width = 100                            # ROI 宽度 (像素)
            # ROI 中心点 = 鼻子沿朝向移动 roi_length/2
            roi_center = (
                int(nose_coords[0] + unit_vector[0] * roi_length / 2),
                int(nose_coords[1] + unit_vector[1] * roi_length / 2)
            )
            # ROI 长边向量 (dx, dy)  & 垂直方向向量
            dx = int(unit_vector[0] * roi_length / 2)
            dy = int(unit_vector[1] * roi_length / 2)
            perp_vector = (-unit_vector[1], unit_vector[0])   # 逆时针 90°
            px = int(perp_vector[0] * roi_width / 2)
            py = int(perp_vector[1] * roi_width / 2)

            # ROI 四个顶点
            pt1 = (roi_center[0] - dx - px, roi_center[1] - dy - py)
            pt2 = (roi_center[0] + dx - px, roi_center[1] + dy - py)
            pt3 = (roi_center[0] + dx + px, roi_center[1] + dy + py)
            pt4 = (roi_center[0] - dx + px, roi_center[1] - dy + py)

            # ----------  可视化 ----------
            frame[mask] = (0, 255, 0)                  # 人体掩码涂绿色
            cv2.polylines(
                frame, [np.array([pt1, pt2, pt3, pt4], np.int32)],
                isClosed=True, color=(0, 255, 255), thickness=2
            )
            # 头部方向线 & 关键点
            cv2.line(frame, nose_coords,
                     (int(nose_coords[0] + direction_vector[0] * 200),
                      int(nose_coords[1] + direction_vector[1] * 200)),
                     (0, 255, 255), 2)
            cv2.circle(frame, nose_coords, 5, (0, 255, 0), -1)
            cv2.circle(frame, left_ear_coords, 5, (255, 0, 0), -1)
            cv2.circle(frame, right_ear_coords, 5, (255, 0, 0), -1)
            
            # -------- 条件①：面部正视摄像头 --------
            if norm < vec_length_threshold:
                cv2.putText(
                    frame, "Face Forward: YOLO Skipped", (50, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2
                )
                cv2.imshow('Head Pose + Local YOLO Detection', frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue                               # 跳过 YOLO

            # -------- 2.5 条件②：鼻子指向线仍在人体内 --------
                               # 落在人体内计数
            roi_length_check = 100            # 只取 40 cm（或像素 400）来判定即可
            samples = 40                      # 采样点数
            inside = 0                        # ← 每帧都要归零

            for i in range(1, samples + 1):   # (0, 1] 等间距
                ratio = i / samples
                sx = int(nose_coords[0] + unit_vector[0] * roi_length_check * ratio)
                sy = int(nose_coords[1] + unit_vector[1] * roi_length_check * ratio)
                if 0 <= sx < w and 0 <= sy < h and mask[sy, sx]:
                    inside += 1

            # 若超过 70 % 的采样点在人体内 → 跳过 YOLO
            if inside / samples > 0.7:
                cv2.putText(frame, "Line Inside Body: YOLO Skipped",
                            (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow('Head Pose + Local YOLO Detection', frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC 可以提前退出
                    break
                continue          # ← 直接进入 while 下一帧
                     # 跳过 YOLO
            
            # ---------- 6. YOLO 全图推理 ----------
            results_yolo = model(frame, verbose=False)

            for r in results_yolo:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())
                    label = model.names[cls]

                    # 计算 bbox 中心点
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    center_point = (int(cx), int(cy))

                    # 判断中心点是否在 ROI 区域内（朝向线构成的多边形）
                    roi_poly = np.array([pt1, pt2, pt3, pt4], np.int32)
                    is_inside = cv2.pointPolygonTest(roi_poly, center_point, measureDist=False)

                    if is_inside < 0:  # 不在 ROI 区域内
                        continue

                    # ✅ 如果在 ROI 中，执行可视化和记录逻辑
                    label_times[label] = current_time

                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}",
                                (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                                (0, 0, 255), 2)


            # ---------- 7. 维护滑动窗口内标签 ----------
            detected_labels = {
                lbl for lbl, t in label_times.items()
                if current_time - t <= window_seconds and lbl != 'person'
            }
            print("Appeared labels:", detected_labels)

            # 如果需要 LLM 分析，可改用 label_analyzer.analyze_labels
            cooking_labels = detected_labels

            # ---------- 8. 绘制标签列表 ----------
            if cooking_labels:
                cooking_text = "Cooking Items: " + ", ".join(cooking_labels)
                cv2.putText(frame, cooking_text, (50, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
        cv2.imshow('Head Pose + Local YOLO Detection', frame) # to show the frame with all bboxes, texts and masks
        if cv2.waitKey(1) & 0xFF == 27: 
            break

        # ---------- 10. TTS / STT 互动 ----------
        if (current_time - last_tts_time > tts_interval) and detected_labels:
            tts_stt_in_progress = True
            cooking_list = list(detected_labels)
            cooking_str = ", ".join(cooking_list)

            # ----- 10.1 仅一个物体 -----
            if len(cooking_list) == 1:
                tts_text = f"Are you looking for {cooking_str}?"
                play_text_to_speech(tts_text, language='en')
                stt_text = transcriber.auto_record_and_transcribe(5)
                print(f"📝 STT Result: {stt_text}")
                last_tts_time = time.time()
                tts_stt_in_progress = False

                if stt_text and ("yes" in stt_text.lower() or
                                 cooking_list[0].lower() in stt_text.lower()):
                    play_text_to_speech(
                        "OK！",
                        language='en'
                    )
                    last_query_result = f'please give me "{cooking_list[0]}"'
                else:
                    last_query_result =""

            # ----- 10.2 多个物体 -----
            else:
                tts_text = f"There are {cooking_str}. What do you want?"
                play_text_to_speech(tts_text, language='en')
                stt_text = transcriber.auto_record_and_transcribe(5)
                print(f"📝 STT Result: {stt_text}")
                last_tts_time = time.time()
                tts_stt_in_progress = False

                found = ""
                if stt_text:
                    stt_lower = stt_text.lower()
                    for item in cooking_list:
                        if item.lower() in stt_lower:
                            found = item
                            break
                if found:
                    last_query_result = f'please give me "{found}"'
                    play_text_to_speech(
                        "OK！",
                        language='en'
                    )
                else:
                    last_query_result = ""

            print(last_query_result)
            # ✅ 写入 transcription.txt（让主线程能读到）
            TRANS_FILE = "src/transcribe/transcription.txt"
            with open(TRANS_FILE, "w", encoding="utf-8") as f:
                f.write(last_query_result)

    # ===== 循环结束，释放资源 =====
    capture.release()
    cv2.destroyAllWindows()


# ---------------- 主程序入口 ----------------
if __name__ == "__main__":
    intention_predict('yolo_model/yolo11m.pt')
