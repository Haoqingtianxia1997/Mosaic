# -*- coding: utf-8 -*-
"""
intention_predict.py
实时视频流：人体姿态 + 头部朝向 ROI + YOLO 目标检测 + 语音交互
"""
import cv2
import time 
import mediapipe as mp 
import numpy as np 
import sys, os 
from ultralytics import YOLO # YOLOv8
# from src.mistral_ai.label_analyzer import LabelAnalyzer  # Optional: LLM Analysis
from src.transcribe.tts import play_text_to_speech        # Text-to-Speech
from src.transcribe.stt import VoiceTranscriber           # Speech-to-Text
from threading import Event
NEW_TEXT_EVENT1 = Event()
TRANS_FILE = "src/transcribe/transcription.txt"

def intention_predict(model_path) -> None:
    """
    Open camera, real-time inference of human pose, head direction, object detection, and voice interaction
    """
    # Initialize voice transcriber and YOLOv8 model
    model_path = model_path if model_path else 'yolo11m.pt'  # Default weight path
    transcriber = VoiceTranscriber()
    model = YOLO(model_path)               # Custom trained weights
    label_times = {}                         # {label: last_seen_time}
    window_seconds = 15                       # Label retention time window (seconds)
    tts_interval = 30                        # Interval between TTS/STT (seconds)
    detected_labels = set()                  # Labels detected in the current window
    last_query_result = None                 # Last STT parsing result

    # Initialize MediaPipe Pose and SelfieSegmentation
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        enable_segmentation=False
    )
    mp_selfie_segmentation = mp.solutions.selfie_segmentation
    selfie_segmentation = mp_selfie_segmentation.SelfieSegmentation(
        model_selection=1      # 1= high accuracy but slow; 0= fast and robust
    )

    # Open default camera (index 0)
    capture = cv2.VideoCapture(0)
    # label_analyzer = LabelAnalyzer()       # If LLM analysis is needed

    tts_stt_in_progress = False              # Whether TTS/STT is in progress
    last_tts_time = 0                        # Last TTS timestamp

    # ===== Main loop: process each frame =====
    while capture.isOpened():
        ret, frame = capture.read()          # Read a frame
        if not ret:                          # If reading fails, exit
            break

        current_time = time.time()           # Current timestamp
        h, w, _ = frame.shape                # Image height, width, channels

        # Real-time display of the original frame
        cv2.imshow('Head Pose + Local YOLO Detection', frame)

        # If voice interaction is in progress, pause detection and show a prompt
        if tts_stt_in_progress:
            cv2.putText(
                frame, "Paused for TTS/STT...", (50, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2
            )
            cv2.imshow('Head Pose + Local YOLO Detection', frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                break
            continue                         # Skip to the next frame

        # ---------- 1. Pose and Segmentation ----------
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = pose.process(rgb_frame)                # Pose inference
        seg_results = selfie_segmentation.process(rgb_frame)  # Human segmentation
        mask = seg_results.segmentation_mask > 0.5            # True=human pixels
        if not pose_results.pose_landmarks:
            print("No pose landmarks detected.")
        # ---------- 2. Only continue if pose landmarks are detected ----------
        if pose_results.pose_landmarks:
            # --------  Keypoint coordinates --------
            landmarks = pose_results.pose_landmarks.landmark
            nose      = landmarks[mp_pose.PoseLandmark.NOSE]
            left_ear  = landmarks[mp_pose.PoseLandmark.LEFT_EAR]
            right_ear = landmarks[mp_pose.PoseLandmark.RIGHT_EAR]

            nose_coords      = (int(nose.x * w),      int(nose.y * h))
            left_ear_coords  = (int(left_ear.x * w),  int(left_ear.y * h))
            right_ear_coords = (int(right_ear.x * w), int(right_ear.y * h))
            

            # --------  Head direction vector --------
            ear_midpoint = (                           # Midpoint between ears
                (left_ear_coords[0] + right_ear_coords[0]) // 2,
                (left_ear_coords[1] + right_ear_coords[1]) // 2
            )
            direction_vector = (                       # Nose direction vector
                nose_coords[0] - ear_midpoint[0],
                nose_coords[1] - ear_midpoint[1]
            )

            # -------- Vector normalization --------
            norm = np.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
            vec_length_threshold = 15                  # Frontal threshold (pixels)
            if norm == 0:                              # Avoid division by zero
                continue
            unit_vector = (direction_vector[0] / norm,
                           direction_vector[1] / norm)

            # ---------- 3. Calculate ROI (Head Pose Rectangle) ----------
            roi_length = 400                         # ROI length (pixels)
            samples = 60                               # Number of samples
            inside = 0  
            roi_width = 100                            # ROI width (pixels)
            # ROI center = Nose position moved roi_length/2
            roi_center = (
                int(nose_coords[0] + unit_vector[0] * roi_length * 3/ 4),
                int(nose_coords[1] + unit_vector[1] * roi_length * 3/ 4)
            )
            # ROI vectors
            dx = int(unit_vector[0] * roi_length / 4)
            dy = int(unit_vector[1] * roi_length / 4)
            perp_vector = (-unit_vector[1], unit_vector[0])   # 逆时针 90°
            px = int(perp_vector[0] * roi_width / 2)
            py = int(perp_vector[1] * roi_width / 2)

            # ROI 4 corners
            pt1 = (roi_center[0] - dx - px, roi_center[1] - dy - py)
            pt2 = (roi_center[0] + dx - px, roi_center[1] + dy - py)
            pt3 = (roi_center[0] + dx + px, roi_center[1] + dy + py)
            pt4 = (roi_center[0] - dx + px, roi_center[1] - dy + py)

            # ----------  Visualization ----------
            frame[mask] = (0, 255, 0)                  # Human area in green
            cv2.polylines(
                frame, [np.array([pt1, pt2, pt3, pt4], np.int32)],
                isClosed=True, color=(0, 255, 255), thickness=2
            )
            # Head direction line & keypoints
            cv2.line(frame, nose_coords,
                     (int(nose_coords[0] + direction_vector[0] * 200),
                      int(nose_coords[1] + direction_vector[1] * 200)),
                     (0, 255, 255), 2)
            cv2.circle(frame, nose_coords, 5, (0, 255, 0), -1)
            cv2.circle(frame, left_ear_coords, 5, (255, 0, 0), -1)
            cv2.circle(frame, right_ear_coords, 5, (255, 0, 0), -1)

            # -------- Condition 1: Face Forward --------
            if norm < vec_length_threshold:
                cv2.putText(
                    frame, "Face Forward: YOLO Skipped", (50, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2
                )
                cv2.imshow('Head Pose + Local YOLO Detection', frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue                               # Skip YOLO

            # -------- Condition 2.5: Nose Direction Line Inside Body --------
            # Count how many fall inside the human body
            roi_length_check = 100            # Only take 40 cm (or pixels 400) for judgment
            samples = 40                      # Number of samples
            inside = 0                        # Reset count

            for i in range(1, samples + 1):   # (0, 1] uniformly sample points
                ratio = i / samples
                sx = int(nose_coords[0] + unit_vector[0] * roi_length_check * ratio)
                sy = int(nose_coords[1] + unit_vector[1] * roi_length_check * ratio)
                if 0 <= sx < w and 0 <= sy < h and mask[sy, sx]:
                    inside += 1

            # If more than 70% of the sample points are inside the human body → Skip YOLO
            if inside / samples > 0.7:
                cv2.putText(frame, "Line Inside Body: YOLO Skipped",
                            (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow('Head Pose + Local YOLO Detection', frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC can exit early
                    break
                continue          # Skip YOLO

            # ---------- 6. YOLO inference for whole image ----------
            results_yolo = model(frame, verbose=False)

            for r in results_yolo:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())
                    label = model.names[cls]

                    # Calculate bbox center point
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    center_point = (int(cx), int(cy))

                    # Check if center point is inside ROI (polygon formed by direction line)
                    roi_poly = np.array([pt1, pt2, pt3, pt4], np.int32)
                    is_inside = cv2.pointPolygonTest(roi_poly, center_point, measureDist=False)

                    if is_inside < 0:  # Not inside ROI
                        continue

                    # ✅ If inside ROI, perform visualization and recording logic
                    label_times[label] = current_time

                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}",
                                (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                                (0, 0, 255), 2)


            # ---------- 7. Maintain labels within sliding window ----------
            detected_labels = {
                lbl for lbl, t in label_times.items()
                if current_time - t <= window_seconds and lbl != 'person'
            }
            print("Appeared labels:", detected_labels)

            # If LLM analysis is needed, you can use label_analyzer.analyze_labels
            cooking_labels = detected_labels

            # ---------- 8. Draw label list ----------
            if cooking_labels:
                cooking_text = "Cooking Items: " + ", ".join(cooking_labels)
                cv2.putText(frame, cooking_text, (50, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
        cv2.imshow('Head Pose + Local YOLO Detection', frame) # to show the frame with all bboxes, texts and masks
        if cv2.waitKey(1) & 0xFF == 27: 
            break

        # ---------- 10. TTS / STT interaction ----------
        if (current_time - last_tts_time > tts_interval) and detected_labels:
            tts_stt_in_progress = True
            cooking_list = list(detected_labels)
            cooking_str = ", ".join(cooking_list)

            # ----- 10.1 Only one object -----
            if len(cooking_list) == 1:
                tts_text = f"Are you looking for {cooking_str}?"
                play_text_to_speech(tts_text, language='en')
                stt_text = transcriber.auto_record_and_transcribe(5)
                print(f"📝 STT Result: {stt_text}")
                last_tts_time = time.time()
                tts_stt_in_progress = False

                if stt_text and ("yes" in stt_text.lower() or
                                 cooking_list[0].lower() in stt_text.lower()):
                    last_query_result = f'please give me "{cooking_list[0]}"'
                else:
                    last_query_result = None

            # ----- 10.2 Multiple objects -----
            else:
                tts_text = f"There are {cooking_str}. What do you want?"
                play_text_to_speech(tts_text, language='en')
                stt_text = transcriber.auto_record_and_transcribe(5)
                print(f"📝 STT Result: {stt_text}")
                last_tts_time = time.time()
                tts_stt_in_progress = False

                found = None
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
                    last_query_result = None

            print(last_query_result)
            # ✅ Write to transcription.txt (so that the main thread can read it)
            with open(TRANS_FILE, "w", encoding="utf-8") as f:
                f.write(last_query_result)

        # ✅ Notify main thread to trigger LLM+TTS
        NEW_TEXT_EVENT1.set()

        # ✅ Optional: Prevent screen flooding
        time.sleep(1)
    # ===== End of loop, release resources =====
    capture.release()
    cv2.destroyAllWindows()


# ---------------- Main program entry ----------------
if __name__ == "__main__":
    intention_predict(None)
