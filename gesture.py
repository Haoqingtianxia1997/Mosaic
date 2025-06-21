import cv2, numpy as np
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
with mp_hands.Hands(static_image_mode=False,
                    max_num_hands=1,
                    model_complexity=1,
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5) as hands:

    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            break

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = hands.process(rgb)
        rgb.flags.writeable = True

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                h, w = frame.shape[:2]

                # ---------- A) 计算手指方向 ----------
                tip_xy  = np.array([hand_lms.landmark[8].x * w,
                                    hand_lms.landmark[8].y * h]).astype(int)
                base_xy = np.array([hand_lms.landmark[5].x * w,
                                    hand_lms.landmark[5].y * h]).astype(int)
                v = tip_xy - base_xy
                scale = 6 
                end_xy = (base_xy + v * scale).astype(int)
                angle = (np.degrees(np.arctan2(-v[1], v[0])) + 360) % 360
                dirs = ['右','右上','上','左上','左','左下','下','右下']
                idx = int(((angle + 22.5) % 360) // 45)

                cv2.arrowedLine(frame, tuple(base_xy), tuple(end_xy),
                                (0, 255, 0), 2, tipLength=0.1)
                cv2.circle(frame, tuple(tip_xy), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"{angle:.1f}° {dirs[idx]}",
                            (base_xy[0]+10, base_xy[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # ---------- B) 估算手掌心 ----------
                palm_ids = [0, 5, 9, 13, 17]
                pts = np.array([[hand_lms.landmark[i].x * w,
                                 hand_lms.landmark[i].y * h] for i in palm_ids])
                center = pts.mean(axis=0).astype(int)

                cv2.circle(frame, tuple(center), 6, (255, 100, 0), -1)
                cv2.putText(frame, f"({center[0]}, {center[1]})",
                            (center[0]+10, center[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # ---------- C) 可选：画骨架 ----------
                mp_drawing.draw_landmarks(frame, hand_lms,
                                          mp_hands.HAND_CONNECTIONS)

        cv2.imshow('Finger & Palm Center', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
