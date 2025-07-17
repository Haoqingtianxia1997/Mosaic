import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import mediapipe as mp
import time
from threading import Thread, Lock
from collections import deque
import os
from intention_utils.intention import Intention
from intention_utils.open3d_viewer import PersistentOpen3DViewer
from action_interfaces.msg import Labels


def create_Twc_from_quaternion(translation: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """
    生成从相机坐标系到世界坐标系的齐次变换矩阵 T_wc。

    Args:
        translation: np.array([x, y, z]) 相机在世界坐标系中的位置
        quaternion:  np.array([x, y, z, w]) 相机姿态，四元数（world ← camera）

    Returns:
        T_wc: 4x4 np.ndarray 齐次变换矩阵
    """
    assert translation.shape == (3,), "translation 应为形如 (3,) 的向量"
    assert quaternion.shape == (4,), "quaternion 应为形如 (4,) 的向量"

    rot = R.from_quat(quaternion)
    R_wc = rot.as_matrix()

    T_wc = np.eye(4)
    T_wc[:3, :3] = R_wc
    T_wc[:3, 3] = translation

    return T_wc


class HandDetectionWithPointCloudNode(Node):
    def __init__(self):
        super().__init__('hand_detection_with_pointcloud_node')
        self.bridge = CvBridge()
        self.intention = Intention()
        self.viewer = PersistentOpen3DViewer()

        self.output_dir = './saved_images'
        os.makedirs(self.output_dir, exist_ok=True)

        self.mp_hands = mp.solutions.hands
        self.hands_detector = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

        self.rgb_buffer = {'left': None, 'right': None, 'front': None}
        self.depth_buffer = {'left': None, 'right': None, 'front': None}
        self.lock = Lock()
        
        time.sleep(5)

        available_topics = dict(self.get_topic_names_and_types())
        left_rgb = '/zedl/zed_node/rgb/image_rect_color'
        right_rgb = '/zedr/zed_node/rgb/image_rect_color'
        self.left_camera_active = left_rgb in available_topics 
        self.right_camera_active = right_rgb in available_topics 

        self.label_pub = self.create_publisher(Labels, 'label_output', 10)

        # TODO: subscription 3
        self.create_subscription(Image, '/gaze_camera/zed_node/rgb/image_rect_color', lambda msg: self.buffer_callback(msg, 'front', 'rgb'), 10)
        self.create_subscription(Image, '/gaze_camera/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'front', 'depth'), 10)


        self.create_subscription(Image, '/zedl/zed_node/rgb/image_rect_color', lambda msg: self.buffer_callback(msg, 'left', 'rgb'), 10)
        self.create_subscription(Image, '/zedr/zed_node/rgb/image_rect_color', lambda msg: self.buffer_callback(msg, 'right', 'rgb'), 10)
        self.create_subscription(Image, '/zedl/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'left', 'depth'), 10)
        self.create_subscription(Image, '/zedr/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'right', 'depth'), 10)


        self.get_logger().info("🖐️ 手部检测 + 点云可视化节点已启动")
        
        self.T_wc_l = create_Twc_from_quaternion(translation = np.array([0.11261126, -0.50195948, 0.55795671]), quaternion = np.array([0.81395177, -0.40028226, -0.07631803, -0.41404371]))
        self.intrinsics_l = (1060.0899658203125, 1059.0899658203125, 958.9099731445312, 561.5670166015625)

        self.T_wc_r = create_Twc_from_quaternion(translation = np.array([0.903701253331141, 0.439249176547482, 0.598645500102408]), quaternion = np.array([-0.404974467935380, -0.808551385290863, 0.425767747250020, 0.031018753461827]))
        self.intrinsics_r = (1059.9764404296875, 1059.9764404296875, 963.07568359375, 522.3530883789062)

        # TODO: front camera
        self.T_wc_f = None
        self.intrinsics_f = None

        # finger detection
        self.finger_pts = deque()    # 存交点与时间戳
        self.finger_base = None
        self.finger_last_output = None
        self.finger_stable_pos = None   # 当前窗口的稳定输出

        # # gaze detection
        # self.gaze_direction_ema = None
        # self.gaze_origin_ema = None
        # self.gaze_pts = deque()
        # self.gaze_base = None
        # self.gaze_last_output = None
        # self.gaze_stable_pos = None

        Thread(target=self.monitor_pair_thread, daemon=True).start()

    def buffer_callback(self, msg, side, kind):
        with self.lock:
            if kind == 'rgb':
                self.rgb_buffer[side] = msg
                img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                results = self.hands_detector.process(img_rgb)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                out_path = os.path.join(self.output_dir, f'{side}_gesture_result.png')
                cv2.imwrite(out_path, img)

            elif kind == 'depth':
                self.depth_buffer[side] = msg

    def monitor_pair_thread(self):
        while rclpy.ok():
            with self.lock:
                hand_detected = False
                
                if self.left_camera_active and self.right_camera_active:
                    rgb_msg_r = self.rgb_buffer['right']
                    depth_msg_r = self.depth_buffer['right']
                    rgb_msg_l = self.rgb_buffer['left']
                    depth_msg_l = self.depth_buffer['left']
                    rgb_msg_front = self.rgb_buffer['front']
                    depth_msg_front = self.depth_buffer['front']

                    if rgb_msg_l is not None and depth_msg_l is not None and rgb_msg_r is not None and depth_msg_r is not None:

                        direction_l, origin_l = self.intention.get_hand_pose(rgb_msg_l, depth_msg_l, self.T_wc_l, self.intrinsics_l)
                        direction_r, origin_r = self.intention.get_hand_pose(rgb_msg_r, depth_msg_r, self.T_wc_r, self.intrinsics_r)
                        if rgb_msg_front is not None and depth_msg_front is not None:
                            gaze_direction, gaze_origin = self.intention.get_gaze_direction(rgb_msg_front, depth_msg_front, self.T_wc_f, self.intrinsics_f)
                        else:
                            gaze_direction, gaze_origin = None, None

                        all_direction=[]
                        all_origin=[]
                
                        if direction_l is not None and origin_l is not None and direction_r is not None and origin_r is not None :
                            print("Detection only with both cameras. ")
                            finger_direction = (direction_l + direction_r) / 2
                            finger_origin = (origin_l + origin_r) / 2

                            if gaze_direction is not None and gaze_origin is not None:

                                all_direction.append(finger_direction)
                                all_origin.append(finger_origin)
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)
                            else:

                                all_direction.append(finger_direction)
                                all_origin.append(finger_origin)


                            if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                direction = direction / np.linalg.norm(direction)
                                hand_detected = True
                                
                                (self.finger_stable_pos, 
                                self.finger_last_output, 
                                self.finger_pts, 
                                self.finger_base, 
                                finger_direction_ema, 
                                finger_origin_ema, 
                                intersect, 
                                finger_label_output) = self.intention.process_detection(direction, origin, [rgb_msg_r, rgb_msg_l], self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = ['fusion_gesture_yolo_r.png', 'fusion_gesture_yolo_l.png'], camera_side = ['right', 'left'])        
                                
                                # publish label output
                                label_msg = Labels()
                                label_msg.gesture_labels = finger_label_output
                                self.label_pub.publish(label_msg)

                                all_direction.append(finger_direction_ema)
                                all_origin.append(finger_origin_ema)

                                # draw arrow
                                self.viewer.update_arrow_async(all_direction, all_origin)
                                if self.intention.in_valid_area(intersect):
                                    self.viewer.update_intersect_async(intersect)
                                else:
                                    self.viewer.update_intersect_async(None)

                        elif direction_l is not None and origin_l is not None:
                            print("Detection only with left camera. ")
                            direction = direction_l
                            origin = origin_l

                            if gaze_direction is not None and gaze_origin is not None:
                                direction = (direction + gaze_direction) / 2
                                origin = (origin + gaze_origin) / 2

                                all_direction.append(direction)
                                all_origin.append(origin)
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)
                            else:
                                direction = direction
                                origin = origin

                                all_direction.append(direction)
                                all_origin.append(origin)


                            if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                direction = direction / np.linalg.norm(direction)
                                hand_detected = True
                                
                                (self.finger_stable_pos, 
                                self.finger_last_output, 
                                self.finger_pts, 
                                self.finger_base, 
                                finger_direction_ema, 
                                finger_origin_ema, 
                                intersect, 
                                finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg_l, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'fusion_gesture_yolo_l.png', camera_side = 'left')
                                
                                all_direction.append(finger_direction_ema)
                                all_origin.append(finger_origin_ema)

                                # draw arrow
                                self.viewer.update_arrow_async(all_direction, all_origin)
                                if self.intention.in_valid_area(intersect):
                                    self.viewer.update_intersect_async(intersect)
                                else:
                                    self.viewer.update_intersect_async(None)

                        elif direction_r is not None and origin_r is not None:
                            print("Detection only with right camera. ")
                            direction = direction_r
                            origin = origin_r

                            if gaze_direction is not None and gaze_origin is not None:
                                direction = (direction + gaze_direction) / 2
                                origin = (origin + gaze_origin) / 2

                                all_direction.append(direction)
                                all_origin.append(origin)
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)
                            else:
                                direction = direction
                                origin = origin

                                all_direction.append(direction)
                                all_origin.append(origin)

                            if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                direction = direction / np.linalg.norm(direction)
                                hand_detected = True
                                
                                (self.finger_stable_pos, 
                                self.finger_last_output, 
                                self.finger_pts, 
                                self.finger_base, 
                                finger_direction_ema, 
                                finger_origin_ema, 
                                intersect, 
                                finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg_r, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'fusion_gesture_yolo_r.png', camera_side = 'right')
                                
                                all_direction.append(finger_direction_ema)
                                all_origin.append(finger_origin_ema)

                                # draw arrow
                                self.viewer.update_arrow_async(all_direction, all_origin)
                                if self.intention.in_valid_area(intersect):
                                    self.viewer.update_intersect_async(intersect)
                                else:
                                    self.viewer.update_intersect_async(None)
                        
                        else:

                            if gaze_direction is not None and gaze_origin is not None:
                                
                                direction = gaze_direction
                                origin = gaze_origin

                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    hand_detected = True
                                    
                                    (self.finger_stable_pos, 
                                    self.finger_last_output, 
                                    self.finger_pts, 
                                    self.finger_base, 
                                    finger_direction_ema, 
                                    finger_origin_ema, 
                                    intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg_r, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'fusion_gesture_yolo_r.png', camera_side = 'right')
                                    
                                    all_direction.append(finger_direction_ema)
                                    all_origin.append(finger_origin_ema)

                                    # draw arrow
                                    self.viewer.update_arrow_async(all_direction, all_origin)
                                    if self.intention.in_valid_area(intersect):
                                        self.viewer.update_intersect_async(intersect)
                                    else:
                                        self.viewer.update_intersect_async(None)
                            else:
                                all_direction.clear()
                                all_origin.clear()

                            

                            self.finger_pts.clear()
                            self.finger_base = None
                            self.finger_last_output = None
                            self.finger_stable_pos = None  

                elif self.right_camera_active:
                    rgb_msg = self.rgb_buffer['right']
                    depth_msg = self.depth_buffer['right']

                    # finger detection
                    if rgb_msg is not None and depth_msg is not None:
                        finger_direction, finger_origin = self.intention.get_hand_pose(rgb_msg, depth_msg, self.T_wc_r, self.intrinsics_r)
                        gaze_direction, gaze_origin = self.intention.get_gaze_direction(rgb_msg, depth_msg, self.T_wc_r, self.intrinsics_r)
                        
                        all_direction=[]
                        all_origin=[]

                        if finger_direction is not None and finger_origin is not None and gaze_direction is not None and gaze_origin is not None:
                            direction = (finger_direction + gaze_direction) / 2
                            origin = (finger_origin + gaze_origin) / 2

                            all_direction.append(finger_direction)
                            all_origin.append(finger_origin)
                            all_direction.append(gaze_direction)
                            all_origin.append(gaze_origin)

                        elif finger_direction is not None and finger_origin is not None:
                            direction = finger_direction
                            origin = finger_origin
                            all_direction.append(finger_direction)
                            all_origin.append(finger_origin)

                        elif gaze_direction is not None and gaze_origin is not None:
                            direction = gaze_direction
                            origin = gaze_origin
                            all_direction.append(gaze_direction)
                            all_origin.append(gaze_origin)
                        else:
                            direction = None
                            origin = None
                            all_direction.clear()
                            all_origin.clear()

                        if direction is not None and origin is not None:
                            print("Detection only with right camera. ")
                            if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                direction = direction / np.linalg.norm(direction)
                                hand_detected = True
                                
                                (self.finger_stable_pos, 
                                self.finger_last_output, 
                                self.finger_pts, 
                                self.finger_base, 
                                finger_direction_ema, 
                                finger_origin_ema, 
                                intersect, 
                                finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'right_gesture_yolo.png', camera_side = 'right')
                                
                                all_direction.append(finger_direction_ema)
                                all_origin.append(finger_origin_ema)

                                # draw arrow
                                self.viewer.update_arrow_async(all_direction, all_origin)
                                if self.intention.in_valid_area(intersect):
                                    self.viewer.update_intersect_async(intersect)
                                else:
                                    self.viewer.update_intersect_async(None)
                        

                    else:
                        self.finger_pts.clear()
                        self.finger_base = None
                        self.finger_last_output = None
                        self.finger_stable_pos = None                   


                elif self.left_camera_active:
                    rgb_msg = self.rgb_buffer['left']
                    depth_msg = self.depth_buffer['left']
                    rgb_msg_front = self.rgb_buffer['front']
                    depth_msg_front = self.depth_buffer['front']

                    if rgb_msg is not None and depth_msg is not None:
                        # print("lo")
                        finger_direction, finger_origin = self.intention.get_hand_pose(rgb_msg, depth_msg, self.T_wc_l, self.intrinsics_l)
                        if rgb_msg_front is not None and depth_msg_front is not None:
                            gaze_direction, gaze_origin = self.intention.get_gaze_direction(rgb_msg_front, depth_msg_front, self.T_wc_f, self.intrinsics_f)
                        else:
                            gaze_direction, gaze_origin = None, None
                        
                        all_direction=[]
                        all_origin=[]

                        if finger_direction is not None and finger_origin is not None and gaze_direction is not None and gaze_origin is not None:
                            direction = (finger_direction + gaze_direction) / 2
                            origin = (finger_origin + gaze_origin) / 2

                            all_direction.append(finger_direction)
                            all_origin.append(finger_origin)
                            all_direction.append(gaze_direction)
                            all_origin.append(gaze_origin)

                        elif finger_direction is not None and finger_origin is not None:
                            direction = finger_direction
                            origin = finger_origin
                            all_direction.append(finger_direction)
                            all_origin.append(finger_origin)

                        elif gaze_direction is not None and gaze_origin is not None:
                            direction = gaze_direction
                            origin = gaze_origin
                            all_direction.append(gaze_direction)
                            all_origin.append(gaze_origin)
                        else:
                            direction = None
                            origin = None
                            all_direction.clear()
                            all_origin.clear()

                        if direction is not None and origin is not None:
                            print("Detection only with left camera. ")
                            if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                direction = direction / np.linalg.norm(direction)
                                hand_detected = True
                                
                                (self.finger_stable_pos, 
                                self.finger_last_output, 
                                self.finger_pts, 
                                self.finger_base, 
                                finger_direction_ema, 
                                finger_origin_ema, 
                                intersect, 
                                finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'left_gesture_yolo.png', camera_side = 'left')
                                
                                all_direction.append(finger_direction_ema)
                                all_origin.append(finger_origin_ema)

                                # draw arrow
                                self.viewer.update_arrow_async(all_direction, all_origin)
                                if self.intention.in_valid_area(intersect):
                                    self.viewer.update_intersect_async(intersect)
                                else:
                                    self.viewer.update_intersect_async(None)
                                
                    else:
                        self.finger_pts.clear()
                        self.finger_base = None
                        self.finger_last_output = None
                        self.finger_stable_pos = None
                
                else:
                    rgb_msg = None
                    depth_msg = None
                    print("There are not any rgb_msg and depth_msg !")
                
            if not hand_detected:
                self.viewer.update_arrow_async(None, None)
                self.viewer.update_intersect_async(None)

            time.sleep(0.01)







def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionWithPointCloudNode()
    ros_spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_spin_thread.start()
    try:
        node.viewer.run_main_loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    node.viewer.close()






if __name__ == '__main__':
    main()
