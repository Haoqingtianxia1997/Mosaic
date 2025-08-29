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
from realsense2_camera_msgs.msg import RGBD


def create_Twc_from_quaternion(translation: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """
    Generate a homogeneous transformation matrix T_wc from camera coordinates to world coordinates.

    Args:
        translation: np.array([x, y, z]) The position of the camera in the world coordinate system.
        quaternion:  np.array([x, y, z, w]) The orientation of the camera as a quaternion (world ← camera).

    Returns:
        T_wc: 4x4 np.ndarray homogeneous transformation matrix
    """
    assert translation.shape == (3,), "translation must be a vector of shape (3,)"
    assert quaternion.shape == (4,), "quaternion must be a vector of shape (4,)"

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

        self.rgb_buffer = {'left': None, 'right': None, 'gaze': None}
        self.depth_buffer = {'left': None, 'right': None, 'gaze': None}
        self.lock = Lock()
        
        time.sleep(5)

        available_topics = dict(self.get_topic_names_and_types())
        left_rgb = '/zedl/zed_node/rgb/image_rect_color/compressed'
        right_rgb = '/zedr/zed_node/rgb/image_rect_color/compressed'
        self.left_camera_active = left_rgb in available_topics 
        self.right_camera_active = right_rgb in available_topics 

        self.label_pub = self.create_publisher(Labels, 'label_output', 10)
        self.label_msg = Labels()

        # TODO: subscription 3
        self.create_subscription(RGBD, '/camera/camera/rgbd', lambda msg: self.buffer_callback(msg, 'gaze','rgb + depth'), 10)
        self.create_subscription(Image, '/zedl/zed_node/rgb/image_rect_color', lambda msg: self.buffer_callback(msg, 'left', 'rgb'), 10)
        self.create_subscription(Image, '/zedr/zed_node/rgb/image_rect_color', lambda msg: self.buffer_callback(msg, 'right', 'rgb'), 10)
        self.create_subscription(Image, '/zedl/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'left', 'depth'), 10)
        self.create_subscription(Image, '/zedr/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'right', 'depth'), 10)


        self.get_logger().info("🖐️ Hand detection + point cloud visualization node started")

        self.T_wc_l = create_Twc_from_quaternion(translation = np.array([0.11261126, -0.50195948, 0.55795671]), quaternion = np.array([0.81395177, -0.40028226, -0.07631803, -0.41404371]))
        self.intrinsics_l = (1060.0899658203125, 1059.0899658203125, 958.9099731445312, 561.5670166015625)

        self.T_wc_r = create_Twc_from_quaternion(translation = np.array([0.903701253331141, 0.439249176547482, 0.598645500102408]), quaternion = np.array([-0.404974467935380, -0.808551385290863, 0.425767747250020, 0.031018753461827]))
        self.intrinsics_r = (1059.9764404296875, 1059.9764404296875, 963.07568359375, 522.3530883789062)

        # gaze camera
        self.T_wc_f = create_Twc_from_quaternion(translation = np.array([0.07261126, -0.54195948, 0.82295671]), quaternion = np.array([0.5, -0.5, 0.5, -0.5]))
        self.intrinsics_f = (910.5794677734375, 910.5794677734375, 643.673583984375, 367.935546875)

        # finger detection
        self.finger_pts = deque()    # Store intersection points and timestamps
        self.finger_base = None
        self.finger_last_output = None
        self.finger_stable_pos = None   # Current window's stable output

        # gaze detection
        self.gaze_pts = deque()
        self.gaze_base = None
        self.gaze_last_output = None
        self.gaze_stable_pos = None

        Thread(target=self.monitor_pair_thread, daemon=True).start()

    def buffer_callback(self, msg, side, kind):
        with self.lock:
            print(side, kind)
            if side == 'gaze':
                self.rgb_buffer[side] = msg.rgb
                img = self.bridge.imgmsg_to_cv2(msg.rgb, 'bgr8')
                out_path = os.path.join(self.output_dir, f'gaze_result.png')
                cv2.imwrite(out_path, img)
                self.depth_buffer[side] = msg.depth
            else:
                print("----------------------")
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
                gaze_detected = False
                self.label_msg = Labels()
                gaze_intersect = None
                finger_intersect = None
                
                if self.left_camera_active and self.right_camera_active:
                    rgb_msg_r = self.rgb_buffer['right']
                    depth_msg_r = self.depth_buffer['right']
                    rgb_msg_l = self.rgb_buffer['left']
                    depth_msg_l = self.depth_buffer['left']
                    rgb_msg_gaze = self.rgb_buffer['gaze']
                    depth_msg_gaze = self.depth_buffer['gaze']

                    if rgb_msg_l is not None and depth_msg_l is not None and rgb_msg_r is not None and depth_msg_r is not None:

                        finger_direction_l, finger_origin_l = self.intention.get_hand_pose(rgb_msg_l, depth_msg_l, self.T_wc_l, self.intrinsics_l)
                        finger_direction_r, finger_origin_r = self.intention.get_hand_pose(rgb_msg_r, depth_msg_r, self.T_wc_r, self.intrinsics_r)
                        if rgb_msg_gaze is not None and depth_msg_gaze is not None:
                            gaze_direction, gaze_origin = self.intention.get_gaze_direction(rgb_msg_gaze, depth_msg_gaze, self.T_wc_f, self.intrinsics_f)
                        else:
                            gaze_direction, gaze_origin = None, None

                        all_direction=[]
                        all_origin=[]
                        # all_intersect=[]

                        if finger_direction_l is not None and finger_origin_l is not None and finger_direction_r is not None and finger_origin_r is not None:
                            print("Detection only with both cameras. ")
                            finger_direction = (finger_direction_l + finger_direction_r) / 2
                            finger_origin = (finger_origin_l + finger_origin_r) / 2
                            
                            if not np.isnan(finger_direction).any() and not np.isnan(finger_origin).any():
                                all_direction.append(finger_direction)
                                all_origin.append(finger_origin)

                                direction, origin = finger_direction, finger_origin 
                                
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    hand_detected = True
                                    
                                    (self.finger_stable_pos, 
                                    self.finger_last_output, 
                                    self.finger_pts, 
                                    self.finger_base, 
                                    finger_direction_ema, 
                                    finger_origin_ema, 
                                    finger_intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, [rgb_msg_r, rgb_msg_l], self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = ['gesture_yolo_r.png', 'gesture_yolo_l.png'], camera_side = ['right', 'left'])        

                                    self.label_msg.gesture_labels = finger_label_output
                          

                            if gaze_direction is not None and gaze_origin is not None and not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)
                                direction, origin = gaze_direction, gaze_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    gaze_detected = True
                                    
                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(direction, origin, [rgb_msg_r, rgb_msg_l], self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = ['gaze_yolo_r.png', 'gaze_yolo_l.png'], camera_side = ['right', 'left'])        

                                    self.label_msg.gaze_labels = gaze_label_output
                      

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(finger_intersect) and self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                                self.viewer.update_intersect_async(gaze_intersect)
                            elif self.intention.in_valid_area(finger_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                            elif self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(gaze_intersect)
                            else:
                                self.viewer.update_intersect_async(None)

                        elif finger_direction_l is not None and finger_origin_l is not None:
                            print("Detection only with left camera. ")
                            finger_direction = finger_direction_l
                            finger_origin = finger_origin_l

                            direction, origin = finger_direction, finger_origin

                            if not np.isnan(direction).any() and not np.isnan(origin).any():
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
                                    finger_intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, [rgb_msg_r, rgb_msg_l], self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = ['gesture_yolo_r.png', 'gesture_yolo_l.png'], camera_side = ['right', 'left'])        

                                    self.label_msg.gesture_labels = finger_label_output
                               

                            if gaze_direction is not None and gaze_origin is not None and not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)
                                direction, origin = gaze_direction, gaze_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    gaze_detected = True
                                    
                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(direction, origin, [rgb_msg_r, rgb_msg_l], self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = ['gaze_yolo_r.png', 'gaze_yolo_l.png'], camera_side = ['right', 'left'])        

                                    self.label_msg.gaze_labels = gaze_label_output
                                    

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(finger_intersect) and self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                                self.viewer.update_intersect_async(gaze_intersect)
                            elif self.intention.in_valid_area(finger_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                            elif self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(gaze_intersect)
                            else:
                                self.viewer.update_intersect_async(None)

                        elif finger_direction_r is not None and finger_origin_r is not None:
                            print("Detection only with right camera. ")
                            finger_direction = finger_direction_r
                            finger_origin = finger_origin_r

                            direction, origin = finger_direction, finger_origin
                            if not np.isnan(direction).any() and not np.isnan(origin).any():
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
                                    finger_intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, [rgb_msg_r, rgb_msg_l], self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = ['gesture_yolo_r.png', 'gesture_yolo_l.png'], camera_side = ['right', 'left'])        

                                    self.label_msg.gesture_labels = finger_label_output
                           

                            if gaze_direction is not None and gaze_origin is not None and not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)
                                direction, origin = gaze_direction, gaze_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    gaze_detected = True
                                    
                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(direction, origin, [rgb_msg_r, rgb_msg_l], self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = ['gaze_yolo_r.png', 'gaze_yolo_l.png'], camera_side = ['right', 'left'])        

                                    self.label_msg.gaze_labels = gaze_label_output
                                 

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(finger_intersect) and self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                                self.viewer.update_intersect_async(gaze_intersect)
                            elif self.intention.in_valid_area(finger_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                            elif self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(gaze_intersect)
                            else:
                                self.viewer.update_intersect_async(None)
                        
                        else:

                            if gaze_direction is not None and gaze_origin is not None and not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():

                                if np.linalg.norm(gaze_direction) > 1e-6 and not np.isnan(gaze_direction).any():
                                    gaze_direction = gaze_direction / np.linalg.norm(gaze_direction)
                                    gaze_detected = True

                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(gaze_direction, gaze_origin, [rgb_msg_r, rgb_msg_l], self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = ['gaze_yolo_r.png', 'gaze_yolo_l.png'], camera_side = ['right', 'left'])

                                    self.label_msg.gaze_labels = gaze_label_output

                                    all_direction.append(gaze_direction)
                                    all_origin.append(gaze_origin)

                                self.label_pub.publish(self.label_msg)
                                
                                # draw arrow
                                self.viewer.update_arrow_async(all_direction, all_origin)
                                if self.intention.in_valid_area(gaze_intersect):
                                    self.viewer.update_intersect_async(gaze_intersect)
                                else:
                                    self.viewer.update_intersect_async(None)
                            else:
                                all_direction.clear()
                                all_origin.clear()

                                self.label_pub.publish(self.label_msg)
                    else:
                        self.finger_pts.clear()
                        self.finger_base = None
                        self.finger_last_output = None
                        self.finger_stable_pos = None  

                        self.gaze_pts.clear()
                        self.gaze_base = None
                        self.gaze_last_output = None
                        self.gaze_stable_pos = None 
                        self.label_pub.publish(self.label_msg)

                elif self.right_camera_active:
                    rgb_msg = self.rgb_buffer['right']
                    depth_msg = self.depth_buffer['right']
                    rgb_msg_gaze = self.rgb_buffer['gaze']
                    depth_msg_gaze = self.depth_buffer['gaze']

                    # finger detection
                    if rgb_msg is not None and depth_msg is not None:

                        finger_direction, finger_origin = self.intention.get_hand_pose(rgb_msg, depth_msg, self.T_wc_r, self.intrinsics_r)
                        gaze_direction, gaze_origin = self.intention.get_gaze_direction(rgb_msg_gaze, depth_msg_gaze, self.T_wc_f, self.intrinsics_f)
                        
                        all_direction=[]
                        all_origin=[]
                        # all_intersect=[]

                        if finger_direction is not None and finger_origin is not None and gaze_direction is not None and gaze_origin is not None:
                            if not np.isnan(finger_direction).any() and not np.isnan(finger_origin).any():
                                all_direction.append(finger_direction)
                                all_origin.append(finger_origin)

                                direction, origin = finger_direction, finger_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    hand_detected = True
                                    
                                    (self.finger_stable_pos, 
                                    self.finger_last_output, 
                                    self.finger_pts, 
                                    self.finger_base, 
                                    finger_direction_ema, 
                                    finger_origin_ema, 
                                    finger_intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'gesture_yolo_r.png', camera_side = 'right')        

                                    self.label_msg.gesture_labels = finger_label_output
                          
                            if not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)

                                direction, origin = gaze_direction, gaze_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    gaze_detected = True
                                    
                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = 'gaze_yolo_r.png', camera_side = 'right')        

                                    self.label_msg.gaze_labels = gaze_label_output
                      

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(finger_intersect) and self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                                self.viewer.update_intersect_async(gaze_intersect)
                            elif self.intention.in_valid_area(finger_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                            elif self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(gaze_intersect)
                            else:
                                self.viewer.update_intersect_async(None)

                        elif finger_direction is not None and finger_origin is not None:
                            if not np.isnan(finger_direction).any() and not np.isnan(finger_origin).any():
                                all_direction.append(finger_direction)
                                all_origin.append(finger_origin)
                            
                                direction, origin = finger_direction, finger_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    hand_detected = True
                                    
                                    (self.finger_stable_pos, 
                                    self.finger_last_output, 
                                    self.finger_pts, 
                                    self.finger_base, 
                                    finger_direction_ema, 
                                    finger_origin_ema, 
                                    finger_intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'gesture_yolo_r.png', camera_side = 'right')        

                                    self.label_msg.gesture_labels = finger_label_output
                      

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(finger_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                            else:
                                self.viewer.update_intersect_async(None)

                        elif gaze_direction is not None and gaze_origin is not None:
                            if not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)

                                direction, origin = gaze_direction, gaze_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    gaze_detected = True
                                    
                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = 'gaze_yolo_r.png', camera_side = 'right')        

                                    self.label_msg.gaze_labels = gaze_label_output
                      

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(gaze_intersect)
                            else:
                                self.viewer.update_intersect_async(None)
                        
                        else:

                            all_direction.clear()
                            all_origin.clear()

                            self.label_pub.publish(self.label_msg)
                                                           
                    else:
                        self.finger_pts.clear()
                        self.finger_base = None
                        self.finger_last_output = None
                        self.finger_stable_pos = None

                        self.gaze_pts.clear()
                        self.gaze_base = None
                        self.gaze_last_output = None
                        self.gaze_stable_pos = None

                        self.label_pub.publish(self.label_msg)

                elif self.left_camera_active:
                    rgb_msg = self.rgb_buffer['left']
                    depth_msg = self.depth_buffer['left']
                    rgb_msg_gaze = self.rgb_buffer['gaze']
                    depth_msg_gaze = self.depth_buffer['gaze']

                    if rgb_msg is not None and depth_msg is not None:

                        finger_direction, finger_origin = self.intention.get_hand_pose(rgb_msg, depth_msg, self.T_wc_l, self.intrinsics_l)
                        if rgb_msg_gaze is not None and depth_msg_gaze is not None:
                            gaze_direction, gaze_origin = self.intention.get_gaze_direction(rgb_msg_gaze, depth_msg_gaze, self.T_wc_f, self.intrinsics_f)
                        else:
                            gaze_direction, gaze_origin = None, None
                        
                        all_direction=[]
                        all_origin=[]
                        # all_intersect=[]

                        if finger_direction is not None and finger_origin is not None and gaze_direction is not None and gaze_origin is not None:
                            if not np.isnan(finger_direction).any() and not np.isnan(finger_origin).any():
                                all_direction.append(finger_direction)
                                all_origin.append(finger_origin)

                                direction, origin = finger_direction, finger_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    hand_detected = True
                                    
                                    (self.finger_stable_pos, 
                                    self.finger_last_output, 
                                    self.finger_pts, 
                                    self.finger_base, 
                                    finger_direction_ema, 
                                    finger_origin_ema, 
                                    finger_intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'gesture_yolo_l.png', camera_side = 'left')        

                                    self.label_msg.gesture_labels = finger_label_output
                          
                            if not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)

                                direction, origin = gaze_direction, gaze_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    gaze_detected = True
                                    
                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = 'gaze_yolo_l.png', camera_side = 'left')        

                                    self.label_msg.gaze_labels = gaze_label_output
                      

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(finger_intersect) and self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                                self.viewer.update_intersect_async(gaze_intersect)
                            elif self.intention.in_valid_area(finger_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                            elif self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(gaze_intersect)
                            else:
                                self.viewer.update_intersect_async(None)

                        elif finger_direction is not None and finger_origin is not None:
                            if not np.isnan(finger_direction).any() and not np.isnan(finger_origin).any():
                                all_direction.append(finger_direction)
                                all_origin.append(finger_origin)

                                direction, origin = finger_direction, finger_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    hand_detected = True
                                    
                                    (self.finger_stable_pos, 
                                    self.finger_last_output, 
                                    self.finger_pts, 
                                    self.finger_base, 
                                    finger_direction_ema, 
                                    finger_origin_ema, 
                                    finger_intersect, 
                                    finger_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.finger_pts, direction_name = "finger_direction", origin_name = "finger_origin", image_name = 'gesture_yolo_l.png', camera_side = 'left')        

                                    self.label_msg.gesture_labels = finger_label_output
                      

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(finger_intersect):
                                self.viewer.update_intersect_async(finger_intersect)
                            else:
                                self.viewer.update_intersect_async(None)

                        elif gaze_direction is not None and gaze_origin is not None:
                            if not np.isnan(gaze_direction).any() and not np.isnan(gaze_origin).any():
                                all_direction.append(gaze_direction)
                                all_origin.append(gaze_origin)

                                direction, origin = gaze_direction, gaze_origin
                                if np.linalg.norm(direction) > 1e-6 and not np.isnan(direction).any():
                                    direction = direction / np.linalg.norm(direction)
                                    gaze_detected = True
                                    
                                    (self.gaze_stable_pos, 
                                    self.gaze_last_output, 
                                    self.gaze_pts, 
                                    self.gaze_base, 
                                    gaze_direction_ema, 
                                    gaze_origin_ema, 
                                    gaze_intersect, 
                                    gaze_label_output) = self.intention.process_detection(direction, origin, rgb_msg, self.gaze_pts, direction_name = "gaze_direction", origin_name = "gaze_origin", image_name = 'gaze_yolo_l.png', camera_side = 'left')        

                                    self.label_msg.gaze_labels = gaze_label_output
                      

                            self.label_pub.publish(self.label_msg)
                            
                            # draw arrow
                            self.viewer.update_arrow_async(all_direction, all_origin)
                            if self.intention.in_valid_area(gaze_intersect):
                                self.viewer.update_intersect_async(gaze_intersect)
                            else:
                                self.viewer.update_intersect_async(None)
                        
                        else:

                            all_direction.clear()
                            all_origin.clear()

                            self.label_pub.publish(self.label_msg)
                                                           
                    else:
                        self.finger_pts.clear()
                        self.finger_base = None
                        self.finger_last_output = None
                        self.finger_stable_pos = None

                        self.gaze_pts.clear()
                        self.gaze_base = None
                        self.gaze_last_output = None
                        self.gaze_stable_pos = None

                        self.label_pub.publish(self.label_msg)

                else:
                    rgb_msg = None
                    depth_msg = None
                    self.label_pub.publish(self.label_msg)
                    print("There are not any rgb_msg and depth_msg !")
                
            if not hand_detected and not gaze_detected:
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
