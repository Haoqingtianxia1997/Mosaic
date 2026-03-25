import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.tasks.python import BaseOptions
import time
from threading import Thread, Lock
from collections import deque
import os
from intention_utils.intention import Intention
from action_interfaces.msg import Labels
from geometry_msgs.msg import Vector3, PoseStamped, PointStamped
from visualization_msgs.msg import Marker


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

        self.output_dir = './saved_images'
        os.makedirs(self.output_dir, exist_ok=True)

        hand_model_path = os.path.join(os.path.dirname(__file__), 'intention_utils', 'models', 'hand_landmarker.task')
        hand_options = mp_vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=hand_model_path),
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.hands_detector = mp_vision.HandLandmarker.create_from_options(hand_options)
        self.mp_drawing = mp_vision.drawing_utils
        self.hand_connections = mp_vision.HandLandmarksConnections.HAND_CONNECTIONS

        self.rgb_buffer = {'left': None, 'right': None}
        self.depth_buffer = {'left': None, 'right': None}

        # Aria gaze buffers
        self.gaze_euler = None
        self.cam_pose = None
        self.gaze_intersect_pos = None
        self.lock = Lock()

        time.sleep(5)

        available_topics = dict(self.get_topic_names_and_types())
        left_rgb = '/zedl/zed_node/rgb/image_rect_color/compressed'
        right_rgb = '/zedr/zed_node/rgb/image_rect_color/compressed'
        self.left_camera_active = left_rgb in available_topics
        self.right_camera_active = right_rgb in available_topics

        self.label_pub = self.create_publisher(Labels, 'label_output', 10)
        self._marker_pub = self.create_publisher(Marker, '/finger/markers', 10)
        self.label_msg = Labels()

        # Aria gaze subscription
        self.create_subscription(PointStamped, '/aria/gaze_xy_intersect', self.gaze_intersect_callback, 10)

        self.create_subscription(CompressedImage, '/zedl/zed_node/rgb/image_rect_color/compressed', lambda msg: self.buffer_callback(msg, 'left', 'rgb'), 10)
        self.create_subscription(CompressedImage, '/zedr/zed_node/rgb/image_rect_color/compressed', lambda msg: self.buffer_callback(msg, 'right', 'rgb'), 10)
        self.create_subscription(Image, '/zedl/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'left', 'depth'), 10)
        self.create_subscription(Image, '/zedr/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'right', 'depth'), 10)


        self.get_logger().info("🖐️ Hand detection + point cloud visualization node started")

        self.T_wc_l = create_Twc_from_quaternion(translation = np.array([0.078, -0.466, 0.554]), quaternion = np.array([0.837, -0.378, 0.154, -0.364]))
        self.intrinsics_l = (1060.0899658203125, 1059.0899658203125, 958.9099731445312, 561.5670166015625)

        self.T_wc_r = create_Twc_from_quaternion(translation = np.array([0.832, 0.468, 0.587]), quaternion = np.array([0.553, 0.749, -0.280, -0.233]))
        self.intrinsics_r = (1059.9764404296875, 1059.9764404296875, 963.07568359375, 522.3530883789062)

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

    def gaze_euler_callback(self, msg):
        with self.lock:
            self.gaze_euler = msg

    def cam_pose_callback(self, msg):
        with self.lock:
            self.cam_pose = msg

    def gaze_intersect_callback(self, msg):
        with self.lock:
            self.gaze_intersect_pos = np.array([msg.point.x, msg.point.y, msg.point.z])

    def buffer_callback(self, msg, side, kind):
        with self.lock:
            print(side, kind)
            print("----------------------")
            if kind == 'rgb':
                self.rgb_buffer[side] = msg
                img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
                results = self.hands_detector.detect(mp_image)
                if results.hand_landmarks:
                    for hand_landmarks in results.hand_landmarks:
                        self.mp_drawing.draw_landmarks(img, hand_landmarks, self.hand_connections)
                out_path = os.path.join(self.output_dir, f'{side}_gesture_result.png')
                cv2.imwrite(out_path, img)
            elif kind == 'depth':
                self.depth_buffer[side] = msg

    def _publish_finger_ray(self, direction: np.ndarray, origin: np.ndarray) -> None:
        ray_length = 1.0
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "finger"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 1.0
        start_pt = PointStamped().point
        start_pt.x = float(origin[0])
        start_pt.y = float(origin[1])
        start_pt.z = float(origin[2])
        end_pt = PointStamped().point
        end_pt.x = float(origin[0] + direction[0] * ray_length)
        end_pt.y = float(origin[1] + direction[1] * ray_length)
        end_pt.z = float(origin[2] + direction[2] * ray_length)
        marker.points = [start_pt, end_pt]
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200_000_000
        self._marker_pub.publish(marker)

    def _publish_finger_hit(self, hit: np.ndarray) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "finger"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.pose.position.x = float(hit[0])
        marker.pose.position.y = float(hit[1])
        marker.pose.position.z = float(hit[2])
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200_000_000
        self._marker_pub.publish(marker)

    def _reset_detection_state(self):
        """Reset all finger and gaze detection state."""
        self.finger_pts.clear()
        self.finger_base = None
        self.finger_last_output = None
        self.finger_stable_pos = None

        self.gaze_pts.clear()
        self.gaze_base = None
        self.gaze_last_output = None
        self.gaze_stable_pos = None

    def _run_finger_detection(self, direction, origin, rgb_msg, image_name, camera_side):
        """Validate direction, normalize, and run finger process_detection.

        Returns finger_intersect point, or None if validation fails.
        """
        if np.isnan(direction).any() or np.isnan(origin).any():
            return None
        if np.linalg.norm(direction) <= 1e-6:
            return None

        normalized = direction / np.linalg.norm(direction)

        (self.finger_stable_pos,
         self.finger_last_output,
         self.finger_pts,
         self.finger_base,
         _, _,
         finger_intersect,
         finger_label_output) = self.intention.process_detection(
            normalized, origin, rgb_msg, self.finger_pts,
            direction_name="finger_direction", origin_name="finger_origin",
            image_name=image_name, camera_side=camera_side)

        self.label_msg.gesture_labels = finger_label_output
        return finger_intersect

    def _run_gaze_detection(self, rgb_msg, image_name, camera_side):
        """Run gaze detection if gaze intersection data is available."""
        if self.gaze_intersect_pos is None:
            return

        (self.gaze_stable_pos,
         self.gaze_last_output,
         self.gaze_pts,
         self.gaze_base,
         _, _, _,
         gaze_label_output) = self.intention.process_detection(
            rgb_msg=rgb_msg, pts=self.gaze_pts,
            direction_name="gaze_direction", origin_name="gaze_origin",
            image_name=image_name, camera_side=camera_side,
            intersect=self.gaze_intersect_pos)

        self.label_msg.gaze_labels = gaze_label_output

    def _merge_finger_detections(self, dir_l, orig_l, dir_r, orig_r):
        """Merge finger detections from two cameras by averaging available results."""
        has_l = dir_l is not None and orig_l is not None
        has_r = dir_r is not None and orig_r is not None

        if has_l and has_r:
            return (dir_l + dir_r) / 2, (orig_l + orig_r) / 2
        elif has_l:
            return dir_l, orig_l
        elif has_r:
            return dir_r, orig_r
        return None, None

    def _process_frame(self):
        """Process a single frame: detect fingers/gaze and publish results."""
        self.label_msg = Labels()
        finger_intersect = None
        finger_direction = None
        finger_origin = None

        if self.left_camera_active and self.right_camera_active:
            rgb_msg_l = self.rgb_buffer['left']
            depth_msg_l = self.depth_buffer['left']
            rgb_msg_r = self.rgb_buffer['right']
            depth_msg_r = self.depth_buffer['right']

            if any(x is None for x in [rgb_msg_l, depth_msg_l, rgb_msg_r, depth_msg_r]):
                self._reset_detection_state()
                self.label_pub.publish(self.label_msg)
                return

            dir_l, orig_l = self.intention.get_hand_pose(rgb_msg_l, depth_msg_l, self.T_wc_l, self.intrinsics_l)
            dir_r, orig_r = self.intention.get_hand_pose(rgb_msg_r, depth_msg_r, self.T_wc_r, self.intrinsics_r)
            finger_direction, finger_origin = self._merge_finger_detections(dir_l, orig_l, dir_r, orig_r)

            rgb_msg = [rgb_msg_r, rgb_msg_l]
            finger_img = ['gesture_yolo_r.png', 'gesture_yolo_l.png']
            gaze_img = ['gaze_yolo_r.png', 'gaze_yolo_l.png']
            cam_side = ['right', 'left']

        elif self.right_camera_active or self.left_camera_active:
            side = 'right' if self.right_camera_active else 'left'
            rgb_msg = self.rgb_buffer[side]
            depth_msg = self.depth_buffer[side]

            if rgb_msg is None or depth_msg is None:
                self._reset_detection_state()
                self.label_pub.publish(self.label_msg)
                return

            T_wc = self.T_wc_r if side == 'right' else self.T_wc_l
            intrinsics = self.intrinsics_r if side == 'right' else self.intrinsics_l
            suffix = 'r' if side == 'right' else 'l'

            finger_direction, finger_origin = self.intention.get_hand_pose(
                rgb_msg, depth_msg, T_wc, intrinsics)

            finger_img = f'gesture_yolo_{suffix}.png'
            gaze_img = f'gaze_yolo_{suffix}.png'
            cam_side = side

        else:
            self.label_pub.publish(self.label_msg)
            print("There are not any rgb_msg and depth_msg !")
            return

        # Process finger detection
        if finger_direction is not None and finger_origin is not None:
            finger_intersect = self._run_finger_detection(
                finger_direction, finger_origin, rgb_msg, finger_img, cam_side)

        # Process gaze detection
        self._run_gaze_detection(rgb_msg, gaze_img, cam_side)

        # Publish labels and markers
        self.label_pub.publish(self.label_msg)
        if self.intention.in_valid_area(finger_intersect):
            self._publish_finger_ray(finger_direction, finger_origin)
            self._publish_finger_hit(finger_intersect)

    def monitor_pair_thread(self):
        while rclpy.ok():
            with self.lock:
                self._process_frame()
            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionWithPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
