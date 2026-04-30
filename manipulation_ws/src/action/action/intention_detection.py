import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String as Strings
import cv2
import time
import json
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

        self.rgb_buffer = {'left': None, 'right': None}
        self.depth_buffer = {'left': None, 'right': None}

        # Aria gaze buffers
        self.gaze_euler = None
        self.cam_pose = None
        self.gaze_intersect_pos = None
        self.lock = Lock()

        time.sleep(5)

        available_topics = dict(self.get_topic_names_and_types())
        
        self.label_pub = self.create_publisher(Labels, 'label_output', 10)
        self._marker_pub = self.create_publisher(Marker, '/finger/markers', 10)
        self._yolo_bbox_marker_pub = self.create_publisher(Marker, '/yolo/bbox_centers', 10)
        self.hand_img_pub = self.create_publisher(Image, '/hand_landmarks_image', 10)
        self._hand_marker_pub = self.create_publisher(Marker, '/hand/landmarks_markers', 10)
        self._mcp_to_bbox_marker_pub = self.create_publisher(Marker, '/finger/mcp_to_bbox_lines', 10)
        self.label_msg = self._new_label_msg()

        # Aria gaze subscription
        self.create_subscription(PointStamped, '/aria/gaze_xy_intersect', self.gaze_intersect_callback, 10)
        # label_msg from /gaze_label only(aria glass)
        self.create_subscription(Strings, '/gaze_label', self._gaze_label_callback, 1)
        
        #=================================right/left zed camera========================================
        
        # self.create_subscription(CompressedImage, '/zedl/zed_node/rgb/image_rect_color/compressed', lambda msg: self.buffer_callback(msg, 'left', 'rgb'), 10)
        # self.create_subscription(CompressedImage, '/zedr/zed_node/rgb/image_rect_color/compressed', lambda msg: self.buffer_callback(msg, 'right', 'rgb'), 10)
        # self.create_subscription(Image, '/zedl/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'left', 'depth'), 10)
        # self.create_subscription(Image, '/zedr/zed_node/depth/depth_registered', lambda msg: self.buffer_callback(msg, 'right', 'depth'), 10)

        # self.T_wc_l = create_Twc_from_quaternion(translation = np.array([0.836, 0.477, 0.328]), quaternion = np.array([0.212, 0.882, -0.373, -0.196]))
        # self.intrinsics_l = (1060.0899658203125, 1059.0899658203125, 958.9099731445312, 561.5670166015625)

        # self.T_wc_r = create_Twc_from_quaternion(translation = np.array([0.736, 0.540, 0.351]), quaternion = np.array([0.212, 0.882, -0.373, -0.196]))
        # self.intrinsics_r = (1059.9764404296875, 1059.9764404296875, 963.07568359375, 522.3530883789062)
        
        # left_rgb = '/zedl/zed_node/rgb/image_rect_color/compressed'
        # right_rgb = '/zedr/zed_node/rgb/image_rect_color/compressed'
        
        #===================================third realsense camera======================================
        
        self.create_subscription(Image, '/camera/camera/color/image_raw', lambda msg: self.buffer_callback(msg, 'right', 'rgb'), 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', lambda msg: self.buffer_callback(msg, 'right', 'depth'), 10)
        self.T_wc_r = create_Twc_from_quaternion(translation = np.array([0.939, 0.364, 0.967]), quaternion = np.array([0.305, 0.936, -0.106, -0.141]))
        self.intrinsics_r = (603.6532592773438, 602.72119140625, 326.14337158203125, 242.20367431640625)
        
        left_rgb = None
        right_rgb = '/camera/camera/color/image_raw'
        
        #=========================================================================
        
        self.left_camera_active = left_rgb in available_topics
        self.right_camera_active = right_rgb in available_topics
        
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

    def _gaze_label_callback(self, msg):
        with self.lock:
            labels = [label.strip() for label in msg.data.split(',') if label.strip()]
            self.label_msg.gaze_info = self.intention._format_label_info(labels)
            
    def _new_label_msg(self):
        msg = Labels()
        msg.gaze_info = "[]"
        msg.gesture_info = "[]"
        return msg
            
    def buffer_callback(self, msg, side, kind):
        with self.lock:
            print(side, kind)
            print("----------------------")
            if kind == 'rgb':
                self.rgb_buffer[side] = msg
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

    def _publish_hand_landmarks(self, landmarks_world: list) -> None:
        """发布手部21个骨骼点（SPHERE_LIST）和连线（LINE_LIST）到 RViz。"""
        HAND_CONNECTIONS = [
            (0,1),(1,2),(2,3),(3,4),
            (0,5),(5,6),(6,7),(7,8),
            (5,9),(9,10),(10,11),(11,12),
            (9,13),(13,14),(14,15),(15,16),
            (13,17),(17,18),(18,19),(19,20),
            (0,17)
        ]
        stamp = self.get_clock().now().to_msg()

        # --- 关节球 ---
        from geometry_msgs.msg import Point
        sphere_marker = Marker()
        sphere_marker.header.stamp = stamp
        sphere_marker.header.frame_id = "base_link"
        sphere_marker.ns = "hand_joints"
        sphere_marker.id = 10
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.scale.x = 0.012
        sphere_marker.scale.y = 0.012
        sphere_marker.scale.z = 0.012
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.3
        sphere_marker.color.a = 1.0
        sphere_marker.lifetime.nanosec = 300_000_000

        for pt in landmarks_world:
            if pt is not None:
                p = Point()
                p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
                sphere_marker.points.append(p)

        # --- 连线 ---
        line_marker = Marker()
        line_marker.header.stamp = stamp
        line_marker.header.frame_id = "base_link"
        line_marker.ns = "hand_bones"
        line_marker.id = 11
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.005
        line_marker.color.r = 0.2
        line_marker.color.g = 0.6
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        line_marker.lifetime.nanosec = 300_000_000

        for start, end in HAND_CONNECTIONS:
            if landmarks_world[start] is not None and landmarks_world[end] is not None:
                ps = Point()
                ps.x, ps.y, ps.z = float(landmarks_world[start][0]), float(landmarks_world[start][1]), float(landmarks_world[start][2])
                pe = Point()
                pe.x, pe.y, pe.z = float(landmarks_world[end][0]), float(landmarks_world[end][1]), float(landmarks_world[end][2])
                line_marker.points.append(ps)
                line_marker.points.append(pe)

        self._hand_marker_pub.publish(sphere_marker)
        self._hand_marker_pub.publish(line_marker)

    def _publish_bbox_centers(self, points_world: list, namespace: str, marker_id: int, color: tuple) -> None:
        """Publish YOLO bbox center points in world coordinates for RViz."""
        from geometry_msgs.msg import Point

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0
        marker.lifetime.nanosec = 300_000_000

        for pt in points_world:
            if pt is None:
                continue
            p = Point()
            p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
            marker.points.append(p)

        self._yolo_bbox_marker_pub.publish(marker)

    def _publish_mcp_to_bbox_lines(self, mcp_world: np.ndarray, bbox_points_world: list) -> None:
        """Publish LINE_LIST from index finger MCP to each YOLO bbox center, for angle verification."""
        from geometry_msgs.msg import Point

        stamp = self.get_clock().now().to_msg()

        # MCP sphere
        mcp_marker = Marker()
        mcp_marker.header.stamp = stamp
        mcp_marker.header.frame_id = "base_link"
        mcp_marker.ns = "mcp_point"
        mcp_marker.id = 31
        mcp_marker.type = Marker.SPHERE
        mcp_marker.action = Marker.ADD
        mcp_marker.scale.x = 0.025
        mcp_marker.scale.y = 0.025
        mcp_marker.scale.z = 0.025
        mcp_marker.color.r = 1.0
        mcp_marker.color.g = 1.0
        mcp_marker.color.b = 0.0
        mcp_marker.color.a = 1.0
        mcp_marker.pose.position.x = float(mcp_world[0])
        mcp_marker.pose.position.y = float(mcp_world[1])
        mcp_marker.pose.position.z = float(mcp_world[2])
        mcp_marker.pose.orientation.w = 1.0
        mcp_marker.lifetime.nanosec = 300_000_000
        self._mcp_to_bbox_marker_pub.publish(mcp_marker)

        # Lines from MCP to each bbox center
        line_marker = Marker()
        line_marker.header.stamp = stamp
        line_marker.header.frame_id = "base_link"
        line_marker.ns = "mcp_to_bbox"
        line_marker.id = 32
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.006
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.9
        line_marker.lifetime.nanosec = 300_000_000

        mcp_pt = Point()
        mcp_pt.x, mcp_pt.y, mcp_pt.z = float(mcp_world[0]), float(mcp_world[1]), float(mcp_world[2])
        for pt in bbox_points_world:
            if pt is None:
                continue
            bbox_pt = Point()
            bbox_pt.x, bbox_pt.y, bbox_pt.z = float(pt[0]), float(pt[1]), float(pt[2])
            line_marker.points.append(mcp_pt)
            line_marker.points.append(bbox_pt)

        self._mcp_to_bbox_marker_pub.publish(line_marker)

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

    def _run_finger_detection(self, direction, origin, rgb_msg, image_name, camera_side, depth_msg=None, camera_intrinsics=None, T_wc=None, finger_tip_world=None):
        """Run YOLO + 3D bbox detection. direction/origin optional: when present, also compute pointing score."""
        if direction is not None and origin is not None:
            if np.isnan(direction).any() or np.isnan(origin).any():
                direction, origin, finger_tip_world = None, None, None
            elif np.linalg.norm(direction) <= 1e-6:
                direction, origin, finger_tip_world = None, None, None
            else:
                direction = direction / np.linalg.norm(direction)

        (self.finger_stable_pos,
         self.finger_last_output,
         self.finger_pts,
         self.finger_base,
         _, finger_origin_ema,
         finger_intersect,
         finger_label_output,
         finger_score_output,
         finger_bbox_world_points) = self.intention.process_detection(
            direction, origin, rgb_msg, self.finger_pts,
            direction_name="finger_direction", origin_name="finger_origin",
            image_name=image_name, camera_side=camera_side,
            depth_msg=depth_msg, camera_intrinsics=camera_intrinsics, T_wc=T_wc,
            finger_tip_world=finger_tip_world)

        self.label_msg.gesture_info = self.intention._format_label_info(finger_label_output, finger_score_output)
        self._publish_bbox_centers(finger_bbox_world_points, "gesture_yolo_bbox_centers", 20, (1.0, 0.35, 0.05))

        if finger_origin_ema is not None and finger_bbox_world_points:
            self._publish_mcp_to_bbox_lines(finger_origin_ema, finger_bbox_world_points)

        for label, score, pt in zip(finger_label_output, finger_score_output, finger_bbox_world_points):
            pt_str = f"({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})" if pt is not None else "None"
            print(f"  bbox: label={label}, score={score:.4f}, 3d={pt_str}")

        return finger_intersect

    def _run_gaze_detection(self, rgb_msg, image_name, camera_side, depth_msg=None, camera_intrinsics=None, T_wc=None):
        """Run gaze detection if gaze intersection data is available."""
        if self.gaze_intersect_pos is None:
            return

        (self.gaze_stable_pos,
         self.gaze_last_output,
         self.gaze_pts,
         self.gaze_base,
         _, _, _,
         gaze_label_output,
         gaze_score_output,
         gaze_bbox_world_points) = self.intention.process_detection(
            rgb_msg=rgb_msg, pts=self.gaze_pts,
            direction_name="gaze_direction", origin_name="gaze_origin",
            image_name=image_name, camera_side=camera_side,
            intersect=self.gaze_intersect_pos,
            depth_msg=depth_msg, camera_intrinsics=camera_intrinsics, T_wc=T_wc)

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
        finger_intersect = None
        finger_direction = None
        finger_origin = None
        finger_tip_world = None

        if self.left_camera_active and self.right_camera_active:
            rgb_msg_l = self.rgb_buffer['left']
            depth_msg_l = self.depth_buffer['left']
            rgb_msg_r = self.rgb_buffer['right']
            depth_msg_r = self.depth_buffer['right']

            if any(x is None for x in [rgb_msg_l, depth_msg_l, rgb_msg_r, depth_msg_r]):
                self._reset_detection_state()
                self.label_pub.publish(self.label_msg)
                return

            dir_l, orig_l, hand_img_l, lm_world_l, tip_l = self.intention.get_hand_pose(rgb_msg_l, depth_msg_l, self.T_wc_l, self.intrinsics_l)
            dir_r, orig_r, hand_img_r, lm_world_r, tip_r = self.intention.get_hand_pose(rgb_msg_r, depth_msg_r, self.T_wc_r, self.intrinsics_r)
            hand_img = hand_img_r if hand_img_r is not None else hand_img_l
            if hand_img is not None:
                self.hand_img_pub.publish(self.bridge.cv2_to_imgmsg(hand_img, encoding='bgr8'))
            lm_world = lm_world_r if lm_world_r is not None else lm_world_l
            if lm_world is not None:
                self._publish_hand_landmarks(lm_world)
            finger_direction, finger_origin = self._merge_finger_detections(dir_l, orig_l, dir_r, orig_r)
            finger_tip_world = tip_r if tip_r is not None else tip_l

            rgb_msg = [rgb_msg_r, rgb_msg_l]
            depth_msg = [depth_msg_r, depth_msg_l]
            intrinsics = [self.intrinsics_r, self.intrinsics_l]
            T_wc = [self.T_wc_r, self.T_wc_l]
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

            finger_direction, finger_origin, hand_img, lm_world, finger_tip_world = self.intention.get_hand_pose(
                rgb_msg, depth_msg, T_wc, intrinsics)
            if hand_img is not None:
                self.hand_img_pub.publish(self.bridge.cv2_to_imgmsg(hand_img, encoding='bgr8'))
            if lm_world is not None:
                self._publish_hand_landmarks(lm_world)

            finger_img = f'gesture_yolo_{suffix}.png'
            gaze_img = f'gaze_yolo_{suffix}.png'
            cam_side = side

        else:
            self.label_pub.publish(self.label_msg)
            print("There are not any rgb_msg and depth_msg !")
            return

        # Always run finger detection for YOLO bbox 3D coords; direction/origin may be None when no hand
        finger_intersect = self._run_finger_detection(
            finger_direction, finger_origin, rgb_msg, finger_img, cam_side,
            depth_msg=depth_msg, camera_intrinsics=intrinsics, T_wc=T_wc,
            finger_tip_world=finger_tip_world)

        # Process gaze detection
        if self.gaze_intersect_pos is not None:
            self._run_gaze_detection(rgb_msg, gaze_img, cam_side,
                                     depth_msg=depth_msg, camera_intrinsics=intrinsics, T_wc=T_wc)

        # Publish labels and markers
        self.label_pub.publish(self.label_msg)
        print(
            f"Published labels: gesture_info={self.label_msg.gesture_info}, "
            f"gaze_info={self.label_msg.gaze_info}"
        )
        self.label_msg = self._new_label_msg()
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
