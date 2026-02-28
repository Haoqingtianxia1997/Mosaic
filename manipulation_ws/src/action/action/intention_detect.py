import time
from collections import deque
from threading import Thread

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image

from action_interfaces.msg import Labels
from intention_utils.intention import Intention


def create_Twc_from_quaternion(translation: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    assert translation.shape == (3,), "translation must be a vector of shape (3,)"
    assert quaternion.shape == (4,), "quaternion must be a vector of shape (4,)"

    rot = R.from_quat(quaternion)
    rotation_wc = rot.as_matrix()

    transform_wc = np.eye(4)
    transform_wc[:3, :3] = rotation_wc
    transform_wc[:3, 3] = translation
    return transform_wc


class IntentionDetectNode(Node):
    def __init__(self):
        super().__init__('intention_detect_node')
        self.bridge = CvBridge()
        self.intention = Intention()

        self.rgb_buffer = {'left': None, 'right': None}
        self.depth_buffer = {'left': None, 'right': None}
        self.aria_cam_pose = None
        self.aria_gaze_euler = None
        self.aria_gaze_x_offset = 0.05

        time.sleep(5)
        available_topics = dict(self.get_topic_names_and_types())
        left_rgb = '/zedl/zed_node/rgb/image_rect_color'
        right_rgb = '/zedr/zed_node/rgb/image_rect_color'
        self.left_camera_active = left_rgb in available_topics
        self.right_camera_active = right_rgb in available_topics

        self.label_pub = self.create_publisher(Labels, 'label_output', 10)
        self.finger_intersect_pub = self.create_publisher(PointStamped, 'finger_intersect', 10)
        self.gaze_intersect_pub = self.create_publisher(PointStamped, 'gaze_intersect', 10)

        self.create_subscription(
            Image,
            '/zedl/zed_node/rgb/image_rect_color',
            lambda msg: self.buffer_callback(msg, 'left', 'rgb'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/zedr/zed_node/rgb/image_rect_color',
            lambda msg: self.buffer_callback(msg, 'right', 'rgb'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/zedl/zed_node/depth/depth_registered',
            lambda msg: self.buffer_callback(msg, 'left', 'depth'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/zedr/zed_node/depth/depth_registered',
            lambda msg: self.buffer_callback(msg, 'right', 'depth'),
            qos_profile_sensor_data,
        )
        self.create_subscription(PoseStamped, '/aria/cam_pose', self.aria_cam_pose_callback, 10)
        self.create_subscription(Vector3, '/aria/gaze_euler', self.aria_gaze_euler_callback, 10)

        self.get_logger().info('Intention detect node started (no Open3D viewer)')

        self.T_wc_l = create_Twc_from_quaternion(
            translation=np.array([0.114, -0.442, 0.546]),
            quaternion=np.array([0.855, -0.410, 0.125, -0.292]),
        )
        self.intrinsics_l = (1060.0899658203125, 1059.0899658203125, 958.9099731445312, 561.5670166015625)

        self.T_wc_r = create_Twc_from_quaternion(
            translation=np.array([0.862, 0.458, 0.587]),
            quaternion=np.array([0.553, 0.749, -0.280, -0.233]),
        )
        self.intrinsics_r = (1059.9764404296875, 1059.9764404296875, 963.07568359375, 522.3530883789062)

        self.finger_pts = deque()
        self.finger_base = None
        self.finger_last_output = None
        self.finger_stable_pos = None

        self.gaze_pts = deque()
        self.gaze_base = None
        self.gaze_last_output = None
        self.gaze_stable_pos = None

        Thread(target=self.monitor_pair_thread, daemon=True).start()

    def buffer_callback(self, msg, side, kind):
        if kind == 'rgb':
            self.rgb_buffer[side] = msg
        elif kind == 'depth':
            self.depth_buffer[side] = msg

    def aria_cam_pose_callback(self, msg):
        self.aria_cam_pose = msg

    def aria_gaze_euler_callback(self, msg):
        self.aria_gaze_euler = msg

    def get_aria_gaze(self, cam_pose=None, gaze_euler=None):
        cam_pose = self.aria_cam_pose if cam_pose is None else cam_pose
        gaze_euler = self.aria_gaze_euler if gaze_euler is None else gaze_euler
        return self.intention.get_gaze_direction(
            cam_pose,
            gaze_euler,
            gaze_x_offset=self.aria_gaze_x_offset,
        )

    @staticmethod
    def _is_valid_direction_origin(direction, origin):
        return (
            direction is not None
            and origin is not None
            and not np.isnan(direction).any()
            and not np.isnan(origin).any()
            and np.linalg.norm(direction) > 1e-6
        )

    def _process_finger(self, direction, origin, rgb_input, image_name, camera_side):
        direction = direction / np.linalg.norm(direction)
        (
            self.finger_stable_pos,
            self.finger_last_output,
            self.finger_pts,
            self.finger_base,
            _,
            _,
            finger_intersect,
            finger_label_output,
        ) = self.intention.process_detection(
            direction,
            origin,
            rgb_input,
            self.finger_pts,
            direction_name='finger_direction',
            origin_name='finger_origin',
            image_name=image_name,
            camera_side=camera_side,
        )
        return finger_intersect, finger_label_output

    def _process_gaze(self, direction, origin, rgb_input, image_name, camera_side):
        direction = direction / np.linalg.norm(direction)
        (
            self.gaze_stable_pos,
            self.gaze_last_output,
            self.gaze_pts,
            self.gaze_base,
            _,
            _,
            gaze_intersect,
            gaze_label_output,
        ) = self.intention.process_detection(
            direction,
            origin,
            rgb_input,
            self.gaze_pts,
            direction_name='gaze_direction',
            origin_name='gaze_origin',
            image_name=image_name,
            camera_side=camera_side,
        )
        return gaze_intersect, gaze_label_output

    def _publish_intersect_point(self, publisher, intersect):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        if intersect is not None and self.intention.in_valid_area(intersect):
            msg.point.x = float(intersect[0])
            msg.point.y = float(intersect[1])
            msg.point.z = float(intersect[2])
        else:
            msg.point.x = float('nan')
            msg.point.y = float('nan')
            msg.point.z = float('nan')
        publisher.publish(msg)

    def _reset_state(self):
        self.finger_pts.clear()
        self.finger_base = None
        self.finger_last_output = None
        self.finger_stable_pos = None

        self.gaze_pts.clear()
        self.gaze_base = None
        self.gaze_last_output = None
        self.gaze_stable_pos = None

    def _snapshot_inputs(self):
        return {
            'rgb_left': self.rgb_buffer['left'],
            'rgb_right': self.rgb_buffer['right'],
            'depth_left': self.depth_buffer['left'],
            'depth_right': self.depth_buffer['right'],
            'aria_cam_pose': self.aria_cam_pose,
            'aria_gaze_euler': self.aria_gaze_euler,
            'left_camera_active': self.left_camera_active,
            'right_camera_active': self.right_camera_active,
        }

    def monitor_pair_thread(self):
        while rclpy.ok():
            snapshot = self._snapshot_inputs()

            label_msg = Labels()
            finger_intersect = None
            gaze_intersect = None

            dual_mode = snapshot['left_camera_active'] and snapshot['right_camera_active']
            left_ready = snapshot['rgb_left'] is not None and snapshot['depth_left'] is not None
            right_ready = snapshot['rgb_right'] is not None and snapshot['depth_right'] is not None

            if dual_mode:
                if not (left_ready and right_ready):
                    self._reset_state()
                    self.label_pub.publish(label_msg)
                    self._publish_intersect_point(self.finger_intersect_pub, None)
                    self._publish_intersect_point(self.gaze_intersect_pub, None)
                    time.sleep(0.02)
                    continue

                rgb_msg_l = snapshot['rgb_left']
                rgb_msg_r = snapshot['rgb_right']
                depth_msg_l = snapshot['depth_left']
                depth_msg_r = snapshot['depth_right']

                finger_direction_l, finger_origin_l = self.intention.get_hand_pose(
                    rgb_msg_l, depth_msg_l, self.T_wc_l, self.intrinsics_l
                )
                finger_direction_r, finger_origin_r = self.intention.get_hand_pose(
                    rgb_msg_r, depth_msg_r, self.T_wc_r, self.intrinsics_r
                )

                finger_direction = None
                finger_origin = None
                if finger_direction_l is not None and finger_origin_l is not None and finger_direction_r is not None and finger_origin_r is not None:
                    finger_direction = (finger_direction_l + finger_direction_r) / 2.0
                    finger_origin = (finger_origin_l + finger_origin_r) / 2.0
                elif finger_direction_l is not None and finger_origin_l is not None:
                    finger_direction = finger_direction_l
                    finger_origin = finger_origin_l
                elif finger_direction_r is not None and finger_origin_r is not None:
                    finger_direction = finger_direction_r
                    finger_origin = finger_origin_r

                if self._is_valid_direction_origin(finger_direction, finger_origin):
                    finger_intersect, finger_labels = self._process_finger(
                        finger_direction,
                        finger_origin,
                        [rgb_msg_r, rgb_msg_l],
                        ['gesture_yolo_r.png', 'gesture_yolo_l.png'],
                        ['right', 'left'],
                    )
                    label_msg.gesture_labels = finger_labels

                gaze_direction, gaze_origin = self.get_aria_gaze(snapshot['aria_cam_pose'], snapshot['aria_gaze_euler'])
                if self._is_valid_direction_origin(gaze_direction, gaze_origin):
                    gaze_intersect, gaze_labels = self._process_gaze(
                        gaze_direction,
                        gaze_origin,
                        [rgb_msg_r, rgb_msg_l],
                        ['gaze_yolo_r.png', 'gaze_yolo_l.png'],
                        ['right', 'left'],
                    )
                    label_msg.gaze_labels = gaze_labels

            elif snapshot['right_camera_active']:
                if not right_ready:
                    self._reset_state()
                    self.label_pub.publish(label_msg)
                    self._publish_intersect_point(self.finger_intersect_pub, None)
                    self._publish_intersect_point(self.gaze_intersect_pub, None)
                    time.sleep(0.02)
                    continue

                rgb_msg = snapshot['rgb_right']
                depth_msg = snapshot['depth_right']
                finger_direction, finger_origin = self.intention.get_hand_pose(
                    rgb_msg, depth_msg, self.T_wc_r, self.intrinsics_r
                )
                if self._is_valid_direction_origin(finger_direction, finger_origin):
                    finger_intersect, finger_labels = self._process_finger(
                        finger_direction,
                        finger_origin,
                        rgb_msg,
                        'gesture_yolo_r.png',
                        'right',
                    )
                    label_msg.gesture_labels = finger_labels

                gaze_direction, gaze_origin = self.get_aria_gaze(snapshot['aria_cam_pose'], snapshot['aria_gaze_euler'])
                if self._is_valid_direction_origin(gaze_direction, gaze_origin):
                    gaze_intersect, gaze_labels = self._process_gaze(
                        gaze_direction,
                        gaze_origin,
                        rgb_msg,
                        'gaze_yolo_r.png',
                        'right',
                    )
                    label_msg.gaze_labels = gaze_labels

            elif snapshot['left_camera_active']:
                if not left_ready:
                    self._reset_state()
                    self.label_pub.publish(label_msg)
                    self._publish_intersect_point(self.finger_intersect_pub, None)
                    self._publish_intersect_point(self.gaze_intersect_pub, None)
                    time.sleep(0.02)
                    continue

                rgb_msg = snapshot['rgb_left']
                depth_msg = snapshot['depth_left']
                finger_direction, finger_origin = self.intention.get_hand_pose(
                    rgb_msg, depth_msg, self.T_wc_l, self.intrinsics_l
                )
                if self._is_valid_direction_origin(finger_direction, finger_origin):
                    finger_intersect, finger_labels = self._process_finger(
                        finger_direction,
                        finger_origin,
                        rgb_msg,
                        'gesture_yolo_l.png',
                        'left',
                    )
                    label_msg.gesture_labels = finger_labels

                gaze_direction, gaze_origin = self.get_aria_gaze(snapshot['aria_cam_pose'], snapshot['aria_gaze_euler'])
                if self._is_valid_direction_origin(gaze_direction, gaze_origin):
                    gaze_intersect, gaze_labels = self._process_gaze(
                        gaze_direction,
                        gaze_origin,
                        rgb_msg,
                        'gaze_yolo_l.png',
                        'left',
                    )
                    label_msg.gaze_labels = gaze_labels

            else:
                self._reset_state()

            self.label_pub.publish(label_msg)
            self._publish_intersect_point(self.finger_intersect_pub, finger_intersect)
            self._publish_intersect_point(self.gaze_intersect_pub, gaze_intersect)

            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = IntentionDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()