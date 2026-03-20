import os
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image


def atomic_imwrite(path: str, img: np.ndarray, params=None) -> None:
    base, ext = os.path.splitext(path)
    tmp = base + ".tmp" + ext

    if params is None:
        ok = cv2.imwrite(tmp, img)
    else:
        ok = cv2.imwrite(tmp, img, params)

    if not ok:
        raise RuntimeError(f"Failed to write image to temp file {tmp}")

    os.replace(tmp, path)


def atomic_npsave(path: str, arr: np.ndarray) -> None:
    base, ext = os.path.splitext(path)
    tmp = base + ".tmp" + ext

    with open(tmp, 'wb') as f:
        np.save(f, arr)

    os.replace(tmp, path)



class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.bridge = CvBridge()
        self.latest_right = None
        self.latest_left = None

        self.declare_parameter(
            'output_dir', f'/home/mosaic/mosaic/manipulation_ws/saved_images')
        self.output_dir = self.get_parameter(
            'output_dir').get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)

        # Subscribe cameras
        self.r_rgb_sub = Subscriber(self, Image,
            '/zedr/zed_node/rgb/image_rect_color')
        self.r_depth_sub = Subscriber(self, Image,
            '/zedr/zed_node/depth/depth_registered')
        self.l_rgb_sub = Subscriber(self, Image,
            '/zedl/zed_node/rgb/image_rect_color')
        self.l_depth_sub = Subscriber(self, Image,
            '/zedl/zed_node/depth/depth_registered')

        # Sync
        self.ts_right = ApproximateTimeSynchronizer(
            [self.r_rgb_sub, self.r_depth_sub], queue_size=10, slop=0.1)
        self.ts_right.registerCallback(self.callback_right)

        self.ts_left = ApproximateTimeSynchronizer(
            [self.l_rgb_sub, self.l_depth_sub], queue_size=10, slop=0.1)
        self.ts_left.registerCallback(self.callback_left)

        # Save every 10s
        self.timer = self.create_timer(3.0, self.save_images_callback)

        self.get_logger().info('ImageSaver node started (atomic writes).')

    def callback_right(self, rgb_msg, depth_msg):
        self.latest_right = (
            self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8'),
            self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        )

    def callback_left(self, rgb_msg, depth_msg):
        self.latest_left = (
            self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8'),
            self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        )

    def save_images_callback(self):
        if self.latest_right:
            self.save_images(*self.latest_right, prefix='r_')
        else:
            self.get_logger().warn("No right camera images yet.")

        if self.latest_left:
            self.save_images(*self.latest_left, prefix='l_')
        else:
            self.get_logger().warn("No left camera images yet.")

    def save_images(self, rgb_image: np.ndarray, depth_image: np.ndarray,
                    prefix=''):

        rgb_path = os.path.join(self.output_dir, f'{prefix}rgb.png')
        depth_vis_path = os.path.join(self.output_dir, f'{prefix}depth.png')
        depth_raw_path = os.path.join(self.output_dir, f'{prefix}depth.npy')

        # --- RGB ---
        atomic_imwrite(rgb_path, rgb_image,
            [cv2.IMWRITE_PNG_COMPRESSION, 0])

        # --- RAW depth ---
        atomic_npsave(depth_raw_path, depth_image)

        # --- Depth visualization ---
        depth_clip = np.nan_to_num(
            np.clip(depth_image, 0, 3.0),
            nan=0.0, posinf=3.0, neginf=0.0
        )
        depth_vis = cv2.normalize(
            depth_clip, None, 0, 255, cv2.NORM_MINMAX
        ).astype('uint8')

        atomic_imwrite(depth_vis_path, depth_vis)

        # --- Timestamp ---
        timestamp, _ = self.get_clock().now().seconds_nanoseconds()
        self.get_logger().info(f'Saved {prefix} images at {timestamp}')

        self._write_timestamp_file(
            os.path.join(self.output_dir, f'{prefix}timestamp.txt'),
            timestamp
        )

    @staticmethod
    def _write_timestamp_file(path: str, timestamp: int) -> None:
        tmp = path + ".tmp"
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write(str(timestamp))
        os.replace(tmp, path)


def main(args: Optional[Any] = None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()