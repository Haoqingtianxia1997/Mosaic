import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque
import cv2
import os
import numpy as np

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()

        self.rgb_queues = {'l': deque(maxlen=1), 'r': deque(maxlen=1)}
        self.depth_queues = {'l': deque(maxlen=1), 'r': deque(maxlen=1)}

        self.create_subscription(Image, '/zedl/zed_node/rgb/image_rect_color', self.make_rgb_cb('l'), 10)
        self.create_subscription(Image, '/zedr/zed_node/rgb/image_rect_color', self.make_rgb_cb('r'), 10)

        self.create_subscription(Image, '/zedl/zed_node/depth/depth_registered', self.make_depth_cb('l'), 10)
        self.create_subscription(Image, '/zedr/zed_node/depth/depth_registered', self.make_depth_cb('r'), 10)

        self.output_dir = os.path.join(os.getcwd(), 'saved_images')
        os.makedirs(self.output_dir, exist_ok=True)

        self.create_timer(10.0, self.auto_save)
        self.get_logger().info("ImageAutoSaver started: will save every 10 seconds (overwrite).")

    def make_rgb_cb(self, cam):
        def cb(msg):
            try:
                img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.rgb_queues[cam].append(img)
            except Exception as e:
                self.get_logger().error(f"[{cam}] RGB error: {e}")
        return cb

    def make_depth_cb(self, cam):
        def cb(msg):
            try:
                depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                self.depth_queues[cam].append(depth)
            except Exception as e:
                self.get_logger().error(f"[{cam}] Depth error: {e}")
        return cb

    def auto_save(self):
        for cam in ['l', 'r']:
            if not self.rgb_queues[cam] or not self.depth_queues[cam]:
                self.get_logger().warn(f"Skipping {cam}: missing data")
                continue

            rgb = self.rgb_queues[cam][-1]
            depth = self.depth_queues[cam][-1]

            rgb_path = os.path.join(self.output_dir, f"{cam}_rgb.png")
            depth_npy_path = os.path.join(self.output_dir, f"{cam}_depth.npy")
            depth_png_path = os.path.join(self.output_dir, f"{cam}_depth.png")

            try:
                cv2.imwrite(rgb_path, rgb)
                np.save(depth_npy_path, depth)

                depth_vis = np.clip(depth, 0, 3.0)
                depth_vis = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
                cv2.imwrite(depth_png_path, depth_vis)

                self.get_logger().info(f"Saved {cam} RGB + Depth")
            except Exception as e:
                self.get_logger().error(f"[{cam}] Failed to save: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
