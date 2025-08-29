import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from realsense2_camera_msgs.msg import RGBD

class RGBDepthSaver(Node):
    def __init__(self):
        super().__init__('rgb_depth_saver')

        self.bridge = CvBridge()
        # Record last saved time for each camera
        self.last_saved_time_r_ = 0
        self.last_saved_time_l_ = 0
        self.last_saved_time_rgbd_ = 0

        # Create save directory
        self.output_dir = os.path.join(os.getcwd(), 'saved_images')
        os.makedirs(self.output_dir, exist_ok=True)

        # Subscribe to RGB and Depth images from left and right cameras
        self.r_rgb_sub = Subscriber(self, Image, '/zedr/zed_node/rgb/image_rect_color')
        self.r_depth_sub = Subscriber(self, Image, '/zedr/zed_node/depth/depth_registered')
        self.l_rgb_sub = Subscriber(self, Image, '/zedl/zed_node/rgb/image_rect_color')
        self.l_depth_sub = Subscriber(self, Image, '/zedl/zed_node/depth/depth_registered')

        self.rgbd = Subscriber(self, RGBD, '/camera/camera/rgbd')

        # Create synchronizers for left and right cameras
        self.ts_right = ApproximateTimeSynchronizer([self.r_rgb_sub, self.r_depth_sub], queue_size=10, slop=0.1)
        self.ts_right.registerCallback(self.callback_right)

        self.ts_left = ApproximateTimeSynchronizer([self.l_rgb_sub, self.l_depth_sub], queue_size=10, slop=0.1)
        self.ts_left.registerCallback(self.callback_left)

        self.ts_rgbd = ApproximateTimeSynchronizer([self.rgbd], queue_size=10, slop=0.1)
        self.ts_rgbd.registerCallback(self.callback_rgbd)

        self.get_logger().info('RGBDepthSaver node has been started.')

    def callback_right(self, rgb_msg, depth_msg):
        self.save_images(rgb_msg, depth_msg, prefix='r_')

    def callback_left(self, rgb_msg, depth_msg):
        self.save_images(rgb_msg, depth_msg, prefix='l_')

    def callback_rgbd(self, rgbd_msg):
        rgb_msg = rgbd_msg.rgb
        depth_msg = rgbd_msg.depth
        self.save_images(rgb_msg, depth_msg, prefix='rgbd_')

    def save_images(self, rgb_msg, depth_msg, prefix=''):
        current_time = time.time()
        # Save every 10 seconds for each camera
        last_saved_attr = f'last_saved_time_{prefix}'
        last_saved = getattr(self, last_saved_attr)
        if current_time - last_saved >= 1.0:
            try:
                # Convert to OpenCV images
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

                # Construct file paths (no timestamp, always overwrite)
                rgb_path = os.path.join(self.output_dir, f'{prefix}rgb.png')
                depth_vis_path = os.path.join(self.output_dir, f'{prefix}depth.png')
                depth_raw_path = os.path.join(self.output_dir, f'{prefix}depth.npy')

                # Save RGB image
                cv2.imwrite(rgb_path, rgb_image)

                # Save raw depth image (float32)
                np.save(depth_raw_path, depth_image)

                if prefix == 'rgbd_':
                    # directly save
                    cv2.imwrite(depth_vis_path, depth_image)
                    
                else:
                    # Save visualized depth image (normalize and convert to uint8)
                    depth_image_clipped = np.clip(depth_image, 0, 3.0)
                    depth_vis = cv2.normalize(depth_image_clipped, None, 0, 255, cv2.NORM_MINMAX)
                    depth_vis = depth_vis.astype('uint8')
                    cv2.imwrite(depth_vis_path, depth_vis)

                setattr(self, last_saved_attr, current_time)
                self.get_logger().info(f'Saved {prefix} images')
            except Exception as e:
                self.get_logger().error(f'Failed to process {prefix} images: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RGBDepthSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()