import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD
from cv_bridge import CvBridge
from collections import deque
import cv2
import os
import numpy as np
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer



class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.interval_time = 10
        self.last_saved_time_rgbd_ = 0

        self.rgb_queues = {'l': deque(maxlen=1), 'r': deque(maxlen=1)}
        self.depth_queues = {'l': deque(maxlen=1), 'r': deque(maxlen=1)}
        
        # 添加时间戳记录
        self.last_received_time = {'l': {'rgb': 0, 'depth': 0}, 'r': {'rgb': 0, 'depth': 0}}
        self.data_timeout = 9.0  # 9秒内没有新数据则认为数据过期

        self.create_subscription(Image, '/zedl/zed_node/rgb/image_rect_color', self.make_rgb_cb('l'), 10)
        self.create_subscription(Image, '/zedr/zed_node/rgb/image_rect_color', self.make_rgb_cb('r'), 10)

        self.create_subscription(Image, '/zedl/zed_node/depth/depth_registered', self.make_depth_cb('l'), 10)
        self.create_subscription(Image, '/zedr/zed_node/depth/depth_registered', self.make_depth_cb('r'), 10)

        self.rgbd = Subscriber(self, RGBD, '/camera/camera/rgbd')
        
        self.output_dir = os.path.join(os.getcwd(), '/home/mosaic/mosaic/manipulation_ws/saved_images')
        os.makedirs(self.output_dir, exist_ok=True)

        self.create_timer(self.interval_time, self.auto_save)

        self.ts_rgbd = ApproximateTimeSynchronizer([self.rgbd], queue_size= 10, slop=0.1)
        self.ts_rgbd.registerCallback(self.callback_rgbd)

        self.get_logger().info("ImageAutoSaver started: will save every 10 seconds (overwrite).")



    def make_rgb_cb(self, cam):
        def cb(msg):
            try:
                img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.rgb_queues[cam].append(img)
                self.last_received_time[cam]['rgb'] = time.time()  # 记录接收时间
            except Exception as e:
                self.get_logger().error(f"[{cam}] RGB error: {e}")
        return cb

    def make_depth_cb(self, cam):
        def cb(msg):
            try:
                depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                self.depth_queues[cam].append(depth)
                self.last_received_time[cam]['depth'] = time.time()  # 记录接收时间
            except Exception as e:
                self.get_logger().error(f"[{cam}] Depth error: {e}")
        return cb


    def callback_rgbd(self, rgbd_msg):
        rgb_msg = rgbd_msg.rgb
        depth_msg = rgbd_msg.depth
        self.save_images(rgb_msg, depth_msg, prefix='rgbd_')


    def save_images(self, rgb_msg, depth_msg, prefix=''):
        current_time = time.time()
        last_saved_attr = f'last_saved_time_{prefix}'
        last_saved = getattr(self, last_saved_attr)
        if current_time - last_saved >= self.interval_time:
            try:

                rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

                rgb_path = os.path.join(self.output_dir, f'{prefix}rgb.png')
                depth_vis_path = os.path.join(self.output_dir, f'{prefix}depth.png')
                depth_raw_path = os.path.join(self.output_dir, f'{prefix}depth.npy')

                cv2.imwrite(rgb_path, rgb_image)

                np.save(depth_raw_path, depth_image)

                cv2.imwrite(depth_vis_path, depth_image)

                setattr(self, last_saved_attr, current_time)
                self.get_logger().info(f'Saved {prefix} RGB + Depth')
            except Exception as e:
                self.get_logger().error(f'Failed to process {prefix} images: {e}')

    def auto_save(self):
        current_time = time.time()
        
        for cam in ['l', 'r']:
            # 检查队列是否为空
            if not self.rgb_queues[cam] or not self.depth_queues[cam]:
                self.get_logger().warn(f"Skipping {cam}: missing data")
                continue
                
            # 检查数据是否过期
            rgb_age = current_time - self.last_received_time[cam]['rgb']
            depth_age = current_time - self.last_received_time[cam]['depth']
            
            if rgb_age > self.data_timeout or depth_age > self.data_timeout:
                self.get_logger().warn(f"Skipping {cam}: data too old (RGB: {rgb_age:.1f}s, Depth: {depth_age:.1f}s)")
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
