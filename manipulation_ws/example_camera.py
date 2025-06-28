import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np

class RGBDepthSaver(Node):
    def __init__(self):
        super().__init__('rgb_depth_saver')

        self.bridge = CvBridge()
        # 分别记录左右相机的上次保存时间
        self.last_saved_time_r_ = 0
        self.last_saved_time_l_ = 0

        # 创建保存目录
        self.output_dir = os.path.join(os.getcwd(), 'saved_images')
        os.makedirs(self.output_dir, exist_ok=True)

        # 订阅左右相机的 RGB 和 Depth 图像
        self.r_rgb_sub = Subscriber(self, Image, '/zedr/zed_node/rgb/image_rect_color')
        self.r_depth_sub = Subscriber(self, Image, '/zedr/zed_node/depth/depth_registered')
        self.l_rgb_sub = Subscriber(self, Image, '/zedl/zed_node/rgb/image_rect_color')
        self.l_depth_sub = Subscriber(self, Image, '/zedl/zed_node/depth/depth_registered')

        # 分别为左右相机设置同步器
        self.ts_right = ApproximateTimeSynchronizer([self.r_rgb_sub, self.r_depth_sub], queue_size=10, slop=0.1)
        self.ts_right.registerCallback(self.callback_right)

        self.ts_left = ApproximateTimeSynchronizer([self.l_rgb_sub, self.l_depth_sub], queue_size=10, slop=0.1)
        self.ts_left.registerCallback(self.callback_left)

        self.get_logger().info('RGBDepthSaver node has been started.')

    def callback_right(self, rgb_msg, depth_msg):
        self.save_images(rgb_msg, depth_msg, prefix='r_')

    def callback_left(self, rgb_msg, depth_msg):
        self.save_images(rgb_msg, depth_msg, prefix='l_')

    def save_images(self, rgb_msg, depth_msg, prefix=''):
        current_time = time.time()
        # 每10秒保存一次，左右相机分别计时
        last_saved_attr = f'last_saved_time_{prefix}'
        last_saved = getattr(self, last_saved_attr)
        if current_time - last_saved >= 10.0:
            try:
                # 转换为 OpenCV 图像
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

                # 构建文件路径（无时间戳，始终覆盖）
                rgb_path = os.path.join(self.output_dir, f'{prefix}rgb.png')
                depth_vis_path = os.path.join(self.output_dir, f'{prefix}depth.png')
                depth_raw_path = os.path.join(self.output_dir, f'{prefix}depth.npy')

                # 保存 RGB 图像
                cv2.imwrite(rgb_path, rgb_image)

                # 保存原始深度图（float32）
                np.save(depth_raw_path, depth_image)

                # 保存可视化深度图（归一化后转为 uint8）
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