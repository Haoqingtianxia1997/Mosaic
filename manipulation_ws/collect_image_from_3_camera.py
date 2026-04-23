import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import sys
import time
import termios
import tty
from threading import Thread


SAVE_DIR = './collected_images'
SAVE_INTERVAL = 1.0  # 每路相机最多每秒保存 1 张


class ImageCollectorNode(Node):
    def __init__(self):
        super().__init__('image_collector_node')
        self.bridge = CvBridge()
        self._recording = False

        for sub in ('zedl', 'zedr', 'realsense'):
            os.makedirs(os.path.join(SAVE_DIR, sub), exist_ok=True)

        self._last_save = {'zedl': 0.0, 'zedr': 0.0, 'realsense': 0.0}

        self.create_subscription(
            CompressedImage,
            '/zedl/zed_node/rgb/image_rect_color/compressed',
            lambda msg: self._save_compressed(msg, 'zedl'),
            10,
        )
        self.create_subscription(
            CompressedImage,
            '/zedr/zed_node/rgb/image_rect_color/compressed',
            lambda msg: self._save_compressed(msg, 'zedr'),
            10,
        )
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            lambda msg: self._save_raw(msg, 'realsense'),
            10,
        )

        Thread(target=self._keyboard_listener, daemon=True).start()
        self.get_logger().info(
            f'ImageCollector ready. 按 [c] 开始/停止存图，存到 {os.path.abspath(SAVE_DIR)}'
        )

    # ------------------------------------------------------------------ #
    # 键盘监听（非阻塞，逐字符读取）
    # ------------------------------------------------------------------ #
    def _keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch == 'c':
                    self._recording = not self._recording
                    state = '▶ 开始存图' if self._recording else '■ 停止存图'
                    self.get_logger().info(state)
                elif ch in ('\x03', 'q'):   # Ctrl-C 或 q 退出
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ------------------------------------------------------------------ #
    # 回调
    # ------------------------------------------------------------------ #
    def _save_compressed(self, msg: CompressedImage, camera: str):
        if not self._recording:
            return
        now = time.time()
        if now - self._last_save[camera] < SAVE_INTERVAL:
            return
        self._last_save[camera] = now
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        self._write(img, camera)

    def _save_raw(self, msg: Image, camera: str):
        if not self._recording:
            return
        now = time.time()
        if now - self._last_save[camera] < SAVE_INTERVAL:
            return
        self._last_save[camera] = now
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self._write(img, camera)

    def _write(self, img, camera: str):
        ts = time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}'
        path = os.path.join(SAVE_DIR, camera, f'{ts}.jpg')
        cv2.imwrite(path, img)
        self.get_logger().info(f'[{camera}] {path}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
