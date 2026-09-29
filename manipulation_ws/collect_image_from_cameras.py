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
from threading import Lock, Thread


SAVE_DIR = './collected_images'
CAMERAS = ('zedr', 'realsense')


class ImageCollectorNode(Node):
    def __init__(self):
        super().__init__('image_collector_node')
        self.bridge = CvBridge()
        self._lock = Lock()
        self._latest = {cam: None for cam in CAMERAS}  # 每路相机最新一帧
        self._count = 0

        for sub in CAMERAS:
            os.makedirs(os.path.join(SAVE_DIR, sub), exist_ok=True)

        self.create_subscription(
            CompressedImage,
            '/zedr/zed_node/rgb/image_rect_color/compressed',
            lambda msg: self._cache(msg, 'zedr'),
            10,
        )
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            lambda msg: self._cache(msg, 'realsense'),
            10,
        )

        Thread(target=self._keyboard_listener, daemon=True).start()
        self.get_logger().info(
            f'ImageCollector ready. 每按一次 [c] 两路各存一张，存到 {os.path.abspath(SAVE_DIR)}'
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
                    self._capture()
                elif ch in ('\x03', 'q'):   # Ctrl-C 或 q 退出
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ------------------------------------------------------------------ #
    # 回调：只缓存最新一帧
    # ------------------------------------------------------------------ #
    def _cache(self, msg, camera: str):
        with self._lock:
            self._latest[camera] = msg

    # ------------------------------------------------------------------ #
    # 按键存图：两路各存一张，文件名相同便于配对
    # ------------------------------------------------------------------ #
    def _capture(self):
        with self._lock:
            snapshot = dict(self._latest)

        missing = [cam for cam, msg in snapshot.items() if msg is None]
        if missing:
            self.get_logger().warn(f'尚未收到 {missing} 的图像，本次不保存')
            return

        ts = time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}'
        for cam, msg in snapshot.items():
            if isinstance(msg, CompressedImage):
                img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            else:
                img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            path = os.path.join(SAVE_DIR, cam, f'{ts}.jpg')
            cv2.imwrite(path, img)
            self.get_logger().info(f'[{cam}] {path}')

        self._count += 1
        self.get_logger().info(f'已保存第 {self._count} 组')


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
