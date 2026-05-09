import struct
import subprocess
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from action_interfaces.srv import FetchSegCloud

CLOUD_SAVE_PATH = '/tmp/seg_cloud.npz'
INFER_SERVICE_TIMEOUT = 120  # seconds


def _parse_and_save_cloud(msg: PointCloud2, out_path: str):
    fields = {f.name: f for f in msg.fields}
    step = msg.point_step
    raw = bytes(msg.data)
    nx = fields['x'].offset
    ny = fields['y'].offset
    nz = fields['z'].offset
    rgb_f = fields.get('rgb') or fields.get('rgba')

    pts, cols = [], []
    for i in range(msg.width * msg.height):
        base = i * step
        x, y, z = struct.unpack_from('fff', raw, base + nx)
        if x != x:  # NaN
            continue
        pts.append([x, y, z])
        if rgb_f is not None:
            p = struct.unpack_from('I', raw, base + rgb_f.offset)[0]
            cols.append([(p >> 16 & 0xFF) / 255.0,
                         (p >>  8 & 0xFF) / 255.0,
                         (p       & 0xFF) / 255.0])
        else:
            cols.append([1.0, 1.0, 1.0])

    if not pts:
        raise ValueError('Received empty point cloud')

    np.savez(out_path,
             points=np.array(pts, dtype=np.float32),
             colors=np.array(cols, dtype=np.float32))


class SegCloudBridge(Node):
    def __init__(self):
        super().__init__('seg_cloud_bridge')
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            PointCloud2, '/seg/point_cloud',
            self._cloud_cb, 10,
            callback_group=self._cb_group,
        )

        self.create_service(
            FetchSegCloud, '/fetch_seg_cloud',
            self._fetch_cb,
            callback_group=self._cb_group,
        )

        self._cloud_event = threading.Event()
        self._latest_cloud: PointCloud2 | None = None
        self._cloud_lock = threading.Lock()
        self.get_logger().info('seg_cloud_bridge ready')

    def _cloud_cb(self, msg: PointCloud2):
        with self._cloud_lock:
            self._latest_cloud = msg
        self._cloud_event.set()

    def _fetch_cb(self, request: FetchSegCloud.Request,
                  response: FetchSegCloud.Response):
        target = request.target_label
        self.get_logger().info(f'fetch_seg_cloud: target="{target}"')

        # Set target_label param on the seg service
        try:
            subprocess.run(
                ['ros2', 'param', 'set', '/seg_service', 'target_label', target],
                check=True, capture_output=True, timeout=5,
            )
        except Exception as e:
            response.success = False
            response.message = f'param set failed: {e}'
            return response

        # Clear the event so we only accept a cloud produced by this inference
        self._cloud_event.clear()

        # Trigger inference (blocking — returns after server publishes the cloud)
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', '/seg/infer',
                 'std_srvs/srv/Trigger', '{}'],
                capture_output=True, text=True,
                timeout=INFER_SERVICE_TIMEOUT,
            )
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = '/seg/infer timed out'
            return response

        if 'success=True' not in result.stdout:
            response.success = False
            response.message = f'/seg/infer reported failure: {result.stdout.strip()}'
            return response

        # Wait for the subscription callback to deliver the cloud
        if not self._cloud_event.wait(timeout=10.0):
            response.success = False
            response.message = 'Timed out waiting for /seg/point_cloud'
            return response

        with self._cloud_lock:
            cloud = self._latest_cloud

        try:
            _parse_and_save_cloud(cloud, CLOUD_SAVE_PATH)
            response.success = True
            response.message = f'Saved {cloud.width * cloud.height} points for "{target}"'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Cloud parse/save failed: {e}'

        return response


def main():
    rclpy.init()
    node = SegCloudBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
