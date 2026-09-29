#!/usr/bin/env python3
"""
Camera extrinsic calibration from AprilTag / ArUco markers or a ChArUco board.

Detects markers in an image topic, estimates each marker's pose in the camera
frame, and uses the known marker poses in a fixed world frame to compute the
camera pose in the world frame:

    T_world_cam = T_world_marker @ inv(T_cam_marker)

With several markers visible, all their corners are used together in one PnP
solve, which gives a more stable result than any single marker.

Camera topics (see src/action/action/image_saver.py):
    ZED right : /zedr/zed_node/rgb/image_rect_color   (info: /zedr/zed_node/rgb/camera_info)
    RealSense : /camera/camera/color/image_raw        (info: /camera/camera/color/camera_info)

Marker frame convention (OpenCV): origin at the marker center, x to the right,
y up, z out of the marker face toward the viewer.
The resulting camera pose is that of the camera *optical* frame
(x right, y down, z forward).

--marker takes one of these forms (repeat it for several markers):
    --marker ID SIZE X Y Z ROLL PITCH YAW       (angles in degrees, extrinsic xyz)
    --marker ID SIZE X Y Z QX QY QZ QW          (quaternion)
SIZE is the black border edge length in meters; X Y Z are in meters.

ChArUco board (--marker-type charuco) uses --board-pose instead of --marker:
    --board-pose X Y Z ROLL PITCH YAW  |  X Y Z QX QY QZ QW
Board frame convention (OpenCV): origin at the top-left outer corner of the
printed board, x along the columns (right), y along the rows (DOWN), so z
points INTO the board. --charuco-squares is COLS ROWS in squares (not inner
corners). Use --charuco-legacy for boards made by OpenCV < 4.6 or calib.io
with an even number of rows.

Examples:
    python3 cam_calib.py --topic /zedr/zed_node/rgb/image_rect_color \
        --marker-type apriltag --marker 0 0.10 0.5 0.0 0.0 0 0 0

    python3 cam_calib.py --topic /camera/camera/color/image_raw \
        --marker-type aruco --aruco-dict DICT_4X4_50 \
        --marker 3 0.08 0.4 0.2 0.0 0 0 90 \
        --marker 7 0.08 0.4 -0.2 0.0 0 0 90 \
        --num-frames 30 --output rs_extrinsic.yaml --show

    python3 cam_calib.py --topic /zedr/zed_node/rgb/image_rect_color \
        --marker-type charuco --charuco-dict DICT_5X5_100 \
        --charuco-squares 7 5 --charuco-square-length 0.04 --charuco-marker-length 0.03 \
        --board-pose 0.3 0.2 0.0 180 0 0
"""

import argparse
import json
import sys
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def make_T(rot: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    return make_T(rot, tvec)


def T_to_rvec_tvec(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec, T[:3, 3].reshape(3, 1).copy()


def marker_corners_local(size: float) -> np.ndarray:
    """Corners in marker frame, in OpenCV detector order: TL, TR, BR, BL."""
    h = size / 2.0
    return np.array([[-h, h, 0.0],
                     [h, h, 0.0],
                     [h, -h, 0.0],
                     [-h, -h, 0.0]], dtype=np.float64)


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

class MarkerSpec:
    def __init__(self, marker_id: int, size: float, T_world_marker: np.ndarray):
        self.id = marker_id
        self.size = size
        self.T_world_marker = T_world_marker

    def corners_world(self) -> np.ndarray:
        local = marker_corners_local(self.size)
        return (self.T_world_marker[:3, :3] @ local.T).T + self.T_world_marker[:3, 3]


def parse_pose(nums: List[float]) -> np.ndarray:
    """X Y Z ROLL PITCH YAW (deg, extrinsic xyz) or X Y Z QX QY QZ QW -> 4x4."""
    xyz, rot_vals = nums[:3], nums[3:]
    if len(rot_vals) == 3:
        rot = R.from_euler('xyz', rot_vals, degrees=True)
    else:
        rot = R.from_quat(rot_vals)  # x, y, z, w
    return make_T(rot.as_matrix(), xyz)


def parse_marker(values: List[str]) -> MarkerSpec:
    if len(values) not in (8, 9):
        raise argparse.ArgumentTypeError(
            f'--marker needs 8 (ID SIZE X Y Z ROLL PITCH YAW) or '
            f'9 (ID SIZE X Y Z QX QY QZ QW) values, got {len(values)}: {values}')
    marker_id = int(values[0])
    nums = [float(v) for v in values[1:]]
    return MarkerSpec(marker_id, nums[0], parse_pose(nums[1:]))


def default_info_topic(image_topic: str) -> str:
    return image_topic.rsplit('/', 1)[0] + '/camera_info'


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='Estimate camera pose in world frame from AprilTag/ArUco markers '
                    'or a ChArUco board.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--topic', required=True,
                   help='Full image topic, e.g. /zedr/zed_node/rgb/image_rect_color '
                        'or /camera/camera/color/image_raw')
    p.add_argument('--camera-info-topic', default=None,
                   help='CameraInfo topic (default: <image topic namespace>/camera_info)')
    p.add_argument('--marker-type', choices=['apriltag', 'aruco', 'charuco'], required=True)
    p.add_argument('--apriltag-dict', default='DICT_APRILTAG_36h11',
                   help='OpenCV dictionary used when --marker-type apriltag')
    p.add_argument('--aruco-dict', default='DICT_4X4_50',
                   help='OpenCV dictionary used when --marker-type aruco')
    p.add_argument('--marker', nargs='+', action='append', default=None,
                   metavar='VAL',
                   help='ID SIZE X Y Z ROLL PITCH YAW  or  ID SIZE X Y Z QX QY QZ QW '
                        '(marker pose in world frame). Repeat for several markers. '
                        'Required for apriltag/aruco.')
    # ChArUco board
    p.add_argument('--charuco-dict', default='DICT_5X5_100',
                   help='OpenCV dictionary of the ChArUco board markers')
    p.add_argument('--charuco-squares', nargs=2, type=int, metavar=('COLS', 'ROWS'),
                   help='Board size in squares (not inner corners)')
    p.add_argument('--charuco-square-length', type=float,
                   help='Chessboard square edge length [m]')
    p.add_argument('--charuco-marker-length', type=float,
                   help='ArUco marker edge length inside a square [m]')
    p.add_argument('--charuco-legacy', action='store_true',
                   help='Legacy pattern (OpenCV < 4.6 / calib.io boards with even rows)')
    p.add_argument('--charuco-min-corners', type=int, default=6,
                   help='Minimum detected ChArUco corners to accept a frame')
    p.add_argument('--board-pose', nargs='+', type=float, metavar='VAL',
                   help='X Y Z ROLL PITCH YAW  or  X Y Z QX QY QZ QW '
                        '(board pose in world frame). Required for charuco.')
    p.add_argument('--num-frames', type=int, default=20,
                   help='Number of frames with valid detections to average')
    p.add_argument('--timeout', type=float, default=30.0,
                   help='Give up after this many seconds')
    p.add_argument('--world-frame', default='world')
    p.add_argument('--camera-frame', default=None,
                   help='Child frame name in the output (default: image header frame_id)')
    p.add_argument('--output', default=None,
                   help='Save result to .yaml or .json')
    p.add_argument('--save-image', default=None,
                   help='Save the last annotated image to this path')
    p.add_argument('--show', action='store_true', help='Show detections live')
    return p


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CameraCalibNode(Node):
    def __init__(self, args):
        super().__init__('cam_calib')
        self.args = args
        self.bridge = CvBridge()

        self.is_charuco = args.marker_type == 'charuco'
        dict_name = {'apriltag': args.apriltag_dict, 'aruco': args.aruco_dict,
                     'charuco': args.charuco_dict}[args.marker_type]
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f'Unknown OpenCV aruco dictionary: {dict_name}')
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

        self.markers: Dict[int, MarkerSpec] = {}
        if self.is_charuco:
            missing = [n for n in ('charuco_squares', 'charuco_square_length',
                                   'charuco_marker_length', 'board_pose')
                       if getattr(args, n) is None]
            if missing:
                raise ValueError(f'charuco needs: {", ".join("--" + m.replace("_", "-") for m in missing)}')
            if len(args.board_pose) not in (6, 7):
                raise ValueError('--board-pose needs 6 (X Y Z R P Y) or 7 (X Y Z QX QY QZ QW) values')
            self.board = cv2.aruco.CharucoBoard(
                tuple(args.charuco_squares), args.charuco_square_length,
                args.charuco_marker_length, dictionary)
            self.board.setLegacyPattern(args.charuco_legacy)
            self.T_world_board = parse_pose(args.board_pose)
            self.charuco_detector = cv2.aruco.CharucoDetector(self.board)
            target_desc = (f'board {args.charuco_squares[0]}x{args.charuco_squares[1]} '
                           f'sq={args.charuco_square_length} mk={args.charuco_marker_length}')
        else:
            if not args.marker:
                raise ValueError(f'{args.marker_type} needs at least one --marker')
            for vals in args.marker:
                spec = parse_marker(vals)
                if spec.id in self.markers:
                    raise ValueError(f'Marker id {spec.id} given twice')
                self.markers[spec.id] = spec
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = (
                cv2.aruco.CORNER_REFINE_APRILTAG if args.marker_type == 'apriltag'
                else cv2.aruco.CORNER_REFINE_SUBPIX)
            self.detector = cv2.aruco.ArucoDetector(dictionary, params)
            target_desc = f'marker ids: {sorted(self.markers)}'

        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None
        self.frame_id: Optional[str] = None
        self.samples: List[np.ndarray] = []      # T_world_cam per frame
        self.reproj_errors: List[float] = []
        self.last_vis: Optional[np.ndarray] = None
        self.done = False

        info_topic = args.camera_info_topic or default_info_topic(args.topic)
        self.create_subscription(CameraInfo, info_topic, self.info_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, args.topic, self.image_cb,
                                 qos_profile_sensor_data)

        self.get_logger().info(
            f'Image: {args.topic} | CameraInfo: {info_topic} | '
            f'{args.marker_type} ({dict_name}) | {target_desc}')

    def info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
            self.get_logger().info(f'Got intrinsics:\nK=\n{self.K}\nD={self.D}')

    def image_cb(self, msg: Image):
        if self.done or self.K is None:
            return
        self.frame_id = msg.header.frame_id
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        vis = img.copy()

        if self.is_charuco:
            T_world_cam, err = self.estimate_charuco(gray, vis)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
            if ids is not None and len(ids):
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            T_world_cam, err = self.estimate(corners, ids, vis)
        if T_world_cam is not None:
            self.samples.append(T_world_cam)
            self.reproj_errors.append(err)
            self.get_logger().info(
                f'[{len(self.samples)}/{self.args.num_frames}] '
                f't={np.round(T_world_cam[:3, 3], 4)} reproj={err:.3f}px')

        self.last_vis = vis
        if self.args.show:
            cv2.imshow('cam_calib', vis)
            cv2.waitKey(1)

        if len(self.samples) >= self.args.num_frames:
            self.done = True

    def estimate(self, corners, ids, vis) -> Tuple[Optional[np.ndarray], float]:
        """Return T_world_cam for one frame and its RMS reprojection error."""
        if ids is None:
            return None, 0.0

        obj_pts, img_pts = [], []
        best = None  # (err, T_cam_world) from the best single marker
        for c, mid in zip(corners, ids.flatten()):
            spec = self.markers.get(int(mid))
            if spec is None:
                continue
            c2d = c.reshape(4, 2).astype(np.float64)

            # Single-marker pose in camera frame (IPPE is exact for squares)
            ok, rvec, tvec = cv2.solvePnP(
                marker_corners_local(spec.size), c2d, self.K, self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            cv2.drawFrameAxes(vis, self.K, self.D, rvec, tvec, spec.size * 0.5)

            T_cam_marker = rvec_tvec_to_T(rvec, tvec)
            T_cam_world = T_cam_marker @ inv_T(spec.T_world_marker)
            proj, _ = cv2.projectPoints(marker_corners_local(spec.size),
                                        rvec, tvec, self.K, self.D)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - c2d) ** 2, axis=1))))
            if best is None or err < best[0]:
                best = (err, T_cam_world)

            obj_pts.append(spec.corners_world())
            img_pts.append(c2d)

        if best is None:
            return None, 0.0

        obj = np.vstack(obj_pts)
        img = np.vstack(img_pts)
        rvec, tvec = T_to_rvec_tvec(best[1])
        if len(obj_pts) > 1:
            # Refine using all visible markers jointly
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, self.D, rvec, tvec)

        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
        return inv_T(rvec_tvec_to_T(rvec, tvec)), err

    def estimate_charuco(self, gray, vis) -> Tuple[Optional[np.ndarray], float]:
        """Return T_world_cam from a ChArUco board and its RMS reprojection error."""
        ch_corners, ch_ids, mk_corners, mk_ids = self.charuco_detector.detectBoard(gray)
        if mk_ids is not None and len(mk_ids):
            cv2.aruco.drawDetectedMarkers(vis, mk_corners, mk_ids)
        if ch_ids is None or len(ch_ids) < max(4, self.args.charuco_min_corners):
            return None, 0.0
        cv2.aruco.drawDetectedCornersCharuco(vis, ch_corners, ch_ids)

        obj, img = self.board.matchImagePoints(ch_corners, ch_ids)
        obj = obj.reshape(-1, 3).astype(np.float64)
        img = img.reshape(-1, 2).astype(np.float64)
        # All corners lie on the board plane -> IPPE, then LM refinement
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None, 0.0
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, self.D, rvec, tvec)
        cv2.drawFrameAxes(vis, self.K, self.D, rvec, tvec,
                          self.args.charuco_square_length * 2)

        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
        T_cam_board = rvec_tvec_to_T(rvec, tvec)
        return self.T_world_board @ inv_T(T_cam_board), err

    def result(self) -> dict:
        Ts = np.stack(self.samples)
        t_all = Ts[:, :3, 3]
        rots = R.from_matrix(Ts[:, :3, :3])
        t_mean = t_all.mean(axis=0)
        rot_mean = rots.mean()
        ang_dev = np.degrees((rots * rot_mean.inv()).magnitude())
        T = make_T(rot_mean.as_matrix(), t_mean)
        return {
            'parent_frame': self.args.world_frame,
            'child_frame': self.args.camera_frame or self.frame_id or 'camera',
            'topic': self.args.topic,
            'marker_type': self.args.marker_type,
            'num_samples': len(self.samples),
            'translation': t_mean.tolist(),
            'quaternion_xyzw': rot_mean.as_quat().tolist(),
            'rpy_deg': rot_mean.as_euler('xyz', degrees=True).tolist(),
            'T_world_cam': T.tolist(),
            'translation_std': t_all.std(axis=0).tolist(),
            'rotation_std_deg': float(np.sqrt(np.mean(ang_dev ** 2))),
            'mean_reproj_error_px': float(np.mean(self.reproj_errors)),
        }


def print_result(res: dict):
    t, q = res['translation'], res['quaternion_xyzw']
    print('\n========== Camera pose in world frame ==========')
    print(f"{res['parent_frame']} -> {res['child_frame']}  ({res['num_samples']} samples)")
    print(f"translation [m]      : {np.round(t, 5).tolist()}")
    print(f"quaternion [x y z w] : {np.round(q, 6).tolist()}")
    print(f"rpy [deg] (xyz)      : {np.round(res['rpy_deg'], 3).tolist()}")
    print(f"translation std [m]  : {np.round(res['translation_std'], 5).tolist()}")
    print(f"rotation std [deg]   : {res['rotation_std_deg']:.4f}")
    print(f"mean reproj err [px] : {res['mean_reproj_error_px']:.4f}")
    print('T_world_cam =')
    print(np.array2string(np.array(res['T_world_cam']), precision=6, suppress_small=True))
    print('\nstatic TF:')
    print('ros2 run tf2_ros static_transform_publisher '
          f'--x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} '
          f'--qx {q[0]:.6f} --qy {q[1]:.6f} --qz {q[2]:.6f} --qw {q[3]:.6f} '
          f"--frame-id {res['parent_frame']} --child-frame-id {res['child_frame']}")
    print('================================================\n')


def save_result(res: dict, path: str):
    if path.endswith('.json'):
        with open(path, 'w') as f:
            json.dump(res, f, indent=2)
    else:
        import yaml
        with open(path, 'w') as f:
            yaml.safe_dump(res, f, sort_keys=False)


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv)
    args = build_arg_parser().parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

    node = CameraCalibNode(args)
    start = node.get_clock().now()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.get_clock().now() - start).nanoseconds / 1e9 > args.timeout:
                node.get_logger().warn('Timeout reached.')
                break
    except KeyboardInterrupt:
        pass

    if node.samples:
        res = node.result()
        print_result(res)
        if args.output:
            save_result(res, args.output)
            print(f'Saved to {args.output}')
    else:
        node.get_logger().error(
            'No valid detections. Check topic, marker type/dictionary and ids '
            '(charuco: squares, lengths, --charuco-legacy).'
            if node.K is not None else 'No CameraInfo received.')

    if args.save_image and node.last_vis is not None:
        cv2.imwrite(args.save_image, node.last_vis)
    if args.show:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
