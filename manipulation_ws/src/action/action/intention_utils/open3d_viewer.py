import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import cv2
import copy
import time
from queue import Queue






class PersistentOpen3DViewer:
    def __init__(self):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name='ZED2 ICP PointCloud Viewer', width=960, height=720)
        self.current_arrows = []
        self.current_intersect = None
        self.arrow_direction = None
        self.arrow_origin = None
        self.arrow_needs_update = False
        self.arrow_queue = Queue()
        self.intersect_queue = Queue()
    
    def load_point_cloud(self, depth_path: str, rgb_path: str, intrinsics: tuple, extrinsics: dict):
        depth = np.load(depth_path)
        rgb = cv2.cvtColor(cv2.imread(rgb_path), cv2.COLOR_BGR2RGB)

        fx, fy, cx, cy = intrinsics
        roll, pitch, yaw = extrinsics['euler']
        t = np.asarray(extrinsics['translation'])
        rot = R.from_euler('xyz', [roll, pitch, yaw])

        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        pts = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        z_flat = z.reshape(-1)
        mask = (z_flat > 0) & (z_flat < 5)
        pts = pts[mask]

        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = t

        pts_h = np.concatenate([pts, np.ones((pts.shape[0], 1))], axis=1)
        pts_w = (T @ pts_h.T).T[:, :3]

        colours = rgb.reshape(-1, 3)[mask] / 255.0

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_w)
        pcd.colors = o3d.utility.Vector3dVector(colours)

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        frame.transform(T)

        return pcd, frame

    def load_and_register(self):
        intr_left = (1060.0899, 1059.0899, 958.9099, 561.5670)
        extr_left = {'euler': (-2.38, 0.47, -0.72), 'translation': [0.11, -0.50, 0.55]}
        pcd_left, frame_left = self.load_point_cloud('./background/l_depth.npy', './background/l_rgb.png', intr_left, extr_left)

        intr_right = (1059.9764, 1059.9764, 963.0756, 522.3530)
        extr_right = {'euler': (-2.29, 0.29, 2.34), 'translation': [0.90, 0.43, 0.59]}
        pcd_right, frame_right = self.load_point_cloud('./background/r_depth.npy', './background/r_rgb.png', intr_right, extr_right)

        voxel = 0.01
        pcd_left_d = pcd_left.voxel_down_sample(voxel)
        pcd_right_d = pcd_right.voxel_down_sample(voxel)

        pcd_left_d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30))
        pcd_right_d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30))

        reg = o3d.pipelines.registration.registration_icp(
            source=pcd_left_d,
            target=pcd_right_d,
            max_correspondence_distance=voxel * 2.0,
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )

        pcd_left_icp = copy.deepcopy(pcd_left)
        pcd_left_icp.transform(reg.transformation)
        frame_left_icp = copy.deepcopy(frame_left)
        frame_left_icp.transform(reg.transformation)
        world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)

        return [pcd_right, pcd_left_icp, frame_right, frame_left_icp, world]

    def update_arrow_async(self, directions, origins):
        if isinstance(directions, list) and isinstance(origins, list):
            self.arrow_queue.put((directions, origins))
        else:
            self.arrow_queue.put((directions, origins))



    def update_intersect_async(self, intersect_point):
        self.intersect_queue.put((intersect_point))

    def initialize_scene(self):
        geometries = self.load_and_register()
        for g in geometries:
            self.vis.add_geometry(g)

    def run_main_loop(self):
        self.initialize_scene()
        while True:
            # update arrows
            while not self.arrow_queue.empty():
                directions, origins = self.arrow_queue.get()
                # 删除所有旧箭头
                for arr in self.current_arrows:
                    self.vis.remove_geometry(arr)
                self.current_arrows = []

                # 判断是否批量
                if isinstance(directions, list) and isinstance(origins, list):
                    for d, o in zip(directions, origins):
                        arr = self._create_arrow(d, o)
                        if arr is not None:
                            self.vis.add_geometry(arr)
                            self.current_arrows.append(arr)
                else:
                    arr = self._create_arrow(directions, origins)
                    if arr is not None:
                        self.vis.add_geometry(arr)
                        self.current_arrows.append(arr)

            # update intersect（不变）
            while not self.intersect_queue.empty():
                pt = self.intersect_queue.get()
                if hasattr(self, "current_intersect") and self.current_intersect:
                    self.vis.remove_geometry(self.current_intersect)
                    self.current_intersect = None
                if pt is not None:
                    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025)
                    sphere.paint_uniform_color([0.1, 0.7, 1.0])
                    sphere.translate(pt)
                    self.vis.add_geometry(sphere)
                    self.current_intersect = sphere

            self.vis.poll_events()
            self.vis.update_renderer()
            cv2.waitKey(1)
            time.sleep(0.03)


    def _create_arrow(self, direction, origin):
        if direction is None or origin is None:
            return None
        
        length = 1.5
        direction = direction / np.linalg.norm(direction)
        arrow = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=0.005,
            cone_radius=0.01,
            cylinder_height=length * 0.8,
            cone_height=length * 0.2
        )
        arrow.compute_vertex_normals()
        default_dir = np.array([0, 0, 1])
        rot_matrix = R.align_vectors([direction], [default_dir])[0].as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = origin
        arrow.transform(T)
        return arrow

    def close(self):
        self.vis.destroy_window()

