import os
import numpy as np
import open3d as o3d
from pathlib import Path
from scipy.spatial.transform import Rotation
from typing import Tuple, Sequence, Optional
from src.grasp.mesh import visualize_3d_objs,create_grasp_mesh

# Patch deprecated np.float aliases removed in NumPy 1.24+
np.float = float

ANYGRASP_CHECKPOINT_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "../../../anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar"
))


class GraspGeneration:
    def __init__(self, bbox_center=None, bbox_rotation_matrix=None, use_anygrasp=True):
        self.bbox_center = bbox_center
        self.bbox_rotation_matrix = bbox_rotation_matrix
        self.valid_grasps_list = []
        self.top_10_grasps = []
        self.use_anygrasp = use_anygrasp
        self._anygrasp_model = None

    def _init_anygrasp(self):
        """Lazy-load AnyGrasp model on first use."""
        if self._anygrasp_model is not None:
            return
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../anygrasp_sdk/grasp_detection"))
        from gsnet import AnyGrasp
        from types import SimpleNamespace
        cfg = SimpleNamespace(
            checkpoint_path=ANYGRASP_CHECKPOINT_PATH,
            max_gripper_width=0.1,
            gripper_height=0.03,
            top_down_grasp=True,
            debug=False,
        )
        self._anygrasp_model = AnyGrasp(cfg)
        self._anygrasp_model.load_net()
        print("AnyGrasp model loaded.")

    def anygrasp_compute_poses(self, points, colors, visualize=True):
        """
        Use AnyGrasp to compute grasp poses from a world-frame point cloud.

        Parameters:
            points: ndarray (N, 3), world-frame point cloud
            colors: ndarray (N, 3), colors in [0, 1]
            visualize: bool, whether to visualize

        Returns:
            Same format as final_compute_poses:
            (pose1_pos, pose1_orn, pose2_pos, pose2_orn, top_10_grasps, valid_grasps_list)
        """
        self._init_anygrasp()

        points = points.astype(np.float32)
        colors = colors.astype(np.float32)
        if colors.max() > 1.1:
            colors = colors / 255.0

        # Filter out invalid points
        mask = np.isfinite(points).all(axis=1)
        points = points[mask]
        colors = colors[mask]

        # Rotate point cloud 180 degrees around X-axis before feeding to AnyGrasp
        R_x_180 = np.array([[1, 0, 0],
                             [0, 1, 0],
                             [0, 0, 1]], dtype=np.float32)
        points_rotated = (R_x_180 @ points.T).T

        # Workspace limits in rotated frame
        xmin, xmax = 0.0, 0.8
        ymin, ymax = -0.8, 0.8
        zmin, zmax = -1.0, 1.0
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        print(f"AnyGrasp input: {len(points_rotated)} points")
        print(f"Point cloud range (original): min={points.min(axis=0)}, max={points.max(axis=0)}")
        print(f"Point cloud range (rotated): min={points_rotated.min(axis=0)}, max={points_rotated.max(axis=0)}")

        gg, cloud = self._anygrasp_model.get_grasp(
            points_rotated, colors, # lims=lims, --- IGNORE ---
            apply_object_mask=True, dense_grasp=False,
            collision_detection=True, voxel_size=0.0005
        )

        if gg is None or len(gg) == 0:
            print("AnyGrasp: No grasp detected after collision detection!")
            return None, None, None, None, None, None

        gg = gg.nms().sort_by_score()
        print(f"AnyGrasp: {len(gg)} grasps after NMS, top score: {gg[0].score:.4f}")

        # Build valid_grasps_list, rotating grasp poses back to original world frame
        R_x_180_f64 = R_x_180.astype(np.float64)
        valid_grasps_list = []
        num_grasps = min(len(gg), 20)
        for i in range(num_grasps):
            g = gg[i]
            # Rotate grasp rotation matrix and translation back to original frame
            R = R_x_180_f64 @ g.rotation_matrix
            grasp_center = R_x_180_f64 @ g.translation
            pose = (R, grasp_center)
            valid_grasps_list.append((float(g.score), pose, None))

        self.valid_grasps_list = valid_grasps_list
        self.top_10_grasps = valid_grasps_list[:10]

        # Best grasp
        best_grasp = valid_grasps_list[0][1]
        pose1_pos, pose1_orn, pose2_pos, pose2_orn = self.compute_grasp_poses(best_grasp)

        # Compute top 10 poses
        top_10_poses = []
        for _, pose, _ in self.top_10_grasps:
            p1_pos, p1_orn, p2_pos, p2_orn = self.compute_grasp_poses(pose)
            top_10_poses.append({
                'prep_pose_pos': p1_pos,
                'prep_pose_orn': p1_orn,
                'grasp_pose_pos': p2_pos,
                'grasp_pose_orn': p2_orn
            })

        print(f"AnyGrasp best grasp - translation: {best_grasp[1]}, score: {valid_grasps_list[0][0]:.4f}")

        # Visualization: original point cloud + rotated-back grippers
        if visualize:
            try:
                cloud_vis = o3d.geometry.PointCloud()
                cloud_vis.points = o3d.utility.Vector3dVector(points)
                cloud_vis.colors = o3d.utility.Vector3dVector(colors)
                # Rotate grippers back to original world frame for visualization
                grippers = gg[:5].to_open3d_geometry_list()
                R_x_180_4x4 = np.eye(4)
                R_x_180_4x4[:3, :3] = R_x_180_f64
                for g_mesh in grippers:
                    g_mesh.transform(R_x_180_4x4)
                o3d.visualization.draw_geometries([*grippers, cloud_vis])
            except Exception as e:
                print(f"AnyGrasp visualization skipped: {e}")

        return pose1_pos, pose1_orn, pose2_pos, pose2_orn, top_10_poses, valid_grasps_list

    def sample_grasps_state(
        self,
        center_point: np.ndarray,
        num_grasps: int,
        rotation_matrix: np.ndarray = None,
        min_point_rotated: np.ndarray = None,
        max_point_rotated: np.ndarray = None,
        center_rotated: np.ndarray = None
    ) -> Sequence[Tuple[np.ndarray, np.ndarray]]:
        """
        Generate multiple random grasp poses within the bounding box.

        Parameters:
            center_point: Centroid coordinates of the point cloud
            num_grasps: Number of random grasp poses to generate
            rotation_matrix: OBB rotation matrix (from OBB coordinate system to world coordinate system)
            min_point_rotated: Minimum point of OBB in rotated coordinate system
            max_point_rotated: Maximum point of OBB in rotated coordinate system
            center_rotated: Origin of the OBB rotated coordinate system in world coordinates

        Returns:
            list: List of rotation matrices and translation vectors
        """
        grasp_poses_list = []
        table_height = 0.00 + 0.01
        
        grasp_points = []
        grasp_directions = []  # save the grasp direction
        
        obb_dims = max_point_rotated - min_point_rotated
        # height_threshold = 0.15  # 15 centimeters
        
        x_size = obb_dims[0]
        y_size = obb_dims[1]
        z_size = obb_dims[2]
              
        bbox_x_axis = rotation_matrix[:, 0]  # X axis of the Bounding box
        bbox_y_axis = rotation_matrix[:, 1]  # Y axis of the Bounding box
        
        # determine the short axis and long axis
        if x_size < y_size:
            short_axis = bbox_x_axis
            long_axis = bbox_y_axis
        else:
            short_axis = bbox_y_axis
            long_axis = bbox_x_axis
              
        # sample the position in the bounding box
        for idx in range(num_grasps):
            rotated_coords = np.zeros(3)

            # Index for long axis and short axis
            if x_size < y_size:
                # y is long axis, x is short axis
                long_axis_idx = 1
                short_axis_idx = 0
            else:
                # x is long axis, y is short axis
                long_axis_idx = 0
                short_axis_idx = 1

            # Long axis uses normal distribution sampling (center biased)
            long_axis_center = (min_point_rotated[long_axis_idx] + max_point_rotated[long_axis_idx]) / 2
            long_axis_range = max_point_rotated[long_axis_idx] - min_point_rotated[long_axis_idx]
            # Use normal distribution with standard deviation as 1/9 of the range
            long_axis_std = long_axis_range / 9
            rotated_coords[long_axis_idx] = np.clip(
                np.random.normal(long_axis_center, long_axis_std),
                min_point_rotated[long_axis_idx],
                max_point_rotated[long_axis_idx]
            )

            # Short axis uses uniform distribution sampling
            rotated_coords[short_axis_idx] = np.random.uniform(min_point_rotated[short_axis_idx], max_point_rotated[short_axis_idx])

            # Z axis uses uniform distribution sampling
            rotated_coords[2] = np.random.uniform(min_point_rotated[2], max_point_rotated[2])
            
            # convert the sampled point from the rotated coordinate system to the world coordinate system
            grasp_center = np.dot(rotated_coords, rotation_matrix.T) + center_rotated
            
            # the grasp point is not lower than the table height
            grasp_center[2] = max(grasp_center[2], table_height)
            
             
            # Z axis is vertical downward
            grasp_z_axis = np.array([0, 0, -1])
            


            world_x = np.array([1.0, 0.0, 0.0])

            if np.dot(long_axis, world_x) < 0:
                long_axis = -long_axis
            # X axis (the thickness direction of the gripper) uses the long axis
            grasp_x_axis = long_axis
            
            # Y axis (the opening direction of the gripper) uses the short axis
            grasp_y_axis = short_axis

            # ensure the coordinate system direction is correct
            if np.dot(np.cross(grasp_x_axis, grasp_y_axis), grasp_z_axis) < 0:
                grasp_y_axis = -grasp_y_axis

            # Random rotation around Z axis by ±10 degrees
            rotation_angle = np.random.uniform(-10, 10)  # Uniform sampling from -10 to +10 degrees
            rotation_rad = np.radians(rotation_angle)

            # Construct rotation matrix around Z axis
            cos_theta = np.cos(rotation_rad)
            sin_theta = np.sin(rotation_rad)
            rotation_z = np.array([
                [cos_theta, -sin_theta, 0],
                [sin_theta, cos_theta, 0],
                [0, 0, 1]
            ])

            # Apply rotation to X axis and Y axis (Z axis remains unchanged)
            grasp_x_axis = rotation_z @ grasp_x_axis
            grasp_y_axis = rotation_z @ grasp_y_axis

            # Build the rotation matrix
            R = np.column_stack((grasp_x_axis, grasp_y_axis, grasp_z_axis))
            grasp_center = grasp_center - 0.2 * grasp_z_axis
            # # save the rotation matrix for visualization
            # grasp_directions.append(R)
            
            # add the grasp pose to the result list
            grasp_poses_list.append((R, grasp_center))
        

            
        return grasp_poses_list
    
    def sample_grasps_state_for_flavoring(
        self,
        center_point: np.ndarray,
        num_grasps: int,
        rotation_matrix: np.ndarray = None,
        min_point_rotated: np.ndarray = None,
        max_point_rotated: np.ndarray = None,
        center_rotated: np.ndarray = None
    ) -> Sequence[Tuple[np.ndarray, np.ndarray]]:
        """
        Generate multiple random grasp poses within the bounding box.

        Parameters:
            center_point: Centroid coordinates of the point cloud
            num_grasps: Number of random grasp poses to generate
            rotation_matrix: OBB rotation matrix (from OBB coordinate system to world coordinate system)
            min_point_rotated: Minimum point of OBB in rotated coordinate system
            max_point_rotated: Maximum point of OBB in rotated coordinate system
            center_rotated: Origin of the OBB rotated coordinate system in world coordinates

        Returns:
            list: List of rotation matrices and translation vectors
        """
        grasp_poses_list = []
        table_height = 0.00 + 0.01
        
        grasp_points = []
        grasp_directions = []  # save the grasp direction
        
        obb_dims = max_point_rotated - min_point_rotated
        # height_threshold = 0.15  # 15 centimeters
        
        x_size = obb_dims[0]
        y_size = obb_dims[1]
        z_size = obb_dims[2]
              
        bbox_x_axis = rotation_matrix[:, 0]  # X axis of the Bounding box
        bbox_y_axis = rotation_matrix[:, 1]  # Y axis of the Bounding box
        
        # determine the short axis and long axis
        if x_size < y_size:
            short_axis = bbox_x_axis
            long_axis = bbox_y_axis
        else:
            short_axis = bbox_y_axis
            long_axis = bbox_x_axis
              
        # sample the position in the bounding box
        for idx in range(num_grasps):
            rotated_coords = np.zeros(3)
            rotated_coords[0] = np.random.uniform(min_point_rotated[0], max_point_rotated[0])
            rotated_coords[1] = np.random.uniform(min_point_rotated[1], max_point_rotated[1])
            rotated_coords[2] = np.random.uniform(min_point_rotated[2], max_point_rotated[2])
            
            # convert the sampled point from the rotated coordinate system to the world coordinate system
            grasp_center = np.dot(rotated_coords, rotation_matrix.T) + center_rotated
            
            # the grasp point is not lower than the table height
            grasp_center[2] = max(grasp_center[2], table_height)
            
                     
            grasp_x_axis = np.array([0, 0, 1])  # X axis is vertical upward

            grasp_y_axis = short_axis

            grasp_z_axis = long_axis

            world_x = np.array([1.0, 0.0, 0.0])

            if np.dot(grasp_z_axis, world_x) < 0:
                grasp_z_axis = -grasp_z_axis
            
            # ensure the coordinate system direction is correct
            if np.dot(np.cross(grasp_x_axis, grasp_y_axis), grasp_z_axis) < 0:
                grasp_y_axis = -grasp_y_axis

            # Random tilt by ±0 degrees around the grasp_y_axis
            theta_deg = np.random.uniform(-0, 0)
            theta_rad = np.radians(theta_deg)

            # Rotation matrix around grasp_y_axis (Rodrigues formula)
            def rodrigues_rotation_matrix(axis, theta):
                axis = axis / np.linalg.norm(axis)
                K = np.array([
                    [0, -axis[2], axis[1]],
                    [axis[2], 0, -axis[0]],
                    [-axis[1], axis[0], 0]
                ])
                I = np.eye(3)
                R = I + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
                return R

            R_tilt = rodrigues_rotation_matrix(grasp_y_axis, theta_rad)

            # Apply additional rotation: rotate grasp_x_axis / grasp_z_axis around grasp_y_axis
            grasp_x_axis = R_tilt @ grasp_x_axis
            grasp_z_axis = R_tilt @ grasp_z_axis

            # # Build the rotation matrix
            # R = np.column_stack((grasp_x_axis, grasp_y_axis, grasp_z_axis))

            grasp_center = grasp_center - 0.2 * grasp_z_axis
            # # save the rotation matrix for visualization
            # grasp_directions.append(R)
           
            to_center = center_point - grasp_center
            to_center /= np.linalg.norm(to_center)

            if np.dot(grasp_z_axis, to_center) < 0:
                grasp_z_axis = -grasp_z_axis
                grasp_y_axis = -grasp_y_axis 

            R = np.column_stack((grasp_x_axis, grasp_y_axis, grasp_z_axis))

            # add the grasp pose to the result list
            grasp_poses_list.append((R, grasp_center))
        

            
        return grasp_poses_list

    def check_grasp_collision(
        self,
        grasp_meshes: Sequence[o3d.geometry.TriangleMesh],
        object_mesh: o3d.geometry.TriangleMesh = None,
        object_pcd = None,
        num_colisions: int = 10,
        tolerance: float = 0.00001) -> bool:

        # Combine gripper meshes
        combined_gripper = o3d.geometry.TriangleMesh()
        for mesh in grasp_meshes:
            combined_gripper += mesh

        # Sample points from mesh
        num_points = 500  # Number of points for subsampling both meshes
        gripper_pcl = combined_gripper.sample_points_uniformly(number_of_points=num_points)
        
        # Determine which object representation to use
        if object_mesh is not None:
            object_pcl = object_mesh.sample_points_uniformly(number_of_points=num_points)
        elif object_pcd is not None:
            object_pcl = object_pcd
        else:
            raise ValueError("Must provide at least one parameter from object_mesh or object_pcd")
        # Build KD tree for object points
        is_collision = False
        object_kd_tree = o3d.geometry.KDTreeFlann(object_pcl)
        collision_count = 0
        for point in gripper_pcl.points:
            if point[2] < 0.005:
                return True
            [_, idx, distance] = object_kd_tree.search_knn_vector_3d(point, 1)
            if distance[0] < tolerance:
                collision_count += 1
                if collision_count >= num_colisions:
                    return True  # Collision detected

        return is_collision

    def check_grasp_containment(
        self,
        left_finger_center: np.ndarray,
        right_finger_center: np.ndarray,
        finger_length: float,
        object_pcd: o3d.geometry.PointCloud,
        num_rays: int,
        rotation_matrix: np.ndarray, # rotation-mat
        visualize_rays: bool = False  # Whether to visualize rays in PyBullet
    ) -> Tuple[bool, float, float]:
        """
        Checks if any line between the gripper fingers intersects with the object mesh.

        Args:
            left_finger_center: Center of Left finger of grasp
            right_finger_center: Center of Right finger of grasp
            finger_length: Finger Length of the gripper.
            object_pcd: Point Cloud of the target object
            num_rays: Number of rays to cast
            rotation_matrix: Rotation matrix for the grasp
            visualize_rays: Whether to visualize rays in PyBullet

        Returns:
            tuple[bool, float, float]: 
            - intersection_exists: True if any line between fingers intersects object
            - containment_ratio: Ratio of rays that hit the object
            - intersection_depth: Depth of deepest intersection point
        """
        left_center = np.asarray(left_finger_center)
        right_center = np.asarray(right_finger_center)

        # Calculate the height and bounding box of the object
        points = np.asarray(object_pcd.points)
        object_center = np.mean(points, axis=0)
        # print(f"Object center point: {object_center}")

        obj_triangle_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd=object_pcd, 
                                                                                          alpha=0.016)
        
        obj_triangle_mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(obj_triangle_mesh)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(obj_triangle_mesh_t)

        hand_width = np.linalg.norm(left_center-right_center)
        ray_direction = (left_center - right_center)/hand_width
        finger_vec = np.array([0, 0, finger_length])

        # Store ray start and end points for visualization
        ray_start_points = []
        ray_end_points = []
        
        # ===== Calculate gripper width direction =====
        # print("Calculating gripper width direction...")
        # Calculate vector in gripper width direction
        # First calculate finger_vec direction in world coordinates
        world_finger_vec = rotation_matrix.dot(finger_vec)
        # Calculate width direction vector
        width_direction = np.cross(ray_direction, world_finger_vec)
        width_direction = width_direction / np.linalg.norm(width_direction)
        
        # Define width direction parameters
        width_planes = 1  # Number of planes on each side in width direction
        width_offset = 0.01  # gripper thickness 0.02
        
        # ===== Generate multiple parallel ray planes =====
        # print("Generating multiple parallel ray planes...")
        # Central plane (original plane)
        rays = []
        contained = False
        
        # Parallel planes on both sides in width direction
        for plane in range(1, width_planes + 1):
            # Calculate current plane offset
            current_offset = width_offset * plane
            
            # Right side plane
            for i in range(num_rays):
                # Calculate sampling point along length direction, and offset in width direction
                right_point = right_center - rotation_matrix.dot(0.5*finger_vec) + rotation_matrix.dot((i/num_rays)*finger_vec) + width_direction * current_offset
                # Add ray from right offset point to left offset point
                rays.append([np.concatenate([right_point, ray_direction])])
                
                # Store ray start and end points for visualization - using actual finger width
                ray_start_points.append(right_point)
                ray_end_points.append(right_point + ray_direction * hand_width)
            
            # Left side plane
            for i in range(num_rays):
                # Calculate sampling point along length direction, and offset in width direction
                right_point = right_center - rotation_matrix.dot(0.5*finger_vec) + rotation_matrix.dot((i/num_rays)*finger_vec) - width_direction * current_offset
                # Add ray from right offset point to left offset point
                rays.append([np.concatenate([right_point, ray_direction])])
                
                # Store ray start and end points for visualization - using actual finger width
                ray_start_points.append(right_point)
                ray_end_points.append(right_point + ray_direction * hand_width)
        
        # print(f"Total of {len(rays)} rays generated")
        

        
        # Execute ray casting
        rays_t = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
        ans = scene.cast_rays(rays_t)
        
        # Process ray casting results
        rays_hit = 0
        max_interception_depth_score = o3d.core.Tensor([0.0], dtype=o3d.core.Dtype.Float32)
        center_rays_from_left = []
        center_rays_from_right = []
        # Track hits for left and right side ray planes
        left_side_hit = False
        right_side_hit = False
    
        # Process results for all rays
        # print("Processing ray casting results...")

        for idx, hit_point in enumerate(ans['t_hit']):
            # Use actual finger width to determine if ray hit the object
            if hit_point < hand_width:
                rays_hit += 1
                
                # Determine if ray belongs to left or right side plane
                total_rays_count = len(rays)
                half_rays_count = total_rays_count // 2
                
                if idx < half_rays_count:
                    # Ray from right side plane
                    right_side_hit = True
                else:
                    # Ray from left side plane
                    left_side_hit = True
                
                # Only calculate depth for rays in the center plane (original plane)
                if idx < num_rays:
                    left_new_center = left_center - rotation_matrix.dot(0.5*finger_vec) + rotation_matrix.dot((idx/num_rays)*finger_vec)
                    right_new_center = right_center - rotation_matrix.dot(0.5*finger_vec) + rotation_matrix.dot((idx/num_rays)*finger_vec)
                    center_rays_from_left.append([np.concatenate([left_new_center, -ray_direction])])
                    center_rays_from_right.append([np.concatenate([right_new_center, ray_direction])])
        
        # Only consider contained when both left and right side planes have at least one ray hit
        contained = left_side_hit and right_side_hit
        
        containment_ratio = 0.0
        if contained:
            # Process rays from left side (only for center plane)
            if center_rays_from_left and center_rays_from_right:
                left_rays_t = o3d.core.Tensor(center_rays_from_left, dtype=o3d.core.Dtype.Float32)
                ans_left = scene.cast_rays(left_rays_t)
                right_rays_t = o3d.core.Tensor(center_rays_from_left, dtype=o3d.core.Dtype.Float32)
                ans_right = scene.cast_rays(right_rays_t)

                if(len(ans_left['t_hit']) == len(ans_right['t_hit'])):
                    hit_point_number = len(ans_left['t_hit'])
                    for idx in range(hit_point_number):
                        interception_depth = hand_width - ans_left['t_hit'][idx].item() - ans_right['t_hit'][idx].item()
                        max_interception_depth_score = max(max_interception_depth_score, interception_depth)


        # print(f"the max interception depth is {max_interception_depth_score}")
        # Calculate overall ray hit ratio
        total_rays = len(rays)
        containment_ratio = rays_hit / total_rays
        # print(f"Ray hit ratio: {containment_ratio:.4f} ({rays_hit}/{total_rays})")
        
        grasp_center = (left_center + right_center) / 2
        
        distance_to_center = np.linalg.norm(grasp_center - object_center)
        
        # Calculate distance score (closer distance gives higher score)
        center_score = np.exp(-distance_to_center**2 / (2 * 0.05**2))
      
        # Incorporate both distance scores into final quality score
        final_quality = 20 * containment_ratio + 10 * center_score + 70 * (1-np.exp(-max_interception_depth_score * 1000))
        
        # print(f"Grasp center: {grasp_center}")
        # print(f"Total distance: {distance_to_center}m, Total distance score: {center_score}")
        # print(f"Final quality score: {final_quality}")


        return contained, final_quality

    def compute_grasp_poses(self, best_grasp):
        """
        Calculate pre-grasp and final grasp poses based on the best grasp

        Parameters:
            best_grasp: Best grasp pose (R, grasp_center)
            
        Returns:
            tuple: (pose1_pos, pose1_orn, pose2_pos, pose2_orn)
        """
        R_mat, grasp_center = best_grasp

        ee_target_pos = grasp_center

        # Convert rotation matrix to Euler angles (degrees)
        rot_world = Rotation.from_matrix(R_mat)
        euler_world = rot_world.as_euler('xyz', degrees=True)

        # Transform to quaternion using scipy
        pose2_orn = rot_world.as_quat()

        # Compute pre-grasp pose (20 cm back along the Z axis)
        z_axis = R_mat[:, 2]
        pose1_pos = ee_target_pos - 0.2 * z_axis
        pose1_orn = pose2_orn

        return pose1_pos, pose1_orn, ee_target_pos, pose2_orn
    
    def final_compute_poses(self, merged_pcd, merged_color=None, visualize=False, grasp_type='other_things', save_dir=None):
        """
        Calculate pre-grasp and final grasp poses based on the best grasp
        
        Parameters:
            best_grasp: Best grasp pose (R, grasp_center)
        """
        if merged_pcd is None:
            print("Error: Cannot merge point clouds, grasping terminated")
            return None, None, None, None, None, None

        # AnyGrasp branch
        if self.use_anygrasp:
            # Convert to numpy arrays if needed
            if isinstance(merged_pcd, o3d.geometry.PointCloud):
                pts = np.asarray(merged_pcd.points).astype(np.float32)
                clr = np.asarray(merged_pcd.colors).astype(np.float32) if merged_pcd.has_colors() else np.ones_like(pts) * 0.5
            else:
                pts = np.asarray(merged_pcd).astype(np.float32)
                clr = np.asarray(merged_color).astype(np.float32) if merged_color is not None else np.ones_like(pts) * 0.5
            return self.anygrasp_compute_poses(pts, clr, visualize=visualize)

        # Original OBB-based method below
        # If input is numpy array, convert to Open3D point cloud
        if isinstance(merged_pcd, np.ndarray):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(merged_pcd)
            if merged_color is not None:
                color = np.asarray(merged_color)
                if color.max() > 1.1:
                    color = color / 255.0
                pcd.colors = o3d.utility.Vector3dVector(color)
            merged_pcd = pcd

        # Get boundary box information
        center = self.bbox_center
        rotation_matrix = self.bbox_rotation_matrix
        
        # Get rotated boundary box coordinates
        points_rotated = np.dot(np.asarray(merged_pcd.points) - center, rotation_matrix)
        min_point_rotated = np.min(points_rotated, axis=0)
        max_point_rotated = np.max(points_rotated, axis=0)
        
        print(f"\nBoundary box information:")
        print(f"Centroid coordinates: {center}")
        print(f"Minimum point in rotated coordinate system: {min_point_rotated}")
        print(f"Maximum point in rotated coordinate system: {max_point_rotated}")
        
        # Generate grasping candidates
        print("\nGenerating grasping candidates...")
        if grasp_type == 'flavoring':
            sampled_grasps_state = self.sample_grasps_state_for_flavoring(
                center, 
                num_grasps=500, 
                rotation_matrix=rotation_matrix,
                min_point_rotated=min_point_rotated,
                max_point_rotated=max_point_rotated,
                center_rotated=center
            )
        else:
            sampled_grasps_state = self.sample_grasps_state(
                center, 
                num_grasps=500, 
                rotation_matrix=rotation_matrix,
                min_point_rotated=min_point_rotated,
                max_point_rotated=max_point_rotated,
                center_rotated=center
            )
            

        # Create mesh for each grasping candidate
        all_grasp_meshes = []
        
        for grasp in sampled_grasps_state:
            R, grasp_center = grasp
            # print(f"Grasp center: {grasp_center}, Rotation matrix: {R}")
            gripper_meshes = create_grasp_mesh(center_point=grasp_center, rotation_matrix=R)
            all_grasp_meshes.append(gripper_meshes)
        # Visualize all grasp candidates
        print("\nVisualizing all grasp candidates...")
        # Create triangle mesh from point cloud for visualization
        obj_triangle_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd=merged_pcd, 
            alpha=0.08
        )
        # # Create box mesh (0.1 x 0.09 x 0.06) and fit to merged_pcd via ICP
        # _box_dims = np.array([0.1, 0.09, 0.06])
        # _box_src = o3d.geometry.TriangleMesh.create_box(width=_box_dims[0], height=_box_dims[1], depth=_box_dims[2])
        # _box_src.translate(-_box_dims / 2)  # center at origin
        # _box_src.compute_vertex_normals()

        # _init_T = np.eye(4)
        # _init_T[:3, :3] = rotation_matrix
        # _init_T[:3, 3] = center

        # _box_pcd = _box_src.sample_points_uniformly(number_of_points=5000)
        # _icp = o3d.pipelines.registration.registration_icp(
        #     _box_pcd,
        #     merged_pcd,
        #     max_correspondence_distance=0.02,
        #     init=_init_T,
        #     estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        #     criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200),
        # )
        # print(f"Box ICP fitness: {_icp.fitness:.4f}, RMSE: {_icp.inlier_rmse:.6f}")

        # # Constrain: box Y axis (height 0.09) must align with world Z so the
        # # 0.1x0.06 face is parallel to the XY plane.
        # _R = _icp.transformation[:3, :3]
        # _t = _icp.transformation[:3, 3]
        # _z_sign = np.sign(_R[2, 1]) if abs(_R[2, 1]) > 1e-6 else 1.0
        # _new_y = np.array([0.0, 0.0, _z_sign])
        # _x_proj = _R[:, 0].copy(); _x_proj[2] = 0.0
        # _x_norm = np.linalg.norm(_x_proj)
        # if _x_norm < 1e-6:
        #     _x_proj = _R[:, 2].copy(); _x_proj[2] = 0.0
        #     _x_norm = np.linalg.norm(_x_proj)
        # _new_x = _x_proj / _x_norm
        # _new_z = np.cross(_new_x, _new_y)
        # _T_constrained = np.eye(4)
        # _T_constrained[:3, :3] = np.column_stack([_new_x, _new_y, _new_z])
        # _T_constrained[:3, 3] = _t

        # obj_triangle_mesh = o3d.geometry.TriangleMesh.create_box(width=_box_dims[0], height=_box_dims[1], depth=_box_dims[2])
        # obj_triangle_mesh.translate(-_box_dims / 2)
        # obj_triangle_mesh.compute_vertex_normals()
        # obj_triangle_mesh.transform(_T_constrained)
        
        # Prepare list of meshes for visualization
        vis_meshes = [obj_triangle_mesh]
        
        # Add all grasp meshes to list
        for grasp_mesh in all_grasp_meshes:
            vis_meshes.extend(grasp_mesh)
            
        # Call visualization function
        if visualize:
            visualize_3d_objs(vis_meshes)
        # Evaluate grasping quality
        print("\nEvaluating grasping quality...")
        
        best_grasp = None
        best_grasp_mesh = None
        highest_quality = 0

        if_collision = False

        for (pose, grasp_mesh) in zip(sampled_grasps_state, all_grasp_meshes):
  
            if_collision = self.check_grasp_collision(grasp_mesh, object_mesh= obj_triangle_mesh, object_pcd = None , num_colisions=1)

            if not if_collision:
                R, grasp_center = pose
                
                valid_grasp, grasp_quality = self.check_grasp_containment(
                    grasp_mesh[0].get_center(), 
                    grasp_mesh[1].get_center(),
                    finger_length=0.13,
                    object_pcd=merged_pcd,
                    num_rays=150,
                    rotation_matrix=pose[0],
                    visualize_rays=False
                )
                
                if valid_grasp:
                    # Store valid grasp with quality score
                    self.valid_grasps_list.append((grasp_quality, pose, grasp_mesh))
                    
                    if grasp_quality > highest_quality:
                        highest_quality = grasp_quality
                        best_grasp = pose
                        best_grasp_mesh = grasp_mesh
                        print(f"Found better grasp, quality: {grasp_quality}")
        
        # Sort valid grasps by quality (descending) and get top 10
        self.valid_grasps_list.sort(key=lambda x: x[0], reverse=True)
        self.top_10_grasps = self.valid_grasps_list[:10]
        top_10_grasps = self.compute_top_10_grasp_poses()

        if best_grasp is None:
            print("No valid grasp found!")
            return None, None, None, None, None, None
        
        print(f"\nFound best grasp, quality score: {highest_quality}")
        
        # Calculate grasping pose (only calculate once)
        grasp_poses = self.compute_grasp_poses(best_grasp)
        pose1_pos, pose1_orn, pose2_pos, pose2_orn = grasp_poses
              
        # Add visualization code after finding the best grasp
        if best_grasp is not None and visualize:
            # Create triangle mesh from point cloud
            obj_triangle_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                pcd=merged_pcd, 
                alpha=0.08
            )
            
            # Prepare list of meshes for visualization
            vis_meshes = [obj_triangle_mesh]
            
            # Add best grasp mesh to list
            vis_meshes.extend(best_grasp_mesh)
            
            # Call visualization function
            visualize_3d_objs(vis_meshes)

        if save_dir is not None and best_grasp_mesh is not None:
            save_path = Path(save_dir)
            save_path.mkdir(parents=True, exist_ok=True)
            existing_count = len(list(save_path.glob("*_object.ply")))
            idx = existing_count + 1
            o3d.io.write_triangle_mesh(str(save_path / f"{idx:03d}_object.ply"), obj_triangle_mesh)
            combined_gripper = o3d.geometry.TriangleMesh()
            for m in best_grasp_mesh:
                combined_gripper += m
            o3d.io.write_triangle_mesh(str(save_path / f"{idx:03d}_gripper.ply"), combined_gripper)
            print(f"Saved grasp meshes → {save_path / f'{idx:03d}_*.ply'}")

        return pose1_pos, pose1_orn, pose2_pos, pose2_orn, top_10_grasps, self.valid_grasps_list

    def compute_top_10_grasp_poses(self):
        """
        Compute prep_pose for the top 10 grasps using compute_grasp_poses function
        
        Returns:
            list: List of tuples containing (quality, prep_pose_pos, prep_pose_orn, grasp_pose_pos, grasp_pose_orn) for top 10 grasps
        """
        if len(self.top_10_grasps) == 0:
            print("No top 10 grasps available.")
            return None

        top_10_poses = []
        
        for _, pose, _ in self.top_10_grasps:
            # Compute grasp poses using the existing function
            pose1_pos, pose1_orn, pose2_pos, pose2_orn = self.compute_grasp_poses(pose)
            # Store prep pose (pose1), and final grasp pose (pose2)
            top_10_poses.append({
                'prep_pose_pos': pose1_pos,
                'prep_pose_orn': pose1_orn,
                'grasp_pose_pos': pose2_pos,
                'grasp_pose_orn': pose2_orn
            })
        
        return top_10_poses