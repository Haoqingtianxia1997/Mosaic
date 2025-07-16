import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from typing import Tuple, Sequence, Optional
from scipy.spatial.transform import Rotation
from src.grasp.mesh import visualize_3d_objs,create_grasp_mesh


class GraspGeneration:
    def __init__(self, bbox_center, bbox_rotation_matrix):
        self.bbox_center = bbox_center
        self.bbox_rotation_matrix = bbox_rotation_matrix

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
            rotated_coords[0] = np.random.uniform(min_point_rotated[0], max_point_rotated[0])
            rotated_coords[1] = np.random.uniform(min_point_rotated[1], max_point_rotated[1])
            rotated_coords[2] = np.random.uniform(min_point_rotated[2], max_point_rotated[2])
            
            # convert the sampled point from the rotated coordinate system to the world coordinate system
            grasp_center = np.dot(rotated_coords, rotation_matrix.T) + center_rotated
            
            # the grasp point is not lower than the table height
            grasp_center[2] = max(grasp_center[2], table_height)
            
             
            # Z axis is vertical downward
            grasp_z_axis = np.array([0, 0, -1])
            
            # X axis (the thickness direction of the gripper) uses the long axis
            grasp_x_axis = long_axis
            
            # Y axis (the opening direction of the gripper) uses the short axis
            grasp_y_axis = short_axis

            # ensure the coordinate system direction is correct
            if np.dot(np.cross(grasp_x_axis, grasp_y_axis), grasp_z_axis) < 0:
                grasp_y_axis = -grasp_y_axis
    
            # build the rotation matrix
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

            # 生成 [-67°, -22°] 范围内的随机角度（单位：弧度）
            theta_deg = np.random.uniform(-67, -22)
            theta_rad = np.radians(theta_deg)

            # 绕 grasp_y_axis 的旋转矩阵（Rodrigues 公式）
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

            # 应用额外旋转：绕 grasp_y_axis 旋转 grasp_x/z
            grasp_x_axis = R_tilt @ grasp_x_axis
            grasp_z_axis = R_tilt @ grasp_z_axis

            # # build the rotation matrix
            # R = np.column_stack((grasp_x_axis, grasp_y_axis, grasp_z_axis))

            grasp_center = grasp_center - 0.1 * grasp_z_axis
            # # save the rotation matrix for visualization
            # grasp_directions.append(R)
            
            # add the grasp pose to the result list
            grasp_poses_list.append((R_tilt, grasp_center))
        

            
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
        num_points = 5000  # Number of points for subsampling both meshes
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
        final_quality = 0.1 * containment_ratio + 0.1 * center_score + 80 * (1-np.exp(-max_interception_depth_score * 1000))
        
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

        # 用scipy直接转四元数（保持[x,y,z,w]顺序与pybullet一致）
        pose2_orn = rot_world.as_quat()

        # 计算前置抓取位姿（在z轴方向上后退0.2米）
        z_axis = R_mat[:, 2]
        pose1_pos = ee_target_pos - 0.2 * z_axis
        pose1_orn = pose2_orn

        return pose1_pos, pose1_orn, ee_target_pos, pose2_orn
    
    def final_compute_poses(self, merged_pcd, merged_color=None, visualize=True, grasp_type='other_things'):
        """
        Calculate pre-grasp and final grasp poses based on the best grasp
        
        Parameters:
            best_grasp: Best grasp pose (R, grasp_center)
        """
        if merged_pcd is None:
            print("Error: Cannot merge point clouds, grasping terminated")
            return None, None, None, None
        
        # 如果是 numpy，转为 open3d 点云
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
                num_grasps=1000, 
                rotation_matrix=rotation_matrix,
                min_point_rotated=min_point_rotated,
                max_point_rotated=max_point_rotated,
                center_rotated=center
            )
        else:
            sampled_grasps_state = self.sample_grasps_state(
                center, 
                num_grasps=1000, 
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
        # Visualize all grasp meshes
        print("\nVisualizing all grasp candidates...")
        # Create triangle mesh from point cloud for visualization
        obj_triangle_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd=merged_pcd, 
            alpha=0.08
        )
        
        # # Prepare list of meshes for visualization
        # vis_meshes = [obj_triangle_mesh]
        
        # # Add all grasp meshes to list
        # for grasp_mesh in all_grasp_meshes:
        #     vis_meshes.extend(grasp_mesh)
            
        # # Call visualization function
        # visualize_3d_objs(vis_meshes)
        # # Evaluate grasping quality
        # print("\nEvaluating grasping quality...")
        
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
                
                if valid_grasp and grasp_quality > highest_quality:
                    highest_quality = grasp_quality
                    best_grasp = pose
                    best_grasp_mesh = grasp_mesh
                    print(f"Found better grasp, quality: {grasp_quality}")
        
        if best_grasp is None:
            print("No valid grasp found!")
            return None, None, None, None
        
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

        return pose1_pos, pose1_orn, pose2_pos, pose2_orn