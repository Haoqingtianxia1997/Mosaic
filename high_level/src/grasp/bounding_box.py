import numpy as np
import open3d as o3d


def compute_obb(points_cloud: np.ndarray) -> np.ndarray:
    """
    Calculate the oriented bounding box (OBB) of the point cloud
    Implemented based on PCA analysis in the XY plane
    
    Returns:
    self: Returns self to support method chaining
    """
    # Check if point cloud is empty
    if len(points_cloud) == 0:
        raise ValueError("Point cloud is empty, cannot calculate bounding box")
    
    # Calculate point cloud centroid
    center = np.mean(points_cloud, axis=0)
    
    # 1. Project point cloud onto XY plane
    points_xy = points_cloud.copy()
    points_xy[:, 2] = 0  # Set Z coordinate to 0, projecting onto XY plane
    
    # 2. Perform PCA on the projected point cloud to find principal axes
    xy_mean = np.mean(points_xy, axis=0)
    xy_centered = points_xy - xy_mean
    cov_xy = np.cov(xy_centered.T)[:2, :2]  # Only take XY plane covariance
    eigenvalues, eigenvectors = np.linalg.eigh(cov_xy)
    # Sort eigenvalues and eigenvectors (descending)
    idx = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    
    # 3. Get principal axis directions, these are the rotation directions in the XY plane
    main_axis_x = np.array([eigenvectors[0, 0], eigenvectors[1, 0], 0])
    main_axis_y = np.array([eigenvectors[0, 1], eigenvectors[1, 1], 0])
    main_axis_z = np.array([0, 0, 1])  # Z axis remains vertical
    
    # Normalize main axes
    main_axis_x = main_axis_x / np.linalg.norm(main_axis_x)
    main_axis_y = main_axis_y / np.linalg.norm(main_axis_y)
    
    # 4. Build rotation matrix
    rotation_matrix = np.column_stack((main_axis_x, main_axis_y, main_axis_z))
    
    # 5. Rotate point cloud to new coordinate system
    points_rotated = np.dot(points_cloud - xy_mean, rotation_matrix)

    
    # 6. Calculate bounding box in new coordinate system
    min_point_rotated = np.min(points_rotated, axis=0)
    max_point_rotated = np.max(points_rotated, axis=0)
    
    # Calculate dimensions of rotated bounding box
    obb_dims = max_point_rotated - min_point_rotated
    height = obb_dims[2]
    
    # 7. Calculate the 8 vertices of the bounding box (in rotated coordinate system)
    bbox_corners_rotated = np.array([
        [min_point_rotated[0], min_point_rotated[1], min_point_rotated[2]],
        [max_point_rotated[0], min_point_rotated[1], min_point_rotated[2]],
        [max_point_rotated[0], max_point_rotated[1], min_point_rotated[2]],
        [min_point_rotated[0], max_point_rotated[1], min_point_rotated[2]],
        [min_point_rotated[0], min_point_rotated[1], max_point_rotated[2]],
        [max_point_rotated[0], min_point_rotated[1], max_point_rotated[2]],
        [max_point_rotated[0], max_point_rotated[1], max_point_rotated[2]],
        [min_point_rotated[0], max_point_rotated[1], max_point_rotated[2]],
    ])
    
    # 8. Transform vertices back to original coordinate system
    obb_corners = np.dot(bbox_corners_rotated, rotation_matrix.T) + xy_mean

    return obb_corners, rotation_matrix, center




# def compute_obb(points_cloud: np.ndarray) -> np.ndarray:
#     """
#     Calculate the oriented bounding box (OBB) of the point cloud
#     Implemented based on PCA analysis in the XY plane
#     Also visualize the convex hull mesh and bounding box in Open3D window.
#     Returns:
#     obb_corners, rotation_matrix, center
#     """
#     if len(points_cloud) == 0:
#         raise ValueError("Point cloud is empty, cannot calculate bounding box")
    
#     # Step 1: Generate Open3D point cloud from numpy array
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points_cloud)
#     pcd.paint_uniform_color([0.1, 0.7, 0.1])  # Green point cloud

#     # Step 2: Compute convex hull (mesh)
#     mesh, _ = pcd.compute_convex_hull()
#     mesh.paint_uniform_color([0.2, 0.2, 0.7])  # Blue convex hull

#     obb_points = np.asarray(mesh.vertices)
#     if obb_points.shape[0] < 4:
#         obb_points = points_cloud

#     # Step 3: Compute OBB (PCA method, consistent with your original code)
#     center = np.mean(obb_points, axis=0)
#     points_xy = obb_points.copy()
#     points_xy[:, 2] = 0
#     xy_mean = np.mean(points_xy, axis=0)
#     xy_centered = points_xy - xy_mean
#     cov_xy = np.cov(xy_centered.T)[:2, :2]
#     eigenvalues, eigenvectors = np.linalg.eigh(cov_xy)
#     idx = eigenvalues.argsort()[::-1]
#     eigenvalues = eigenvalues[idx]
#     eigenvectors = eigenvectors[:, idx]
#     main_axis_x = np.array([eigenvectors[0, 0], eigenvectors[1, 0], 0])
#     main_axis_y = np.array([eigenvectors[0, 1], eigenvectors[1, 1], 0])
#     main_axis_z = np.array([0, 0, 1])
#     main_axis_x = main_axis_x / np.linalg.norm(main_axis_x)
#     main_axis_y = main_axis_y / np.linalg.norm(main_axis_y)
#     rotation_matrix = np.column_stack((main_axis_x, main_axis_y, main_axis_z))
#     points_rotated = np.dot(obb_points - xy_mean, rotation_matrix)
#     min_point_rotated = np.min(points_rotated, axis=0)
#     max_point_rotated = np.max(points_rotated, axis=0)
#     bbox_corners_rotated = np.array([
#         [min_point_rotated[0], min_point_rotated[1], min_point_rotated[2]],
#         [max_point_rotated[0], min_point_rotated[1], min_point_rotated[2]],
#         [max_point_rotated[0], max_point_rotated[1], min_point_rotated[2]],
#         [min_point_rotated[0], max_point_rotated[1], min_point_rotated[2]],
#         [min_point_rotated[0], min_point_rotated[1], max_point_rotated[2]],
#         [max_point_rotated[0], min_point_rotated[1], max_point_rotated[2]],
#         [max_point_rotated[0], max_point_rotated[1], max_point_rotated[2]],
#         [min_point_rotated[0], max_point_rotated[1], max_point_rotated[2]],
#     ])
#     obb_corners = np.dot(bbox_corners_rotated, rotation_matrix.T) + xy_mean

#     # Step 4: Visualize mesh, point cloud, and OBB using Open3D
#     # 1. Draw OBB wireframe
#     lines = [
#         [0, 1], [1, 2], [2, 3], [3, 0],
#         [4, 5], [5, 6], [6, 7], [7, 4],
#         [0, 4], [1, 5], [2, 6], [3, 7]
#     ]
#     colors = [[1, 0, 0] for _ in range(len(lines))]  # Red OBB lines
#     line_set = o3d.geometry.LineSet(
#         points=o3d.utility.Vector3dVector(obb_corners),
#         lines=o3d.utility.Vector2iVector(lines),
#     )
#     line_set.colors = o3d.utility.Vector3dVector(colors)
#
#     # 2. Optional: Set transparency of convex hull mesh (requires open3d>=0.13)
#     if hasattr(mesh, 'compute_vertex_normals'):
#         mesh.compute_vertex_normals()
#     mesh.compute_triangle_normals()
#     mesh.paint_uniform_color([0.2, 0.2, 0.7])  # Still blue
#     # Open3D does not support mesh transparency directly in visualization

#     # 3. Display
#     o3d.visualization.draw_geometries([pcd, mesh, line_set],
#                                       window_name="PointCloud + ConvexHull + OBB",
#                                       width=960, height=720)

#     return obb_corners, rotation_matrix, center

