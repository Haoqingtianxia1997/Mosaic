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
