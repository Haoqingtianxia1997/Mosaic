import open3d as o3d
import numpy as np
import subprocess
import re
import shlex
import yaml
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial import cKDTree
from src.VLM_agent.agent import VLM_agent


ROS2_SERVICE_TIMEOUT_SEC = 15


def visualize_obb_and_center(all_points_arr, all_colors_arr, obb_corners, center):
    """
    Visualize point cloud with OBB and center using Open3D
    """

    # Create point cloud for OBB bounding box
    pcd_vis = o3d.geometry.PointCloud()
    pcd_vis.points = o3d.utility.Vector3dVector(all_points_arr)
    if all_colors_arr.max() > 1.1:
        all_colors_arr = all_colors_arr / 255.0
    pcd_vis.colors = o3d.utility.Vector3dVector(all_colors_arr)

    # Create line set for OBB bounding box
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
        [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
        [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges
    ]
    colors = [[1, 0, 0] for _ in range(len(lines))]  # Red edges
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(obb_corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # Create center point sphere
    center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
    center_sphere.paint_uniform_color([0, 1, 0])  # Green
    center_sphere.translate(center)

    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0,0,0])   
    vis_geometries = [pcd_vis, line_set, center_sphere, coord_frame]
    o3d.visualization.draw_geometries(vis_geometries,
                                    window_name='Point Cloud with OBB and Center',
                                    width=800, height=600)  

def translation_only_icp(source, target, max_iter=50, tolerance=1e-6):
    """
    source: (N,3) Source point cloud
    target: (M,3) Target point cloud
    Returns: Transformed source point cloud, Translation vector T
    """
    src = np.asarray(source).copy()
    tgt = np.asarray(target).copy()

    prev_error = float('inf')
    T_total = np.zeros(3)

    for i in range(max_iter):
        # 1. Nearest neighbor matching
        tree = cKDTree(tgt)
        dist, idx = tree.query(src)
        tgt_corr = tgt[idx]

        # 2. Compute translation (i.e., average offset of matched points)
        delta_t = tgt_corr.mean(axis=0) - src.mean(axis=0)
        src += delta_t
        T_total += delta_t

        # 3. Convergence check
        mean_error = np.mean(np.linalg.norm(src - tgt_corr, axis=1))
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return src, T_total

def preprocess_pointcloud(points, colors, voxel_size=0.005, nb_points=10, radius=0.02):
    """
    Downsample, radius filter, and cluster the input point cloud and colors, keeping the largest cluster.
    Returns the processed Open3D point cloud object.
    """
    import open3d as o3d
    import numpy as np
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    if colors is not None:
        col = np.asarray(colors)
        # Check if color array is empty
        if col.size > 0 and col.max() > 1.1:
            col = col / 255.0
        # Only set colors if color array is not empty
        if col.size > 0:
            pcd.colors = o3d.utility.Vector3dVector(col)

    # Voxel downsampling
    pcd = pcd.voxel_down_sample(voxel_size)

    # Radius filtering
    pcd, _ = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)

    # Clustering analysis, keeping the largest cluster (only clustering in XY direction)
    if len(pcd.points) > 0:
        # Extract XY coordinates for clustering, ignoring Z coordinate
        points_3d = np.asarray(pcd.points)
        points_xy = points_3d[:, :2]  # Only take X and Y coordinates

        # Create temporary 2D point cloud for clustering
        pcd_2d = o3d.geometry.PointCloud()
        # Expand 2D points to 3D (set Z to 0) for clustering algorithm
        points_xy_3d = np.column_stack([points_xy, np.zeros(len(points_xy))])
        pcd_2d.points = o3d.utility.Vector3dVector(points_xy_3d)

        # Use DBSCAN clustering algorithm (only in XY plane)
        labels = np.array(pcd_2d.cluster_dbscan(eps=0.01, min_points=10, print_progress=False))

        # If clusters are found
        if len(labels) > 0 and np.max(labels) >= 0:
            # Count points in each cluster, -1 indicates noise points
            unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
            
            if len(unique_labels) > 0:
                # Find the label of the largest cluster
                largest_cluster_label = unique_labels[np.argmax(counts)]
                
                # Save points belonging to the largest cluster
                largest_cluster_indices = labels == largest_cluster_label
                largest_cluster_points = points_3d[largest_cluster_indices]

                # Create new point cloud object
                pcd_filtered = o3d.geometry.PointCloud()
                pcd_filtered.points = o3d.utility.Vector3dVector(largest_cluster_points)

                # If color information is available, keep the corresponding colors
                if pcd.has_colors():
                    largest_cluster_colors = np.asarray(pcd.colors)[largest_cluster_indices]
                    pcd_filtered.colors = o3d.utility.Vector3dVector(largest_cluster_colors)

                print(f"XY plane clustering completed: found {len(unique_labels)} clusters, kept largest cluster with {np.max(counts)} points")
                return pcd_filtered
            else:
                print("XY plane clustering failed: no valid clusters found, returning original point cloud")
                return pcd
        else:
            print("XY plane clustering failed: all points are noise, returning original point cloud")
            return pcd
    else:
        print("Point cloud is empty, returning empty point cloud")
        return pcd

# def filter_and_merge_icp_translation_only(
#         points_l, colors_l, points_r, colors_r, 
#         voxel_size=0.005, nb_points=10, radius=0.02):

#     pcd_l = preprocess_pointcloud(points_l, colors_l, voxel_size, nb_points, radius)
#     pcd_r = preprocess_pointcloud(points_r, colors_r, voxel_size, nb_points, radius)

#     arr_l = np.asarray(pcd_l.points)
#     arr_r = np.asarray(pcd_r.points)
    
#     # 3. Manual ICP-only translation
#     aligned_l, T = translation_only_icp(arr_l, arr_r)

#     # 4. Merge point clouds
#     all_points_arr = np.vstack([aligned_l, arr_r])
#     if pcd_l.has_colors() and pcd_r.has_colors():
#         all_colors_arr = np.vstack([np.asarray(pcd_l.colors), np.asarray(pcd_r.colors)])
#     else:
#         all_colors_arr = np.ones_like(all_points_arr)

#     return all_points_arr, all_colors_arr, T


def filter_and_merge_icp_translation_only(
        points_l, colors_l, points_r, colors_r, 
        voxel_size=0.005, nb_points=10, radius=0.01):
    
    pcd_l = preprocess_pointcloud(points_l, colors_l, voxel_size, 10, 0.01)
    pcd_r = preprocess_pointcloud(points_r, colors_r, voxel_size, 10, 0.01)

    # # 1. No downsampling, directly use input points
    # pcd_l = o3d.geometry.PointCloud()
    # pcd_l.points = o3d.utility.Vector3dVector(points_l)
    # if colors_l is not None:
    #     pcd_l.colors = o3d.utility.Vector3dVector(colors_l)

    # pcd_r = o3d.geometry.PointCloud()
    # pcd_r.points = o3d.utility.Vector3dVector(points_r)
    # if colors_r is not None:
    #     pcd_r.colors = o3d.utility.Vector3dVector(colors_r)

    arr_l = np.asarray(pcd_l.points)
    arr_r = np.asarray(pcd_r.points)

    # 2. Manual ICP-only translation
    aligned_l, T = translation_only_icp(arr_l, arr_r)

    # 3. Merge point clouds
    all_points_arr = np.vstack([aligned_l, arr_r])
    if pcd_l.has_colors() and pcd_r.has_colors():
        all_colors_arr = np.vstack([np.asarray(pcd_l.colors), np.asarray(pcd_r.colors)])
    else:
        all_colors_arr = np.ones_like(all_points_arr)

    # 4. Preprocess the merged point cloud
    merged_pcd = preprocess_pointcloud(all_points_arr, all_colors_arr, voxel_size, nb_points, radius)
    filtered_points = np.asarray(merged_pcd.points)
    filtered_colors = np.asarray(merged_pcd.colors) if merged_pcd.has_colors() else None

    return filtered_points, filtered_colors, T    

def filter_and_merge_icp(
        points_l, colors_l, points_r, colors_r, 
        voxel_size=0.005, nb_points=10, radius=0.02, 
        icp_max_corr=0.03, icp_threshold=1.0):
    """
    :param points_l, colors_l: left point cloud and colors (Nx3)
    :param points_r, colors_r: right point cloud and colors (Mx3)
    :return: all_points_arr, all_colors_arr, transformation (left aligned to right)
    """

    pcd_l = preprocess_pointcloud(points_l, colors_l, voxel_size, nb_points, radius)
    pcd_r = preprocess_pointcloud(points_r, colors_r, voxel_size, nb_points, radius)

    # 4. ICP registration (left aligned to right)
    reg = o3d.pipelines.registration.registration_icp(
        pcd_l, pcd_r, icp_max_corr,
        np.eye(4),  # Initial transformation
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    T = reg.transformation
    # Transform left point cloud
    pcd_l.transform(T)

    # 5. Merge point clouds
    all_points_arr = np.vstack([
        np.asarray(pcd_l.points), np.asarray(pcd_r.points)
    ])
    if pcd_l.has_colors() and pcd_r.has_colors():
        all_colors_arr = np.vstack([
            np.asarray(pcd_l.colors), np.asarray(pcd_r.colors)
        ])
    else:
        all_colors_arr = np.ones_like(all_points_arr)  # If no color, set to white

    return all_points_arr, all_colors_arr, T  # T is optional

def sphere_at(point, color, radius=0.02):
    s = o3d.geometry.TriangleMesh.create_sphere(radius)
    s.paint_uniform_color(color)
    s.translate(point)
    return s

def open3d_show(all_points_arr, all_colors_arr, target_center_point, target_max_z_point, center_world_points):
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points_arr)
    if all_colors_arr.max() > 1.1:   # If colors are in 0~255 range
        all_colors_arr = all_colors_arr / 255.0
    pcd.colors = o3d.utility.Vector3dVector(all_colors_arr)

    vis_geoms = [pcd]
    vis_geoms.append(sphere_at(target_center_point, color=[1,0,0], radius=0.012))# Red sphere represents centroid
    vis_geoms.append(sphere_at(target_max_z_point, color=[0,0,1], radius=0.01))# Blue sphere represents max Z point
    vis_geoms.append(sphere_at(center_world_points, color=[0,1,0], radius=0.01))# Green sphere represents target point
    vis_geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0,0,0]))  # Coordinate frame
    o3d.visualization.draw_geometries(vis_geoms)

def call_ros2_service(service_name, service_type, args_dict):
    # Convert dictionary to a single line YAML string
    arg_str = yaml.dump(args_dict, default_flow_style=True, sort_keys=False).strip()
    cmd = [
        "ros2", "service", "call",
        service_name,
        service_type,
        arg_str
    ]
    print(f"\n🚀 Calling service: {' '.join(cmd)}")

    try:
        result = subprocess.check_output(
            cmd,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=ROS2_SERVICE_TIMEOUT_SEC,
        )
        print("✅ Service call returned:")
        print(result)
        if "success=True" in result:
            return True
        else:
            print("❌ Service reported failure.")
            return False
    except subprocess.TimeoutExpired:
        print(f"❌ Service call timeout after {ROS2_SERVICE_TIMEOUT_SEC}s: {service_name}")
        return False
    except subprocess.CalledProcessError as e:
        print("❌ Service call failed:")
        print(e.output)
        return False

def call_fk_service():
    """
    Call the `ros2 service call /fk_service action_interfaces/srv/Fk "{}"` via shell
    Returns a tuple of (x, y, z) float values
    """
    cmd = 'ros2 service call /fk_service action_interfaces/srv/Fk "{}"'
    # Use shlex.split to avoid issues with quotes
    try:
        result = subprocess.run(
            shlex.split(cmd),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=ROS2_SERVICE_TIMEOUT_SEC,
        )
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(
            f"fk_service timeout after {ROS2_SERVICE_TIMEOUT_SEC}s"
        ) from exc

    if result.returncode != 0:
        raise RuntimeError(f"ros2 命令失败: {result.stderr}")

    # Find x=..., y=..., z=... in the output
    # Compatible with the following two formats:
    #   action_interfaces.srv.Fk_Response(x=0.39, y=0.29, z=0.30)
    #   x: 0.39
    pattern = r'[xyz]\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)'
    matches = re.findall(pattern, result.stdout)
    if len(matches) >= 3:
        x, y, z = map(float, matches[:3])
        return x, y, z
    else:
        # Second YAML style
        yaml_pat = r'^\s*[xyz]:\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)\s*$'
        found = {}
        for line in result.stdout.splitlines():
            m = re.match(yaml_pat, line)
            if m:
                key = line.strip().split(':')[0]
                found[key] = float(m.group(1))
        if {'x', 'y', 'z'} <= found.keys():
            return found['x'], found['y'], found['z']
        raise ValueError("Cannot find x y z in service response")

def get_cam_world_points(
    client,
    target,
    rgb_path,
    depth_path,
    pixels_to_world_func,
    name,
    agent_image_path=None,
    segmenter=None      
    
):
    """
    target: Perception target
    rgb_path: Path to RGB image
    depth_path: Path to depth image (.npy)
    pixels_to_world_func: Pixel to world coordinate transformation function (e.g., pixels_to_world_left/right)
    agent_image_path: If you want to specify an image to send to VLM_agent, you can use this, otherwise it will use rgb_path
    """
    img_path = agent_image_path if agent_image_path else rgb_path
    if_find, response,  box_center_point, seg_center_point, all_seg_points = VLM_agent(target, img_path, name, client, segmenter)  # Call VLM agent to perceive target and get pixel coordinates
    if not if_find or box_center_point is None or seg_center_point is None or all_seg_points is None:
        print(f"❌ Failed to perceive target: {target}")
        return if_find, response, None, None, None
    
    center_pixel_point = [seg_center_point] if seg_center_point is not None else [box_center_point]
    print(f"Perceived pixel points ({img_path}): {center_pixel_point}")
    all_pixel_points = all_seg_points if all_seg_points is not None else []
    print(f"All pixel points ({img_path}): {all_pixel_points}")

    depth_img = np.load(depth_path)
    rgb_img = cv2.cvtColor(cv2.imread(rgb_path), cv2.COLOR_BGR2RGB)
    result = []

    for points in [center_pixel_point, all_seg_points]:
        if isinstance(points, tuple):
            points = [points]
        elif isinstance(points, list) and isinstance(points[0], int):
            # Compatible with formats like [x, y]
            points = [tuple(points)]
        
        pixel_points = np.array(points, dtype=int)
        u = pixel_points[:, 0]
        v = pixel_points[:, 1]
        depths = depth_img[v, u]
        target_point = pixel_points.tolist()
        world_points, color = pixels_to_world_func(target_point, depths, rgb_img=rgb_img)
        result.append(world_points)
    
    center_world_points, all_world_points = result
  
    return if_find, response, center_world_points, all_world_points, color


def merge_points_icp(world_points_r, world_points_l, threshold=0.02, visualize=False):
    """
    Align the left point cloud to the right point cloud using ICP, and then merge the two parts.
    world_points_r, world_points_l: list/ndarray, Nx3, right and left camera point clouds
    threshold: ICP convergence distance
    visualize: Whether to pop up a window to preview the point cloud
    Returns: merged_points, transformation (transformation matrix from left to right)
    """
    # Build Open3D point clouds
    pcd_r = o3d.geometry.PointCloud()
    pcd_l = o3d.geometry.PointCloud()
    pcd_r.points = o3d.utility.Vector3dVector(np.array(world_points_r))
    pcd_l.points = o3d.utility.Vector3dVector(np.array(world_points_l))

    # Perform ICP registration
    reg = o3d.pipelines.registration.registration_icp(
        pcd_l, pcd_r, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    pcd_l.transform(reg.transformation)

    # Merge point clouds
    merged_points = np.vstack([
        np.asarray(pcd_r.points),
        np.asarray(pcd_l.points)
    ])
    if visualize:
        pcd_merged = o3d.geometry.PointCloud()
        pcd_merged.points = o3d.utility.Vector3dVector(merged_points)
        o3d.visualization.draw_geometries([pcd_merged], window_name='ICP Merged Point Cloud')
    return merged_points, reg.transformation