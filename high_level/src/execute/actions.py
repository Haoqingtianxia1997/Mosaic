import subprocess
import re
import shlex
import json
import time
import os
from networkx import center
import yaml
import numpy as np
import cv2
import open3d as o3d
import traceback
from scipy.spatial import cKDTree
from src.VLM_agent.agent import VLM_agent  
from src.pixel_world.pixel_and_world import pixels_to_world_left, pixels_to_world_right, world_to_pixels_left, world_to_pixels_right
from src.transcribe.tts import run_tts, play_text_to_speech
from src.grasp.bounding_box import compute_obb
from src.grasp.grasp_generation import GraspGeneration

def visualize_obb_and_center(all_points_arr, all_colors_arr, obb_corners, center):
    """
    Visualize point cloud with OBB and center using Open3D
    """
    import open3d as o3d

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
        result = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        print("✅ Service call returned:")
        print(result)
        if "success=True" in result:
            return True
        else:
            print("❌ Service reported failure.")
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
    result = subprocess.run(
        shlex.split(cmd),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

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
    
):
    """
    target: Perception target
    rgb_path: Path to RGB image
    depth_path: Path to depth image (.npy)
    pixels_to_world_func: Pixel to world coordinate transformation function (e.g., pixels_to_world_left/right)
    agent_image_path: If you want to specify an image to send to VLM_agent, you can use this, otherwise it will use rgb_path
    """
    img_path = agent_image_path if agent_image_path else rgb_path
    if_find, response,  box_center_point, seg_center_point, all_seg_points = VLM_agent(target, img_path, name, client)
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

def execute_action_sequence(actions, vlm_client):
    """
    Execute a sequence of actions serially, waiting for each service to complete successfully before proceeding to the next.
    """
  
    # photo and depth image paths
    CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        
    IMAGE_FOLDER_PATH = os.path.abspath(os.path.join(
        CURRENT_DIR, "../../../manipulation_ws/saved_images"
    ))

    l_img_path = os.path.join(IMAGE_FOLDER_PATH, "l_rgb.png")
    l_depth_path = os.path.join(IMAGE_FOLDER_PATH, "l_depth.npy")
    r_img_path = os.path.join(IMAGE_FOLDER_PATH, "r_rgb.png")
    r_depth_path = os.path.join(IMAGE_FOLDER_PATH, "r_depth.npy")

    # point clouds
    all_points_arr = None # all points in world coordinates

    #declare parameters for each action type
    move_target_point = None # perceived target point
    target_max_z_point = None # highest point in z-axis
    target_center_point = None # centroid point
    center_world_points = None # perceived image target point in world coordinates
    all_points_arr = None # all points in world coordinates
    all_colors_arr = None # all colors in world coordinates

    target = None # current target, used to update the target in each action
    
    move_params = {"move_x" : None, "move_y" : None, "move_z" : None, "move_qx" : None, "move_qy" : None, "move_qz" : None, "move_qw" : None}

    grasp_params = {"x_prep": None, "y_prep": None, "z_prep": None, "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
                                "x_grasp": None, "y_grasp": None, "z_grasp": None, "qx_grasp": None, "qy_grasp": None, "qz_grasp": None, "qw_grasp": None}# grasp 
    
    rb_params = {"x_prep": None, "y_prep": None, "z_prep": None, "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
                                "x_place": None, "y_place": None, "z_place": None, "qx_place": None, "qy_place": None, "qz_place": None, "qw_place": None}# return back
    stir_time = 30
    add_times = 2
    
    # state flags
    grasped_thing = ""

    success = True  # Used to track the success status of each action

    for i, action in enumerate(actions):

        # Check if the service was successful
        if not success:
            print(f"⛔ Aborting action sequence due to failure at step {i}.")
            break

        try:
            print(f"\n▶️ Executing action {i+1}/{len(actions)}: {action}")
            act_type = action["type"]

            if action["target"] is not None:
                target = action["target"]
                print(f"🔍 Target: {target}")
            params = action.get("parameters", {})

            if "stir_time" in params:
                if params["stir_time"] is not None:
                    stir_time = params["stir_time"]
                    print(f"🔧 Parameters: {params}")
                else:
                    stir_time =  stir_time
            
            if "add_times" in params:
                if params["add_times"] is not None:
                    add_times = params["add_times"]
                    print(f"🔧 Add times: {add_times}")
                else:
                    add_times = add_times
            
            
            if act_type == "perceive":
                success = False  # Reset success status
                if target ==  grasped_thing:
                    success = True
                    print(f"✅ Target '{target}' already grasped, skipping perception.")
                    continue            
                elif target == "user person":
                    move_params = {"move_x" : 0.718029693832728, "move_y" : -0.07702313108387482, "move_z" : 0.4, "move_qx" : 0.707, "move_qy" : -0.707, "move_qz" : 0.0, "move_qw" : 0.0}
                    success = True
                    print("✅ Perceived user person, moving to target point.")
                    continue
                elif target == "spoon":
                    move_params = {"move_x" : 0.406, "move_y" : -0.313, "move_z" : 0.6, "move_qx" : 0.999, "move_qy" : 0.023, "move_qz" : 0.026, "move_qw" : 0.001}
                    grasp_params = {"x_prep": 0.406, "y_prep": -0.313, "z_prep": 0.57, "qx_prep": 0.999, "qy_prep": 0.023, "qz_prep": 0.026, "qw_prep": 0.001,
                            "x_grasp": 0.406, "y_grasp": -0.313, "z_grasp": 0.3, "qx_grasp": 0.999, "qy_grasp": 0.023, "qz_grasp": 0.026, "qw_grasp": 0.001}
                    success = True
                    print("✅ Perceived spoon, moving to target point.")
                    continue
                elif target == "soup pot":
                    move_params = {"move_x" : 0.6, "move_y" : -0.3, "move_z" : 0.32, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
                    success = True
                    print("✅ Perceived soup pot, moving to target point.")
                    continue
                elif target == "salt bottle":
                    move_params = {"move_x" : 0.434, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
                    grasp_params = {"x_prep": 0.434, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
                                "x_grasp": 0.434, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}
                    success = True
                    print("✅ Perceived salt bottle, moving to target point.")
                    continue
                elif target == "pepper bottle":
                    move_params = {"move_x" : 0.27, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
                    grasp_params = {"x_prep": 0.27, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
                                "x_grasp": 0.27, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}
                    success = True
                    print("✅ Perceived pepper bottle, moving to target point.")
                    continue

                else:  
                    print(f"Perceiving target: {target}")
                    
                    if_find_r, response_r, center_world_points_r, all_world_points_r ,color_r = get_cam_world_points(
                    vlm_client,
                    target,
                    rgb_path= r_img_path,
                    depth_path= r_depth_path,
                    pixels_to_world_func = pixels_to_world_right,
                    name= "right",
                    
                    )

                    # if_find_r, response_r, center_world_points_r, all_world_points_r, color_r = None, None, None, None, None
                    
           
                    if_find_l, response_l, center_world_points_l, all_world_points_l, color_l  = get_cam_world_points(
                        vlm_client,
                        target,
                        rgb_path= l_img_path,
                        depth_path= l_depth_path,
                        pixels_to_world_func = pixels_to_world_left,
                        name= "left",
                        
                    )

                    # if_find_l, response_l, center_world_points_l, all_world_points_l, color_l = None, None, None, None,None


                    if all_world_points_r is not None and all_world_points_l is not None:
                         
                        mask_valid = ~np.isnan(all_world_points_r).any(axis=1)
                        all_world_points_r = all_world_points_r[mask_valid]
                        color_r = color_r[mask_valid]

                        print(f"World point in right camera: {all_world_points_r}, in left camera: {all_world_points_l}")

                        mask_valid = ~np.isnan(all_world_points_l).any(axis=1)
                        all_world_points_l = all_world_points_l[mask_valid]
                        color_l = color_l[mask_valid]

                        # If both sides have point clouds, use ICP to align and merge
                        all_points, all_colors, T = filter_and_merge_icp_translation_only(
                            all_world_points_l, color_l, all_world_points_r, color_r
                        )           

                        # # Example of merging point clouds
                        # all_points = list(all_world_points_r) + list(all_world_points_l)
                        # all_colors = list(color_r) + list(color_l)

                        center_world_points = (center_world_points_l + center_world_points_r)/2
                        success = True
                        
                    elif all_world_points_r is not None and all_world_points_l is None:

                        mask_valid = ~np.isnan(all_world_points_r).any(axis=1)
                        all_world_points_r = all_world_points_r[mask_valid]
                        color_r = color_r[mask_valid]

                        print(f"World point in right camera: {all_world_points_r}, in left camera: None")

                        # If only the right side has point clouds, use the right side's point clouds directly
                        pcd_r = preprocess_pointcloud(all_world_points_r, color_r, voxel_size=0.005, nb_points=10, radius=0.02)
                        all_world_points_r = np.asarray(pcd_r.points)
                        color_r = np.asarray(pcd_r.colors)

                        all_points = list(all_world_points_r)
                        all_colors = list(color_r)
                        center_world_points = center_world_points_r
                        success = True
                        
                    elif all_world_points_r is None and all_world_points_l is not None:
                        
                        mask_valid = ~np.isnan(all_world_points_l).any(axis=1)
                        all_world_points_l = all_world_points_l[mask_valid]
                        color_l = color_l[mask_valid]

                        print(f"World point in right camera: None, in left camera: {all_world_points_l}")

                        # If only the left side has point clouds, use the left side's point clouds directly
                        pcd_l = preprocess_pointcloud(all_world_points_l, color_l, voxel_size=0.005, nb_points=10, radius=0.02)
                        all_world_points_l = np.asarray(pcd_l.points)
                        color_l = np.asarray(pcd_l.colors)

                        all_points = list(all_world_points_l)
                        all_colors = list(color_l)
                        center_world_points = center_world_points_l
                        success = True
                    else:
                        print("❌ Failed to perceive target points in both cameras.")
                        if response_r:
                            play_text_to_speech(response_r, language='en')
                        else:
                            play_text_to_speech("Sorry, I can't find that. Please try again.", language='en')

                        success = False
                        continue


                    # Calculate centroid
                    all_points_arr = np.array(all_points)
                    all_colors_arr = np.array(all_colors)
                    
                    
                    mask_valid = ~np.isnan(all_points_arr).any(axis=1)
                    all_points_arr = all_points_arr[mask_valid]
                    all_colors_arr = all_colors_arr[mask_valid]
                    if len(all_points_arr) == 0:
                        print("❌ All points are invalid (contain NaNs)")
                        success = False
                    


                    # Calculate center point
                    center_world_points = center_world_points[0]
                    print ("center_world_points:", center_world_points)

                    # Calculate centroid
                    target_center_point = all_points_arr.mean(axis=0)
                    print("Centroid:", target_center_point)

                    # Calculate highest point in z-axis
                    max_z_index = np.argmax(all_points_arr[:,2])
                    target_max_z_point = all_points_arr[max_z_index]
                    print("Highest point in z-axis:", target_max_z_point)

                    # Calculate move_target_point
                    move_target_point = target_center_point.copy()
                    move_target_point[2] += 0.4  # Raise by 0.4 meters

                    print("Move target point:", move_target_point)

                    move_params["move_x"] = float(move_target_point[0])
                    move_params["move_y"] = float(move_target_point[1])
                    move_params["move_z"] = float(move_target_point[2])
                    move_params["move_qx"] = 1.0
                    move_params["move_qy"] = 0.0
                    move_params["move_qz"] = 0.0
                    move_params["move_qw"] = 0.0
                    
                    open3d_show(all_points_arr, all_colors_arr, target_center_point, target_max_z_point, center_world_points)
                    # move_params = {"move_x" : 0.5, "move_y" : 0.6, "move_z" : 0.4, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
                    # success = True
                
            elif act_type == "move":
                success = False  # Reset success status
                if target is  grasped_thing:
                    continue  
                current_position = [0.0, 0.0, 0.0]  # Initialize current position

                current_position[0], current_position[1], current_position[2] = call_fk_service()
                print(f"Current position: {current_position}")

                if current_position[2] < 0.3:   
                    try:
                        if any(v is None for v in move_params.values()):
                            raise ValueError("Missing move parameters.")
                        success = call_ros2_service(
                            "/move_cartesian_service",
                            "action_interfaces/srv/Move",
                            {
                                "x": float(current_position[0]),
                                "y": float(current_position[1]),
                                "z": float(0.5),  # Raise to 0.5 meters
                                "qx": 1.0,
                                "qy": 0.0,
                                "qz": 0.0,
                                "qw": 0.0,
                            }
                        )
                    except ValueError as e:
                        print(e)
                    # time.sleep(3)  # Wait for service call to complete
                    while True:
                        if success:
                            print("✅ List action executed successfully.")
                            break
                        else:
                            print("❌ List action failed, retrying...")
                
                
                
                if move_params["move_z"] < 0.35:   
                    try:
                        if any(v is None for v in move_params.values()):
                            raise ValueError("Missing move parameters.")
                        success = call_ros2_service(
                            "/move_cartesian_service",
                            "action_interfaces/srv/Move",
                            {
                                "x": move_params["move_x"],
                                "y": move_params["move_y"],
                                "z":                    0.5,# Raise to 0.5 meters
                                "qx": move_params["move_qx"],
                                "qy": move_params["move_qy"],
                                "qz": move_params["move_qz"],
                                "qw": move_params["move_qw"],
                            }
                        )
                    except ValueError as e:
                        print(e)
                    # time.sleep(3)  # Wait for service call to complete
                    while True:
                        if success:
                            print("✅ List action executed successfully.")
                            break
                        else:
                            print("❌ List action failed, retrying...")
                
                try:
                    if any(v is None for v in move_params.values()):
                        raise ValueError("Missing move parameters.")
                    success = call_ros2_service(
                        "/move_cartesian_service",
                        "action_interfaces/srv/Move",
                        {
                            "x": move_params["move_x"],
                            "y": move_params["move_y"],
                            "z": move_params["move_z"],
                            "qx": move_params["move_qx"],
                            "qy": move_params["move_qy"],
                            "qz": move_params["move_qz"],
                            "qw": move_params["move_qw"],
                        }
                    )
                except ValueError as e:
                    print(e)
                # time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Move action executed successfully.")
                        break
                    # else:
                    #     print("❌ Move action failed, retrying...")

            # elif act_type == "grasp_flavoring":
            #     success = False  # Reset success status
            #     if target is  grasped_thing:
            #         continue 
                
            #     print("execute grasp action")
 
                
            #     if target == "spoon": 
            #         grasp_params = {"x_prep": 0.5, "y_prep": 0.6, "z_prep": 0.3, "qx_prep": 1.0, "qy_prep": 0.0, "qz_prep": 0.0, "qw_prep": 0.0,
            #                     "x_grasp": 0.5, "y_grasp": 0.6, "z_grasp": 0.2, "qx_grasp": 1.0, "qy_grasp": 0.0, "qz_grasp": 0.0, "qw_grasp": 0.0}
            #     else:
 
            #         if all_points_arr is None:
            #             print("❌ Failed to perceive target points in both cameras.")

            #         else:

            #             obb_corners, rotation_matrix, center = compute_obb(all_points_arr)

            #             visualize_obb_and_center(all_points_arr, all_colors_arr, obb_corners, center)

            #             grasp_generator = GraspGeneration(center, rotation_matrix)
            #             pose1_pos, pose1_orn, pose2_pos, pose2_orn = grasp_generator.final_compute_poses(all_points_arr, all_colors_arr, visualize=True, grasp_type='flavoring')    
            #             print("Pose 1 - Position:", pose1_pos, "Orientation:", pose1_orn)
            #             print("Pose 2 - Position:", pose2_pos, "Orientation:", pose2_orn)
                        
            #             if pose1_pos is None or pose1_orn is None or pose2_pos is None or pose2_orn is None:
            #                 print("❌ Failed to compute grasp poses from OBB.")
            #                 success = False
            #                 continue

            #             grasp_params = {
            #                 "x_prep": float(pose1_pos[0]), "y_prep": float(pose1_pos[1]), "z_prep": float(pose1_pos[2]),
            #                 "qx_prep": float(pose1_orn[0]), "qy_prep": float(pose1_orn[1]), "qz_prep": float(pose1_orn[2]), "qw_prep": float(pose1_orn[3]),
            #                 "x_grasp": float(pose2_pos[0]), "y_grasp": float(pose2_pos[1]), "z_grasp": float(pose2_pos[2]),
            #                 "qx_grasp": float(pose2_orn[0]), "qy_grasp": float(pose2_orn[1]), "qz_grasp": float(pose2_orn[2]), "qw_grasp": float(pose2_orn[3])
            #             }
            #             print("Grasp parameters computed from OBB:", grasp_params) 
                

            #     try:
            #         # Check all values
            #         if any(v is None for v in grasp_params.values()):
            #             raise ValueError("Missing grasp parameters.")
            #         success = call_ros2_service(
            #             "/grasp_service",
            #             "action_interfaces/srv/Grasp",
            #             {
            #                 "x_prep":   grasp_params["x_prep"],
            #                 "y_prep":   grasp_params["y_prep"],
            #                 "z_prep":   grasp_params["z_prep"],
            #                 "qx_prep":  grasp_params["qx_prep"],
            #                 "qy_prep":  grasp_params["qy_prep"],
            #                 "qz_prep":  grasp_params["qz_prep"],
            #                 "qw_prep":  grasp_params["qw_prep"],
            #                 "x_grasp":  grasp_params["x_grasp"],
            #                 "y_grasp":  grasp_params["y_grasp"],
            #                 "z_grasp":  grasp_params["z_grasp"],
            #                 "qx_grasp": grasp_params["qx_grasp"],
            #                 "qy_grasp": grasp_params["qy_grasp"],
            #                 "qz_grasp": grasp_params["qz_grasp"],
            #                 "qw_grasp": grasp_params["qw_grasp"],
            #             }
            #         )
            #     except ValueError as e:
            #         print(e)
            #     while True:
            #         if success:
            #             print("✅ Grasp flavoring action executed successfully.")
            #             grasped_thing = target
            #             break
            #         # else:
            #         #     print("❌ Grasp flavoring action failed, retrying...")

            elif act_type == "grasp_otherthings":
                success = False  # Reset success status
                if target is grasped_thing:
                    continue
                print("Execute grasp action")

                # put the value into go_params by grasp strategy with points cloud or something else
                if target == "spoon": 
                    grasp_params = {"x_prep": 0.406, "y_prep": -0.313, "z_prep": 0.57, "qx_prep": 0.999, "qy_prep": 0.023, "qz_prep": 0.026, "qw_prep": 0.001,
                            "x_grasp": 0.406, "y_grasp": -0.313, "z_grasp": 0.3, "qx_grasp": 0.999, "qy_grasp": 0.023, "qz_grasp": 0.026, "qw_grasp": 0.001}
                elif target == "pepper bottle":
                    grasp_params = {"x_prep": 0.27, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
                                "x_grasp": 0.27, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}
                elif target == "salt bottle":
                    grasp_params = {"x_prep": 0.434, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
                                "x_grasp": 0.434, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}

                else:
                    if all_points_arr is None:
                        print("❌ Failed to perceive target points in both cameras.")
                    #     maybe other method to determine the grasp strategy
                    else:
                    #     ................
                    #     now we have world_points of target (all_points_arr) , which is a array , each element is a point in world coordinates
                    #     put the value into gf_params by grasp strategy with points cloud or something else   
                        obb_corners, rotation_matrix, center = compute_obb(all_points_arr)

                        visualize_obb_and_center(all_points_arr, all_colors_arr, obb_corners, center)

                        grasp_generator = GraspGeneration(center, rotation_matrix)
                        pose1_pos, pose1_orn, pose2_pos, pose2_orn = grasp_generator.final_compute_poses(all_points_arr, all_colors_arr, visualize=True, grasp_type='otherthings')    
                        # print("Pose 1 - Position:", pose1_pos, "Orientation:", pose1_orn)
                        # print("Pose 2 - Position:", pose2_pos, "Orientation:", pose2_orn)
                        
                        if pose1_pos is None or pose1_orn is None or pose2_pos is None or pose2_orn is None:
                            print("❌ Failed to compute grasp poses from OBB.")
                            success = False
                            continue

                        grasp_params = {
                            "x_prep": float(pose1_pos[0]), "y_prep": float(pose1_pos[1]), "z_prep": float(pose1_pos[2]),
                            "qx_prep": float(pose1_orn[0]), "qy_prep": float(pose1_orn[1]), "qz_prep": float(pose1_orn[2]), "qw_prep": float(pose1_orn[3]),
                            "x_grasp": float(pose2_pos[0]), "y_grasp": float(pose2_pos[1]), "z_grasp": float(pose2_pos[2]),
                            "qx_grasp": float(pose2_orn[0]), "qy_grasp": float(pose2_orn[1]), "qz_grasp": float(pose2_orn[2]), "qw_grasp": float(pose2_orn[3])
                        }
                        print("Grasp parameters computed from OBB:", grasp_params)      
                
                try:
                    # Check all grasp_params values
                    if any(v is None for v in grasp_params.values()):
                        raise ValueError("Missing grasp parameters in go_params.")
                    success = call_ros2_service(
                        "/grasp_service",
                        "action_interfaces/srv/Grasp",
                        {
                            "x_prep":   grasp_params["x_prep"],
                            "y_prep":   grasp_params["y_prep"],
                            "z_prep":   grasp_params["z_prep"],
                            "qx_prep":  grasp_params["qx_prep"],
                            "qy_prep":  grasp_params["qy_prep"],
                            "qz_prep":  grasp_params["qz_prep"],
                            "qw_prep":  grasp_params["qw_prep"],
                            "x_grasp":  grasp_params["x_grasp"],
                            "y_grasp":  grasp_params["y_grasp"],
                            "z_grasp":  grasp_params["z_grasp"],
                            "qx_grasp": grasp_params["qx_grasp"],
                            "qy_grasp": grasp_params["qy_grasp"],
                            "qz_grasp": grasp_params["qz_grasp"],
                            "qw_grasp": grasp_params["qw_grasp"],
                        }
                    )
                except ValueError as e:
                    print(e)
                    
                    
                # time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Grasp other things action executed successfully.")
                        grasped_thing = target
                        break
                    else:
                        print("❌ Grasp other things action failed, retrying...")
                            
            elif act_type == "stir":
                success = False  # Reset success status
                print("Execute stir action")
                # time.sleep(5)
                # success = True
                try:
                    if stir_time is None:
                        raise ValueError("Missing stir time.")
                    success = call_ros2_service("/stir_service", "action_interfaces/srv/Stir", {
                        "center_x": 0.62,
                        "center_y": -0.37,
                        "center_z": 0.42,
                        "radius": 0.05,
                        "start_angle_deg": 0.0,
                        "move_down_offset": 0.1,
                        "speed": 0.5,
                        "stir_time": stir_time
                    })
                except ValueError as e:
                    print(e)
                time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Stir action executed successfully.")
                        break
                    else:
                        print("❌ Stir action failed, retrying...")     
                
            elif act_type == "reset":
                success = False  # Reset success status
                print("Execute reset action")
                # time.sleep(5)
                # success = True
                success = call_ros2_service("/reset_service", "std_srvs/srv/Trigger", {})

                time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Reset action executed successfully.")
                        break
                    else:
                        print("❌ Reset action failed, retrying...")

            elif act_type == "add":
                success = False  # Reset success status
                print("Execute add action")
                # time.sleep(5)
                # success = True
                
                try:
                    if add_times is None:
                        raise ValueError("Missing add times.")
                    success = call_ros2_service("/add_service", "action_interfaces/srv/Add", 
                        {"times": add_times})
                except ValueError as e:
                    print(e)

                time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Add action executed successfully.")
                        break
                    else:
                        print("❌ Add action failed, retrying...")
                
            elif act_type == "return_back":
                success = False  # Reset success status
                print("Execute back_move action, moving back to original point")
                # time.sleep(5)
                # success = True
                print("=============================================================================================grasp_params:", grasp_params)
                
                if any(v is not None for v in grasp_params.values()):
                    rb_params["x_prep"] = grasp_params["x_prep"] 
                    rb_params["y_prep"] = grasp_params["y_prep"]
                    rb_params["z_prep"] = grasp_params["z_prep"]
                    rb_params["qx_prep"] = grasp_params["qx_prep"]
                    rb_params["qy_prep"] = grasp_params["qy_prep"]
                    rb_params["qz_prep"] = grasp_params["qz_prep"]
                    rb_params["qw_prep"] = grasp_params["qw_prep"]
                    rb_params["x_place"] = grasp_params["x_grasp"]
                    rb_params["y_place"] = grasp_params["y_grasp"]
                    rb_params["z_place"] = grasp_params["z_grasp"]
                    rb_params["qx_place"] = grasp_params["qx_grasp"]
                    rb_params["qy_place"] = grasp_params["qy_grasp"]
                    rb_params["qz_place"] = grasp_params["qz_grasp"]
                    rb_params["qw_place"] = grasp_params["qw_grasp"]
                else:
                    print("No return_back parameters provided, please check! ")
                    
                

                try:
                    # Check if there are any unassigned parameters in rb_params
                    if any(v is None for v in rb_params.values()):
                        raise ValueError("Missing return_back parameters in rb_params.")
                        
                    else:
                        success = call_ros2_service(
                            "/return_back_service",
                            "action_interfaces/srv/ReturnBack",
                            {
                                "x_prep":   rb_params["x_prep"],
                                "y_prep":   rb_params["y_prep"],
                                "z_prep":   rb_params["z_prep"],
                                "qx_prep":  rb_params["qx_prep"],
                                "qy_prep":  rb_params["qy_prep"],
                                "qz_prep":  rb_params["qz_prep"],
                                "qw_prep":  rb_params["qw_prep"],
                                "x_place":  rb_params["x_place"],
                                "y_place":  rb_params["y_place"],
                                "z_place":  rb_params["z_place"],
                                "qx_place": rb_params["qx_place"],
                                "qy_place": rb_params["qy_place"],
                                "qz_place": rb_params["qz_place"],
                                "qw_place": rb_params["qw_place"],
                            }
                        )
                except ValueError as e:
                    print(e)
                time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Return back action executed successfully.")
                        break
                    else:
                        print("❌ Return back action failed, retrying...")
                        
            elif act_type == "open":
                success = False  # Reset success status
                print("Execute open action")
                success = call_ros2_service("/open_service", "std_srvs/srv/Trigger", {})
                time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Open action executed successfully.")
                        break
                    else:
                        print("❌ Open action failed, retrying...")

            elif act_type == "close":
                success = False  # Reset success status
                print("Execute close action")
                success = call_ros2_service("/close_service", "std_srvs/srv/Trigger", {})
                time.sleep(3)  # Wait for service call to complete
                while True:
                    if success:
                        print("✅ Close action executed successfully.")
                        break
                    else:
                        print("❌ Close action failed, retrying...")
            
            else:
                print(f"⚠️ Unknown action type: {act_type}")
                success = False

            time.sleep(0.5)  # Optional delay

        except Exception as e:
            print("❌ Exception inside execute_action_sequence:")
            traceback.print_exc()
            raise
        print("✅ Action sequence completed.")

# ✅ Test case: Manually construct action sequence
if __name__ == "__main__":
    actions = [
        {"type": "perceive", "target": "tomato", "parameters": {}},
        {"type": "move", "target": "tomato","parameters": {}},
        {"type": "grasp_otherthings", "target": "tomato", "parameters": {}},
        {"type": "perceive", "target": "user person", "parameters": {}},
        {"type": "move", "target": "user person", "parameters": {}},
        {"type": "open", "target": "user person", "parameters": {}},
        {"type": "reset", "target": "home", "parameters": {}}
    ]

    execute_action_sequence(actions)



# # Example your_msgs/srv/Move.srv
# string target
# ---
# bool success
# string message

# Must return:
# return Move.Response(success=True, message="Moved to apple.")