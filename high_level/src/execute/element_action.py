
import time
import numpy as np
from src.pixel_world.pixel_and_world import pixels_to_world_left, pixels_to_world_right
from src.transcribe.tts import play_text_to_speech
from src.grasp.bounding_box import compute_obb
from src.grasp.grasp_generation import GraspGeneration
from src.execute.utils import *



def action_perceive(target, grasped_thing, vlm_client, l_img_path, l_depth_path, r_img_path, r_depth_path):
    """Perceive the target object using VLM_agent and point cloud processing.

    Args:
        target (str): The target object to perceive.
        grasped_thing (str): The currently grasped object, if any.
        vlm_client: The VLM client instance of Mistralmodel() for perception.
        l_img_path (str): Path to the left RGB image.
        l_depth_path (str): Path to the left depth image (.npy).
        r_img_path (str): Path to the right RGB image.
        r_depth_path (str): Path to the right depth image (.npy).
            
    Returns: 
        move_params: dict with keys "move_x", "move_y", "move_z", "move_qx", "move_qy", "move_qz", "move_qw"
        all_points_arr: ndarray of shape (N, 3), perceived point cloud in world coordinates
        all_colors_arr: ndarray of shape (N, 3), colors corresponding to the point cloud
        target_center_point: ndarray of shape (3,), centroid of the perceived object
        target_max_z_point: ndarray of shape (3,), point with the highest z-coordinate
        center_world_points: ndarray of shape (3,), perceived target point in world coordinates
        success: bool, whether perception was successful
    
    Raises:
        ValueError: If perception fails or required data is missing.
    """
    move_params = {"move_x" : None, "move_y" : None, "move_z" : None, "move_qx" : None, "move_qy" : None, "move_qz" : None, "move_qw" : None}
    success = False  # Reset success status
    if target ==  grasped_thing:
        success = True
        print(f"✅ Target '{target}' already grasped, skipping perception.")
        return move_params, None, None, None, None, None, success        
    elif target == "user person":
        move_params = {"move_x" : 0.718029693832728, "move_y" : -0.07702313108387482, "move_z" : 0.4, "move_qx" : 0.707, "move_qy" : -0.707, "move_qz" : 0.0, "move_qw" : 0.0}
        success = True
        print("✅ Perceived user person, moving to target point.")
        return move_params, None, None, None, None, None, success
    elif target == "spoon":
        move_params = {"move_x" : 0.406, "move_y" : -0.313, "move_z" : 0.6, "move_qx" : 0.999, "move_qy" : 0.023, "move_qz" : 0.026, "move_qw" : 0.001}
        success = True
        print("✅ Perceived spoon, moving to target point.")
        return move_params, None, None, None, None, None, success
    elif target == "soup pot":
        move_params = {"move_x" : 0.6, "move_y" : -0.3, "move_z" : 0.32, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
        success = True
        print("✅ Perceived soup pot, moving to target point.")
        return move_params, None, None, None, None, None, success
    elif target == "salt bottle":
        move_params = {"move_x" : 0.434, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
        success = True
        print("✅ Perceived salt bottle, moving to target point.")
        return move_params, None, None, None, None, None, success
    elif target == "pepper bottle":
        move_params = {"move_x" : 0.27, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
        success = True
        print("✅ Perceived pepper bottle, moving to target point.")
        return move_params, None, None, None, None, None, success

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
            return None, None, None, None, None, None, success


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
        return move_params, all_points_arr, all_colors_arr, target_center_point, target_max_z_point, center_world_points, success

def action_move(move_params, target, grasped_thing):
    """Move the robot to a specified target.

    Args:
        move_params: dict with keys "move_x", "move_y", "move_z", "move_qx", "move_qy", "move_qz", "move_qw"
        target: string, the target to move to
        grasped_thing: string, the currently grasped object
            
    Returns:
        success: bool, whether the move was successful
    
    Raises:
        ValueError: If move_params are not provided or incomplete.
    """
    success = False  # Reset success status
    if target is grasped_thing:
        success = True
        print(f"✅ Target '{target}' already grasped, skipping move.")
        return success
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
    return success

def action_grasp(target, all_points_arr, all_colors_arr, grasped_thing, grasp_params):
    """Grasp an object using predefined parameters.
        Objects including spoon, salt bottle, pepper bottle have been hardcoded with grasp parameters.

    Args:
        target: string, the target to grasp
        all_points_arr: ndarray, Nx3, point cloud of the target object
        all_colors_arr: ndarray, Nx3, colors corresponding to the point cloud
        grasped_thing: string, the currently grasped object, if any
        grasp_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
                        "x_grasp", "y_grasp", "z_grasp", "qx_grasp", "qy_grasp", "qz_grasp", "qw_grasp"
            
    Returns:
        grasped_thing: string, the currently grasped object
        grasp_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
                        "x_grasp", "y_grasp", "z_grasp", "qx_grasp", "qy_grasp", "qz_grasp", "qw_grasp"
        success: bool, whether the grasp was successful
    
    Raises:
        ValueError: If any grasp parameters are missing.
    """
    success = False  # Reset success status
    if target is grasped_thing:
        success = True
        print(f"✅ Target '{target}' already grasped, skipping grasp.")
        return grasped_thing, grasp_params, success
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
            success = False
            return grasped_thing, grasp_params, success
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
                return grasped_thing, grasp_params, success

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
    
    return grasped_thing, grasp_params, success
    
def action_stir(stir_time):
    """Stir a pot for a specified duration.

    Args:
        stir_time: in seconds
    
    Returns:
        success: bool, whether the stir was successful
    
    Raises:
        ValueError: If stir_time is not provided.
    """
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
    return success

def action_reset():
    """Reset the robot to its home position.

    Args:
        None
            
    Returns:
        success: bool, whether the reset was successful
    
    Raises:
        None
    """
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
    return success

def action_add(add_times):
    """Add an ingredient a specified number of times.

    Args:
        add_times: integer, number of times to add the ingredient

    Returns:
        success: bool, whether the add was successful

    Raises:
        ValueError: If add_times is not provided.
    """
    success = False  # Reset success status
    print("Execute add action")
    
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
    return success

def action_return_back(grasp_params):
    """Return the robot to its original position after placing an object.

    Args:
        grasp_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
                      "x_grasp", "y_grasp", "z_grasp", "qx_grasp", "qy_grasp", "qz_grasp", "qw_grasp"

    Returns:
        grasped_thing: string, always "" because the robot has placed the object down
        rb_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
                   "x_place", "y_place", "z_place", "qx_place", "qy_place", "qz_place", "qw_place"
        success: bool, whether the return back was successful

    Raises:
        ValueError: If rb_params are not provided or incomplete.
    """
    rb_params = {
        "x_prep": None, "y_prep": None, "z_prep": None,
        "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
        "x_place": None, "y_place": None, "z_place": None,
        "qx_place": None, "qy_place": None, "qz_place": None, "qw_place": None
    }
    grasped_thing = ""
    success = False  # Reset success status
    print("Execute back_move action, moving back to original point")
    # time.sleep(5)
    # success = True
    
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
            grasped_thing = ""  # Reset grasped thing after returning back
            break
        else:
            print("❌ Return back action failed, retrying...")
    return grasped_thing, rb_params, success

def action_open():
    """Open the robot's gripper.

    Args:
        None
            
    Returns:
        success: bool, whether the open was successful
    
    Raises:
        None
    """
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
    return success

def action_close():
    """Close the robot's gripper.
    
    Args:
        None
    
    Returns:
        success: bool, whether the close was successful
    
    Raises:
        None
    """
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
    return success


