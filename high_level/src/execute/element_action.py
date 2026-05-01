import time
import os
import traceback
import numpy as np
from src.pixel_world.pixel_and_world import pixels_to_world_left, pixels_to_world_realsense, pixels_to_world_right
from src.transcribe.tts import play_text_to_speech
from src.grasp.bounding_box import compute_obb
from src.grasp.grasp_generation import GraspGeneration
from src.execute.utils import *
from src.VLM_agent.OwlViT_FastSAM_SAM import TextDrivenSegmenter


class ActionExecutor:
    """
    A class that encapsulates all robot action execution functionality.
    Parameters are stored as class variables for easy access across methods.
    """
    
    def __init__(self):
        """Initialize the ActionExecutor with all necessary parameters as class variables."""
        self.segmenter = TextDrivenSegmenter() 
        # Image paths
        CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        self.IMAGE_FOLDER_PATH = os.path.abspath(os.path.join(
            CURRENT_DIR, "../../../manipulation_ws/saved_images"
        ))
        self.r_img_path = os.path.join(self.IMAGE_FOLDER_PATH, "r_rgb.png")
        self.r_depth_path = os.path.join(self.IMAGE_FOLDER_PATH, "r_depth.npy")
        self.rs_img_path = os.path.join(self.IMAGE_FOLDER_PATH, "rs_rgb.png")
        self.rs_depth_path = os.path.join(self.IMAGE_FOLDER_PATH, "rs_depth.npy")

        # Action parameters as class variables
        self.move_target_point = None  # perceived target point
        self.target_max_z_point = None  # highest point in z-axis
        self.target_center_point = None  # centroid point
        self.center_world_points = None  # perceived image target point in world coordinates
        self.all_points_arr = None  # all points in world coordinates
        self.all_colors_arr = None  # all colors in world coordinates
        self.target = None  # current target, used to update the target in each action
        self.if_visualize = False  # whether to visualize the point cloud and grasp poses
        self.bbox_only = True  # whether to use bbox-only segmentation for perception
        # Move parameters
        self.move_params = {
            "move_x": None, "move_y": None, "move_z": None, 
            "move_qx": None, "move_qy": None, "move_qz": None, "move_qw": None
        }

        # Grasp parameters
        self.grasp_params = {
            "x_prep": None, "y_prep": None, "z_prep": None, 
            "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
            "x_grasp": None, "y_grasp": None, "z_grasp": None, 
            "qx_grasp": None, "qy_grasp": None, "qz_grasp": None, "qw_grasp": None
        }
        
        # Return back parameters
        self.rb_params = {
            "x_prep": None, "y_prep": None, "z_prep": None, 
            "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
            "x_place": None, "y_place": None, "z_place": None, 
            "qx_place": None, "qy_place": None, "qz_place": None, "qw_place": None
        }
        
        # Action timing parameters
        self.stir_time = 30
        self.add_times = 2
        
        # Z-axis height threshold to filter out table/background points
        self.z_filter_threshold = 0.01  # points with z below this value are removed

        # Action move offset
        self.delta = 0.0
        self.direction = None

        # State flags
        self.grasped_thing = ""
        self.success = True

    def execute_action_sequence(self, actions, vlm_client):
        """
        Execute a sequence of actions serially, waiting for each service to complete successfully before proceeding to the next.
        """
        
        # Reset class variables for new action sequence
        self.success = True

        for i, action in enumerate(actions):
            # Check if the service was successful
            if not self.success:
                print(f"⛔ Aborting action sequence due to failure at step {i}.")
                play_text_to_speech('Sorry, I cannot do that. Please help me.', language='en')
                self.action_open()
                self.action_reset()
                break

            try:
                print(f"\n▶️ Executing action {i+1}/{len(actions)}: {action}")
                act_type = action["type"]

                if action["target"] is not None:
                    self.target = action["target"]
                    print(f"🔍 Target: {self.target}")
                params = action.get("parameters", {})

                if "stir_time" in params:
                    if params["stir_time"] is not None:
                        self.stir_time = params["stir_time"]
                        print(f"🔧 Parameters: {params}")
                
                if "add_times" in params:
                    if params["add_times"] is not None:
                        self.add_times = params["add_times"]
                        print(f"🔧 Add times: {self.add_times}")

                if "delta" in params:
                    if params["delta"] is not None:
                        self.delta = params["delta"]
                        print(f"🔧 Delta: {self.delta}")

                if "direction" in params:
                    if params["direction"] is not None:
                        self.direction = params["direction"]
                        print(f"🔧 Direction: {self.direction}")

                if act_type == "perceive":
                    self.action_perceive(vlm_client)
                    
                elif act_type == "move":
                    self.action_move()

                elif act_type == "grasp_otherthings":
                    self.action_grasp()
                                
                elif act_type == "stir":
                    self.action_stir()
                    
                elif act_type == "reset":
                    self.action_reset()

                elif act_type == "add":
                    self.action_add()

                elif act_type == "return_back":
                    self.action_return_back()
                            
                elif act_type == "open":
                    self.action_open()

                elif act_type == "close":
                    self.action_close()

                elif act_type == "grasp_detection":
                    self.action_grasp_detection()

                elif act_type == "get_grasps":
                    self.get_grasps()   

                elif act_type == "move_offset":
                    self.action_move_offset()

                elif act_type == "get_objects_pos":
                    self.get_object_pos(vlm_client, self.target)
                else:
                    print(f"⚠️ Unknown action type: {act_type}")
                    self.success = False

                # time.sleep(0.5)  # Optional delay

            except Exception as e:
                print("❌ Exception inside execute_action_sequence:")
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()
                traceback.print_exc()
                raise
        print("✅ Action sequence completed.")

    def action_perceive(self, vlm_client):
        """Perceive the target object using VLM_agent and point cloud processing.
        Uses class variables for all parameters and updates them accordingly.
        
        Args:
            vlm_client: The VLM client instance of Mistralmodel() for perception.
        
        Updates class variables:
            self.move_params, self.all_points_arr, self.all_colors_arr, 
            self.target_center_point, self.target_max_z_point, 
            self.center_world_points, self.success
        
        Raises:
            ValueError: If perception fails or required data is missing.
        """
        self.success = False  # Reset success status
        
        if self.target == self.grasped_thing:
            self.success = True
            print(f"✅ Target '{self.target}' already grasped, skipping perception.")
            return        
        elif self.target == "user person":
            self.move_params = {"move_x" : 0.718029693832728, "move_y" : -0.07702313108387482, "move_z" : 0.4, "move_qx" : 0.707, "move_qy" : -0.707, "move_qz" : 0.0, "move_qw" : 0.0}
            self.success = True
            print("✅ Perceived user person, moving to target point.")
            return
        elif self.target == "spoon":
            self.move_params = {"move_x" : 0.52, "move_y" : -0.14, "move_z" : 0.6, "move_qx" : 0.999, "move_qy" : 0.023, "move_qz" : 0.026, "move_qw" : 0.001}
            # "move_x" : 0.406, "move_y" : -0.313, "move_z" : 0.6
            self.success = True
            print("✅ Perceived spoon, moving to target point.")
            return
        elif self.target == "soup pot":
            self.move_params = {"move_x" : 0.62, "move_y" : -0.33, "move_z" : 0.37, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
            self.success = True
            print("✅ Perceived soup pot, moving to target point.")
            return
        elif self.target == "salt bottle":
            self.move_params = {"move_x" : 0.434, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
            self.success = True
            print("✅ Perceived salt bottle, moving to target point.")
            return
        elif self.target == "pepper bottle":
            self.move_params = {"move_x" : 0.27, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
            self.success = True
            print("✅ Perceived pepper bottle, moving to target point.")
            return
        elif self.target == "dessert plate":
            self.move_params = {"move_x" : 0.426, "move_y" : -0.353, "move_z" : 0.32, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
            self.success = True
            print("✅ Perceived dessert plate, moving to target point.")
            return

        else:  
            print(f"Perceiving target: {self.target}")
            
            all_world_points_rs = None
            all_world_points_r = None
            
            if self.target == "cucumber" or self.target == "banana" or self.target == "tomato" or self.target == "sponge" or self.target == "juice":

                if_find_r, response_r, center_world_points_r, all_world_points_r, color_r  = get_cam_world_points(
                    vlm_client,
                    self.target,
                    rgb_path= self.r_img_path,
                    depth_path= self.r_depth_path,
                    pixels_to_world_func = pixels_to_world_right,
                    name= "right",
                    segmenter=self.segmenter,
                    bbox_only= self.bbox_only
                )
            else:
                if_find_rs, response_rs, center_world_points_rs, all_world_points_rs ,color_rs = get_cam_world_points(
                    vlm_client,
                    self.target,
                    rgb_path= self.rs_img_path,
                    depth_path= self.rs_depth_path,
                    pixels_to_world_func = pixels_to_world_realsense,
                    name= "realsense",
                    segmenter=self.segmenter,
                    bbox_only= self.bbox_only
                )

        
            z_filter_threshold = self.z_filter_threshold

            if all_world_points_r is not None and all_world_points_rs is not None: 
                mask_valid = ~np.isnan(all_world_points_r).any(axis=1)
                all_world_points_r = all_world_points_r[mask_valid]
                color_r = color_r[mask_valid]

                mask_valid = ~np.isnan(all_world_points_rs).any(axis=1)
                all_world_points_rs = all_world_points_rs[mask_valid]
                color_rs = color_rs[mask_valid]
                
                
                all_world_points_r = np.asarray(all_world_points_r)
                all_world_points_rs = np.asarray(all_world_points_rs)
                color_r = np.asarray(color_r) if color_r is not None else None
                color_rs = np.asarray(color_rs) if color_rs is not None else None

                all_world_points_r, color_r = filter_out_pcd_by_z(
                    all_world_points_r, color_r, z_filter_threshold
                )
                all_world_points_rs, color_rs = filter_out_pcd_by_z(
                    all_world_points_rs, color_rs, z_filter_threshold
                )
                if self.if_visualize:   
                    open3d_show(all_world_points_rs, color_rs, center_world_points_rs, all_world_points_r, color_r, center_world_points_r)
                print(f"World point in right camera: {all_world_points_r}, in realsense camera: {all_world_points_rs}")
                
                # If both sides have point clouds, use ICP to align and merge
                all_points, all_colors, T = filter_and_merge_icp_translation_only(
                    all_world_points_rs, color_rs, all_world_points_r, color_r
                )           

                self.center_world_points = (center_world_points_rs + center_world_points_r)/2
                self.success = True
                
            elif all_world_points_r is not None and all_world_points_rs is None:

                mask_valid = ~np.isnan(all_world_points_r).any(axis=1)
                all_world_points_r = all_world_points_r[mask_valid]
                color_r = color_r[mask_valid]
                
                all_world_points_r = np.asarray(all_world_points_r)
                color_r = np.asarray(color_r) if color_r is not None else None

                all_world_points_r, color_r = filter_out_pcd_by_z(
                    all_world_points_r, color_r, z_filter_threshold
                )
                if self.if_visualize:
                    open3d_show(all_world_points_r, color_r, center_world_points_r)
                print(f"World point in right camera: {all_world_points_r}, in left camera: None")

                # If only the right side has point clouds, use the right side's point clouds directly
                pcd_r = preprocess_pointcloud(all_world_points_r, color_r, voxel_size=0.005, nb_points=10, radius=0.02)
                all_world_points_r = np.asarray(pcd_r.points)
                color_r = np.asarray(pcd_r.colors)

                all_points = list(all_world_points_r)
                all_colors = list(color_r)
                self.center_world_points = center_world_points_r
                self.success = True
                
            elif all_world_points_r is None and all_world_points_rs is not None:
                mask_valid = ~np.isnan(all_world_points_rs).any(axis=1)
                all_world_points_rs = all_world_points_rs[mask_valid]
                color_rs = color_rs[mask_valid]
                
                all_world_points_rs = np.asarray(all_world_points_rs)
                color_rs = np.asarray(color_rs) if color_rs is not None else None

                all_world_points_rs, color_rs = filter_out_pcd_by_z(
                    all_world_points_rs, color_rs, z_filter_threshold
                )
                if self.if_visualize:
                    open3d_show(all_world_points_rs, color_rs, center_world_points_rs)
                print(f"World point in right camera: None, in realsense camera: {all_world_points_rs}")

                # If only the realsense side has point clouds, use the realsense side's point clouds directly
                pcd_rs = preprocess_pointcloud(all_world_points_rs, color_rs, voxel_size=0.005, nb_points=10, radius=0.02)
                all_world_points_rs = np.asarray(pcd_rs.points)
                color_rs = np.asarray(pcd_rs.colors)

                all_points = list(all_world_points_rs)
                all_colors = list(color_rs)
                self.center_world_points = center_world_points_rs
                self.success = True
            else:
                print("❌ Failed to perceive target points in both cameras.")
                if response_r:
                    play_text_to_speech(response_r, language='en')
                else:
                    play_text_to_speech("Sorry, I can't find that. Please try again.", language='en')
                self.action_open()
                self.action_reset()
                self.success = False
                return

            # Calculate centroid
            self.all_points_arr = np.array(all_points)
            self.all_colors_arr = np.array(all_colors)
            
            mask_valid = ~np.isnan(self.all_points_arr).any(axis=1)
            self.all_points_arr = self.all_points_arr[mask_valid]
            self.all_colors_arr = self.all_colors_arr[mask_valid]
            if len(self.all_points_arr) == 0:
                print("❌ All points are invalid (contain NaNs)")
                self.success = False
                self.action_open()
                self.action_reset()
                return

            # Calculate center point
            self.center_world_points = self.center_world_points[0]
            print ("center_world_points:", self.center_world_points)

            # Calculate centroid
            self.target_center_point = self.all_points_arr.mean(axis=0)
            print("Centroid:", self.target_center_point)

            # Calculate highest point in z-axis
            max_z_index = np.argmax(self.all_points_arr[:,2])
            self.target_max_z_point = self.all_points_arr[max_z_index]
            print("Highest point in z-axis:", self.target_max_z_point)

            # Calculate move_target_point
            self.move_target_point = self.target_center_point.copy()
            self.move_target_point[2] += 0.4  # Raise by 0.4 meters

            print("Move target point:", self.move_target_point)

            self.move_params["move_x"] = float(self.move_target_point[0])
            self.move_params["move_y"] = float(self.move_target_point[1])
            self.move_params["move_z"] = float(self.move_target_point[2])
            self.move_params["move_qx"] = 1.0
            self.move_params["move_qy"] = 0.0
            self.move_params["move_qz"] = 0.0
            self.move_params["move_qw"] = 0.0
            
            if self.if_visualize:
                open3d_show(self.all_points_arr, self.all_colors_arr, self.target_center_point, self.target_max_z_point, self.center_world_points)

    def action_move(self):
        """Move the robot to a specified target using class variables.
        Uses self.move_params, self.target, and self.grasped_thing.
        Updates self.success.
        
        Raises:
            ValueError: If move_params are not provided or incomplete.
        """
        self.success = False  # Reset success status
        if self.target == self.grasped_thing:
            self.success = True
            print(f"✅ Target '{self.target}' already grasped, skipping move.")
            return
        current_position = [0.0, 0.0, 0.0]  # Initialize current position

        try:
            current_position[0], current_position[1], current_position[2] = call_fk_service()
        except Exception as e:
            print(f"❌ Failed to get FK position: {e}")
            self.success = False
            play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
            self.action_open()
            self.action_reset()
            return

        print(f"Current position: {current_position}")

        if current_position[2] < 0.5:   
            try:
                if any(v is None for v in self.move_params.values()):
                    raise ValueError("Missing move parameters.")
                self.success = call_ros2_service(
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
            if self.success:
                print("✅ Lift-to-safe-height step succeeded.")
            else:
                print("❌ Lift-to-safe-height step failed.")
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()
                return
        
        if self.move_params["move_z"] < 0.5:   
            try:
                if any(v is None for v in self.move_params.values()):
                    raise ValueError("Missing move parameters.")
                self.success = call_ros2_service(
                    "/move_cartesian_service",
                    "action_interfaces/srv/Move",
                    {
                        "x": self.move_params["move_x"],
                        "y": self.move_params["move_y"],
                        "z": 0.5,# Raise to 0.5 meters
                        "qx": self.move_params["move_qx"],
                        "qy": self.move_params["move_qy"],
                        "qz": self.move_params["move_qz"],
                        "qw": self.move_params["move_qw"],
                    }
                )
            except ValueError as e:
                print(e)
            if self.success:
                print("✅ Pre-move safe-height step succeeded.")
            else:
                print("❌ Pre-move safe-height step failed.")
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()
                return
        
        try:
            if any(v is None for v in self.move_params.values()):
                raise ValueError("Missing move parameters.")
            self.success = call_ros2_service(
                "/move_cartesian_service",
                "action_interfaces/srv/Move",
                {
                    "x": self.move_params["move_x"],
                    "y": self.move_params["move_y"],
                    "z": self.move_params["move_z"],
                    "qx": self.move_params["move_qx"],
                    "qy": self.move_params["move_qy"],
                    "qz": self.move_params["move_qz"],
                    "qw": self.move_params["move_qw"],
                }
            )
        except ValueError as e:
            print(e)
        if self.success:
            print("✅ Move action executed successfully.")
        else:
            print("❌ Move action failed.")
            play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
            self.action_open()
            self.action_reset()

    def action_grasp(self):
        """Grasp an object using predefined parameters or computed grasp poses.
        Uses class variables for all parameters and updates them accordingly.
        Updates self.success and self.grasped_thing.
        
        Raises:
            ValueError: If any grasp parameters are missing.
        """
        self.success = False  # Reset success status
        if self.target is self.grasped_thing:
            self.success = True
            print(f"✅ Target '{self.target}' already grasped, skipping grasp.")
            return
        print("Execute grasp action")

        # put the value into go_params by grasp strategy with points cloud or something else
        if self.target == "spoon": 
            self.grasp_params = {"x_prep": 0.52, "y_prep": -0.14, "z_prep": 0.57, "qx_prep": 0.999, "qy_prep": 0.023, "qz_prep": 0.026, "qw_prep": 0.001,
                    "x_grasp": 0.52, "y_grasp": -0.14, "z_grasp": 0.37, "qx_grasp": 0.999, "qy_grasp": 0.023, "qz_grasp": 0.026, "qw_grasp": 0.001}
        elif self.target == "pepper bottle":
            self.grasp_params = {"x_prep": 0.27, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
                        "x_grasp": 0.27, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}
        elif self.target == "salt bottle":
            self.grasp_params = {"x_prep": 0.434, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
                        "x_grasp": 0.434, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}

        else:
            if self.all_points_arr is None:
                print("❌ Failed to perceive target points in both cameras.")
                self.success = False
                play_text_to_speech('Sorry, I cannot perceive target points in both cameras.', language='en')
                self.action_open()
                self.action_reset()
                return

            # use_anygrasp=True: AnyGrasp neural network method (no OBB needed)
            # use_anygrasp=False: original OBB + sampling method
            grasp_generator = GraspGeneration(use_anygrasp=False)

            if not grasp_generator.use_anygrasp:
                obb_corners, rotation_matrix, center = compute_obb(self.all_points_arr)
                # visualize_obb_and_center(self.all_points_arr, self.all_colors_arr, obb_corners, center)
                grasp_generator.bbox_center = center
                grasp_generator.bbox_rotation_matrix = rotation_matrix

            pose1_pos, pose1_orn, pose2_pos, pose2_orn, _, _ = grasp_generator.final_compute_poses(self.all_points_arr, self.all_colors_arr, visualize=self.if_visualize, grasp_type='otherthings')

            if pose1_pos is None or pose1_orn is None or pose2_pos is None or pose2_orn is None:
                play_text_to_speech('Sorry, I cannot find the suitable grasp poses.', language='en')
                print("❌ Failed to compute grasp poses.")
                self.action_open()
                self.action_reset()
                self.success = False
                return

            self.grasp_params = {
                "x_prep": float(pose1_pos[0]), "y_prep": float(pose1_pos[1]), "z_prep": float(pose1_pos[2]),
                "qx_prep": float(pose1_orn[0]), "qy_prep": float(pose1_orn[1]), "qz_prep": float(pose1_orn[2]), "qw_prep": float(pose1_orn[3]),
                "x_grasp": float(pose2_pos[0]), "y_grasp": float(pose2_pos[1]), "z_grasp": float(pose2_pos[2]),
                "qx_grasp": float(pose2_orn[0]), "qy_grasp": float(pose2_orn[1]), "qz_grasp": float(pose2_orn[2]), "qw_grasp": float(pose2_orn[3])
            }
            print("Grasp parameters computed:", self.grasp_params)
        
        try:
            # Check all grasp_params values
            if any(v is None for v in self.grasp_params.values()):
                raise ValueError("Missing grasp parameters in go_params.")
            self.success = call_ros2_service(
                "/grasp_service",
                "action_interfaces/srv/Grasp",
                {
                    "x_prep":   self.grasp_params["x_prep"],
                    "y_prep":   self.grasp_params["y_prep"],
                    "z_prep":   self.grasp_params["z_prep"],
                    "qx_prep":  self.grasp_params["qx_prep"],
                    "qy_prep":  self.grasp_params["qy_prep"],
                    "qz_prep":  self.grasp_params["qz_prep"],
                    "qw_prep":  self.grasp_params["qw_prep"],
                    "x_grasp":  self.grasp_params["x_grasp"],
                    "y_grasp":  self.grasp_params["y_grasp"],
                    "z_grasp":  self.grasp_params["z_grasp"],
                    "qx_grasp": self.grasp_params["qx_grasp"],
                    "qy_grasp": self.grasp_params["qy_grasp"],
                    "qz_grasp": self.grasp_params["qz_grasp"],
                    "qw_grasp": self.grasp_params["qw_grasp"],
                }
            )
        except ValueError as e:
            print(e)
            
        while True:
            if self.success:
                print("✅ Grasp other things action executed successfully.")
                self.grasped_thing = self.target
                break
            else:
                print("❌ Grasp other things action failed, retrying...")
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()

    def action_stir(self):
        """Stir a pot for a specified duration using class variables.
        Uses self.stir_time and updates self.success.
        
        Raises:
            ValueError: If stir_time is not provided.
        """
        self.success = False  # Reset success status
        print("Execute stir action")
        try:
            if self.stir_time is None:
                raise ValueError("Missing stir time.")
            self.success = call_ros2_service("/stir_service", "action_interfaces/srv/Stir", {
                "center_x": 0.62,
                "center_y": -0.37,
                "center_z": 0.49,
                "radius": 0.05,
                "start_angle_deg": 0.0,
                "move_down_offset": 0.1,
                "speed": 0.5,
                "stir_time": self.stir_time
            })
        except ValueError as e:
            print(e)
        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                print("✅ Stir action executed successfully.")
                break
            else:
                print("❌ Stir action failed, retrying...")
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()

    def action_reset(self):
        """Reset the robot to its home position.
        Updates self.success.
        """
        self.success = False  # Reset success status
        print("Execute reset action")
        self.success = call_ros2_service("/reset_service", "std_srvs/srv/Trigger", {})

        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                print("✅ Reset action executed successfully.")
                break
            else:
                print("❌ Reset action failed, retrying...")

    def action_add(self):
        """Add an ingredient a specified number of times using class variables.
        Uses self.add_times and updates self.success.

        Raises:
            ValueError: If add_times is not provided.
        """
        self.success = False  # Reset success status
        print("Execute add action")
        
        try:
            if self.add_times is None:
                raise ValueError("Missing add times.")
            self.success = call_ros2_service("/add_service", "action_interfaces/srv/Add", 
                {"times": self.add_times})
        except ValueError as e:
            print(e)

        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                print("✅ Add action executed successfully.")
                break
            else:
                print("❌ Add action failed, retrying...")
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')  
                self.action_open()
                self.action_reset()

    def action_return_back(self):
        """Return the robot to its original position after placing an object.
        Uses self.grasp_params to set self.rb_params and updates self.success and self.grasped_thing.

        Raises:
            ValueError: If rb_params are not provided or incomplete.
        """
        self.success = False  # Reset success status
        print("Execute back_move action, moving back to original point")
        
        if any(v is not None for v in self.grasp_params.values()):
            self.rb_params["x_prep"] = self.grasp_params["x_prep"] 
            self.rb_params["y_prep"] = self.grasp_params["y_prep"]
            self.rb_params["z_prep"] = self.grasp_params["z_prep"]
            self.rb_params["qx_prep"] = self.grasp_params["qx_prep"]
            self.rb_params["qy_prep"] = self.grasp_params["qy_prep"]
            self.rb_params["qz_prep"] = self.grasp_params["qz_prep"]
            self.rb_params["qw_prep"] = self.grasp_params["qw_prep"]
            self.rb_params["x_place"] = self.grasp_params["x_grasp"]
            self.rb_params["y_place"] = self.grasp_params["y_grasp"]
            self.rb_params["z_place"] = self.grasp_params["z_grasp"]
            self.rb_params["qx_place"] = self.grasp_params["qx_grasp"]
            self.rb_params["qy_place"] = self.grasp_params["qy_grasp"]
            self.rb_params["qz_place"] = self.grasp_params["qz_grasp"]
            self.rb_params["qw_place"] = self.grasp_params["qw_grasp"]
        else:
            print("No return_back parameters provided, please check! ")
            
        try:
            # Check if there are any unassigned parameters in rb_params
            if any(v is None for v in self.rb_params.values()):
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()
                raise ValueError("Missing return_back parameters in rb_params.")
                
            else:
                self.success = call_ros2_service(
                    "/return_back_service",
                    "action_interfaces/srv/ReturnBack",
                    {
                        "x_prep":   self.rb_params["x_prep"],
                        "y_prep":   self.rb_params["y_prep"],
                        "z_prep":   self.rb_params["z_prep"],
                        "qx_prep":  self.rb_params["qx_prep"],
                        "qy_prep":  self.rb_params["qy_prep"],
                        "qz_prep":  self.rb_params["qz_prep"],
                        "qw_prep":  self.rb_params["qw_prep"],
                        "x_place":  self.rb_params["x_place"],
                        "y_place":  self.rb_params["y_place"],
                        "z_place":  self.rb_params["z_place"],
                        "qx_place": self.rb_params["qx_place"],
                        "qy_place": self.rb_params["qy_place"],
                        "qz_place": self.rb_params["qz_place"],
                        "qw_place": self.rb_params["qw_place"],
                    }
                )
        except ValueError as e:
            print(e)
        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                print("✅ Return back action executed successfully.")
                self.grasped_thing = ""  # Reset grasped thing after returning back
                break
            else:
                print("❌ Return back action failed, retrying...")
                self.grasped_thing = ""
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()

    def action_open(self):
        """Open the robot's gripper.
        Updates self.success.
        """
        self.success = False  # Reset success status
        print("Execute open action")
        self.success = call_ros2_service("/open_service", "std_srvs/srv/Trigger", {})
        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                self.grasped_thing = ""
                print("✅ Open action executed successfully.")
                break
            else:
                self.grasped_thing = ""
                print("❌ Open action failed, retrying...")

    def action_close(self):
        """Close the robot's gripper.
        Updates self.success.   
        """
        self.success = False  # Reset success status
        print("Execute close action")
        self.success = call_ros2_service("/close_service", "std_srvs/srv/Trigger", {})
        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                print("✅ Close action executed successfully.")
                break
            else:
                print("❌ Close action failed, retrying...")

    def action_grasp_detection(self):
        """Detect if the robot has successfully grasped an object.
        Updates self.success.
        """
        self.success = False  # Reset success status
        print("Execute grasp detection action")
        self.success = call_ros2_service("/grasp_detection_service", "action_interfaces/srv/GraspDetect", {})
        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                print("✅ Grasp detection action executed successfully.")
                break
            else:
                print("❌ Grasp detection action failed, retrying...")
                play_text_to_speech('Sorry, something went wrong. Please help me.', language='en')
                self.action_open()
                self.action_reset()

    def get_grasps(self):
        """
        Given a point cloud, compute and return the best grasp poses using OBB and GraspGeneration.
        Args:
            all_points_arr: ndarray of shape (N, 3), perceived point cloud in world coordinates
            all_colors_arr: ndarray of shape (N, 3), colors corresponding to the point cloud
        Returns:
            best_pose: list of [pose1_pos, pose1_orn, pose2_pos, pose2_orn]
            top_10_grasps: list of top 10 grasps
            valid_grasps: list of valid grasps
        Raises:
            ValueError: If point cloud is empty or invalid.
        """
        try:
            if self.all_points_arr is None or len(self.all_points_arr) == 0:
                raise ValueError("Point cloud is empty or None.")
        except ValueError as e:
            print(e)
            return None, None, None

        _, rotation_matrix, center = compute_obb(self.all_points_arr)
        grasp_generator = GraspGeneration(center, rotation_matrix)
        pose1_pos, pose1_orn, pose2_pos, pose2_orn, top_10_grasps, valid_grasps = grasp_generator.final_compute_poses(self.all_points_arr, self.all_colors_arr, visualize=False, grasp_type='otherthings')
        best_pose = [pose1_pos, pose1_orn, pose2_pos, pose2_orn]
        print("Best grasp pose:", best_pose)
        print("Top 10 grasps:", top_10_grasps)
        print("Valid grasps:", valid_grasps)
        return best_pose, top_10_grasps, valid_grasps
    
    def action_move_offset(self):
        """Move the robot's end effector by a small offset.
        Updates self.success.
        """
        self.success = False  # Reset success status
        print("Execute move offset action")
        self.success = call_ros2_service("/move_offset_service", "action_interfaces/srv/MoveOffset", {"delta": self.delta, "direction": self.direction})
        # time.sleep(3)  # Wait for service call to complete
        while True:
            if self.success:
                print("✅ Move offset action executed successfully.")
                break
            else:
                print("❌ Move offset action failed, retrying...")

    def get_object_pos(self, vlm_client, object_name):
        """Get the position of a specified object using the VLM client.
        
        Args:
            object_name (str): The name of the object to locate.
        
        Returns:
            position (tuple): The (x, y, z) coordinates of the object.
        
        Raises:
            ValueError: If the object cannot be located.
        """
        if vlm_client is None:
            raise ValueError("VLM client is not initialized.")
        
        self.target = object_name
        self.action_perceive(vlm_client)
        print(f"center_world_points: {self.center_world_points}")
        return self.center_world_points




# def action_perceive(target, grasped_thing, vlm_client, l_img_path, l_depth_path, r_img_path, r_depth_path):
#     """Perceive the target object using VLM_agent and point cloud processing.

#     Args:
#         target (str): The target object to perceive.
#         grasped_thing (str): The currently grasped object, if any.
#         vlm_client: The VLM client instance of Mistralmodel() for perception.
#         l_img_path (str): Path to the left RGB image.
#         l_depth_path (str): Path to the left depth image (.npy).
#         r_img_path (str): Path to the right RGB image.
#         r_depth_path (str): Path to the right depth image (.npy).
            
#     Returns: 
#         move_params: dict with keys "move_x", "move_y", "move_z", "move_qx", "move_qy", "move_qz", "move_qw"
#         all_points_arr: ndarray of shape (N, 3), perceived point cloud in world coordinates
#         all_colors_arr: ndarray of shape (N, 3), colors corresponding to the point cloud
#         target_center_point: ndarray of shape (3,), centroid of the perceived object
#         target_max_z_point: ndarray of shape (3,), point with the highest z-coordinate
#         center_world_points: ndarray of shape (3,), perceived target point in world coordinates
#         success: bool, whether perception was successful
    
#     Raises:
#         ValueError: If perception fails or required data is missing.
#     """
#     move_params = {"move_x" : None, "move_y" : None, "move_z" : None, "move_qx" : None, "move_qy" : None, "move_qz" : None, "move_qw" : None}
#     success = False  # Reset success status
#     if target ==  grasped_thing:
#         success = True
#         print(f"✅ Target '{target}' already grasped, skipping perception.")
#         return move_params, None, None, None, None, None, success        
#     elif target == "user person":
#         move_params = {"move_x" : 0.718029693832728, "move_y" : -0.07702313108387482, "move_z" : 0.4, "move_qx" : 0.707, "move_qy" : -0.707, "move_qz" : 0.0, "move_qw" : 0.0}
#         success = True
#         print("✅ Perceived user person, moving to target point.")
#         return move_params, None, None, None, None, None, success
#     elif target == "spoon":
#         move_params = {"move_x" : 0.406, "move_y" : -0.313, "move_z" : 0.6, "move_qx" : 0.999, "move_qy" : 0.023, "move_qz" : 0.026, "move_qw" : 0.001}
#         success = True
#         print("✅ Perceived spoon, moving to target point.")
#         return move_params, None, None, None, None, None, success
#     elif target == "soup pot":
#         move_params = {"move_x" : 0.6, "move_y" : -0.3, "move_z" : 0.32, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
#         success = True
#         print("✅ Perceived soup pot, moving to target point.")
#         return move_params, None, None, None, None, None, success
#     elif target == "salt bottle":
#         move_params = {"move_x" : 0.434, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
#         success = True
#         print("✅ Perceived salt bottle, moving to target point.")
#         return move_params, None, None, None, None, None, success
#     elif target == "pepper bottle":
#         move_params = {"move_x" : 0.27, "move_y" : 0.561, "move_z" : 0.523, "move_qx" : 0.725, "move_qy" : 0.688, "move_qz" : 0.023, "move_qw" : -0.007}
#         success = True
#         print("✅ Perceived pepper bottle, moving to target point.")
#         return move_params, None, None, None, None, None, success

#     else:  
#         print(f"Perceiving target: {target}")
        
#         if_find_r, response_r, center_world_points_r, all_world_points_r ,color_r = get_cam_world_points(
#         vlm_client,
#         target,
#         rgb_path= r_img_path,
#         depth_path= r_depth_path,
#         pixels_to_world_func = pixels_to_world_right,
#         name= "right",
        
#         )

#         # if_find_r, response_r, center_world_points_r, all_world_points_r, color_r = None, None, None, None, None
        

#         if_find_l, response_l, center_world_points_l, all_world_points_l, color_l  = get_cam_world_points(
#             vlm_client,
#             target,
#             rgb_path= l_img_path,
#             depth_path= l_depth_path,
#             pixels_to_world_func = pixels_to_world_left,
#             name= "left",
            
#         )

#         # if_find_l, response_l, center_world_points_l, all_world_points_l, color_l = None, None, None, None,None


#         if all_world_points_r is not None and all_world_points_l is not None:
                
#             mask_valid = ~np.isnan(all_world_points_r).any(axis=1)
#             all_world_points_r = all_world_points_r[mask_valid]
#             color_r = color_r[mask_valid]

#             print(f"World point in right camera: {all_world_points_r}, in left camera: {all_world_points_l}")

#             mask_valid = ~np.isnan(all_world_points_l).any(axis=1)
#             all_world_points_l = all_world_points_l[mask_valid]
#             color_l = color_l[mask_valid]

#             # If both sides have point clouds, use ICP to align and merge
#             all_points, all_colors, T = filter_and_merge_icp_translation_only(
#                 all_world_points_l, color_l, all_world_points_r, color_r
#             )           

#             # # Example of merging point clouds
#             # all_points = list(all_world_points_r) + list(all_world_points_l)
#             # all_colors = list(color_r) + list(color_l)

#             center_world_points = (center_world_points_l + center_world_points_r)/2
#             success = True
            
#         elif all_world_points_r is not None and all_world_points_l is None:

#             mask_valid = ~np.isnan(all_world_points_r).any(axis=1)
#             all_world_points_r = all_world_points_r[mask_valid]
#             color_r = color_r[mask_valid]

#             print(f"World point in right camera: {all_world_points_r}, in left camera: None")

#             # If only the right side has point clouds, use the right side's point clouds directly
#             pcd_r = preprocess_pointcloud(all_world_points_r, color_r, voxel_size=0.005, nb_points=10, radius=0.02)
#             all_world_points_r = np.asarray(pcd_r.points)
#             color_r = np.asarray(pcd_r.colors)

#             all_points = list(all_world_points_r)
#             all_colors = list(color_r)
#             center_world_points = center_world_points_r
#             success = True
            
#         elif all_world_points_r is None and all_world_points_l is not None:
            
#             mask_valid = ~np.isnan(all_world_points_l).any(axis=1)
#             all_world_points_l = all_world_points_l[mask_valid]
#             color_l = color_l[mask_valid]

#             print(f"World point in right camera: None, in left camera: {all_world_points_l}")

#             # If only the left side has point clouds, use the left side's point clouds directly
#             pcd_l = preprocess_pointcloud(all_world_points_l, color_l, voxel_size=0.005, nb_points=10, radius=0.02)
#             all_world_points_l = np.asarray(pcd_l.points)
#             color_l = np.asarray(pcd_l.colors)

#             all_points = list(all_world_points_l)
#             all_colors = list(color_l)
#             center_world_points = center_world_points_l
#             success = True
#         else:
#             print("❌ Failed to perceive target points in both cameras.")
#             if response_r:
#                 play_text_to_speech(response_r, language='en')
#             else:
#                 play_text_to_speech("Sorry, I can't find that. Please try again.", language='en')

#             success = False
#             return None, None, None, None, None, None, success


#         # Calculate centroid
#         all_points_arr = np.array(all_points)
#         all_colors_arr = np.array(all_colors)
        
        
#         mask_valid = ~np.isnan(all_points_arr).any(axis=1)
#         all_points_arr = all_points_arr[mask_valid]
#         all_colors_arr = all_colors_arr[mask_valid]
#         if len(all_points_arr) == 0:
#             print("❌ All points are invalid (contain NaNs)")
#             success = False
        


#         # Calculate center point
#         center_world_points = center_world_points[0]
#         print ("center_world_points:", center_world_points)

#         # Calculate centroid
#         target_center_point = all_points_arr.mean(axis=0)
#         print("Centroid:", target_center_point)

#         # Calculate highest point in z-axis
#         max_z_index = np.argmax(all_points_arr[:,2])
#         target_max_z_point = all_points_arr[max_z_index]
#         print("Highest point in z-axis:", target_max_z_point)

#         # Calculate move_target_point
#         move_target_point = target_center_point.copy()
#         move_target_point[2] += 0.4  # Raise by 0.4 meters

#         print("Move target point:", move_target_point)

#         move_params["move_x"] = float(move_target_point[0])
#         move_params["move_y"] = float(move_target_point[1])
#         move_params["move_z"] = float(move_target_point[2])
#         move_params["move_qx"] = 1.0
#         move_params["move_qy"] = 0.0
#         move_params["move_qz"] = 0.0
#         move_params["move_qw"] = 0.0
        
#         open3d_show(all_points_arr, all_colors_arr, target_center_point, target_max_z_point, center_world_points)
#         return move_params, all_points_arr, all_colors_arr, target_center_point, target_max_z_point, center_world_points, success

# def action_move(move_params, target, grasped_thing):
#     """Move the robot to a specified target.

#     Args:
#         move_params: dict with keys "move_x", "move_y", "move_z", "move_qx", "move_qy", "move_qz", "move_qw"
#         target: string, the target to move to
#         grasped_thing: string, the currently grasped object
            
#     Returns:
#         success: bool, whether the move was successful
    
#     Raises:
#         ValueError: If move_params are not provided or incomplete.
#     """
#     success = False  # Reset success status
#     if target is grasped_thing:
#         success = True
#         print(f"✅ Target '{target}' already grasped, skipping move.")
#         return success
#     current_position = [0.0, 0.0, 0.0]  # Initialize current position

#     current_position[0], current_position[1], current_position[2] = call_fk_service()
#     print(f"Current position: {current_position}")

#     if current_position[2] < 0.3:   
#         try:
#             if any(v is None for v in move_params.values()):
#                 raise ValueError("Missing move parameters.")
#             success = call_ros2_service(
#                 "/move_cartesian_service",
#                 "action_interfaces/srv/Move",
#                 {
#                     "x": float(current_position[0]),
#                     "y": float(current_position[1]),
#                     "z": float(0.5),  # Raise to 0.5 meters
#                     "qx": 1.0,
#                     "qy": 0.0,
#                     "qz": 0.0,
#                     "qw": 0.0,
#                 }
#             )
#         except ValueError as e:
#             print(e)
#         # time.sleep(3)  # Wait for service call to complete
#         while True:
#             if success:
#                 print("✅ List action executed successfully.")
#                 break
#             else:
#                 print("❌ List action failed, retrying...")
    
    
    
#     if move_params["move_z"] < 0.35:   
#         try:
#             if any(v is None for v in move_params.values()):
#                 raise ValueError("Missing move parameters.")
#             success = call_ros2_service(
#                 "/move_cartesian_service",
#                 "action_interfaces/srv/Move",
#                 {
#                     "x": move_params["move_x"],
#                     "y": move_params["move_y"],
#                     "z":                    0.5,# Raise to 0.5 meters
#                     "qx": move_params["move_qx"],
#                     "qy": move_params["move_qy"],
#                     "qz": move_params["move_qz"],
#                     "qw": move_params["move_qw"],
#                 }
#             )
#         except ValueError as e:
#             print(e)
#         # time.sleep(3)  # Wait for service call to complete
#         while True:
#             if success:
#                 print("✅ List action executed successfully.")
#                 break
#             else:
#                 print("❌ List action failed, retrying...")
    
#     try:
#         if any(v is None for v in move_params.values()):
#             raise ValueError("Missing move parameters.")
#         success = call_ros2_service(
#             "/move_cartesian_service",
#             "action_interfaces/srv/Move",
#             {
#                 "x": move_params["move_x"],
#                 "y": move_params["move_y"],
#                 "z": move_params["move_z"],
#                 "qx": move_params["move_qx"],
#                 "qy": move_params["move_qy"],
#                 "qz": move_params["move_qz"],
#                 "qw": move_params["move_qw"],
#             }
#         )
#     except ValueError as e:
#         print(e)
#     # time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Move action executed successfully.")
#             break
#         # else:
#         #     print("❌ Move action failed, retrying...")
#     return success

# def action_grasp(target, all_points_arr, all_colors_arr, grasped_thing, grasp_params):
#     """Grasp an object using predefined parameters.
#         Objects including spoon, salt bottle, pepper bottle have been hardcoded with grasp parameters.

#     Args:
#         target: string, the target to grasp
#         all_points_arr: ndarray, Nx3, point cloud of the target object
#         all_colors_arr: ndarray, Nx3, colors corresponding to the point cloud
#         grasped_thing: string, the currently grasped object, if any
#         grasp_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
#                         "x_grasp", "y_grasp", "z_grasp", "qx_grasp", "qy_grasp", "qz_grasp", "qw_grasp"
            
#     Returns:
#         grasped_thing: string, the currently grasped object
#         grasp_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
#                         "x_grasp", "y_grasp", "z_grasp", "qx_grasp", "qy_grasp", "qz_grasp", "qw_grasp"
#         success: bool, whether the grasp was successful
    
#     Raises:
#         ValueError: If any grasp parameters are missing.
#     """
#     success = False  # Reset success status
#     if target is grasped_thing:
#         success = True
#         print(f"✅ Target '{target}' already grasped, skipping grasp.")
#         return grasped_thing, grasp_params, success
#     print("Execute grasp action")

#     # put the value into go_params by grasp strategy with points cloud or something else
#     if target == "spoon": 
#         grasp_params = {"x_prep": 0.406, "y_prep": -0.313, "z_prep": 0.57, "qx_prep": 0.999, "qy_prep": 0.023, "qz_prep": 0.026, "qw_prep": 0.001,
#                 "x_grasp": 0.406, "y_grasp": -0.313, "z_grasp": 0.3, "qx_grasp": 0.999, "qy_grasp": 0.023, "qz_grasp": 0.026, "qw_grasp": 0.001}
#     elif target == "pepper bottle":
#         grasp_params = {"x_prep": 0.27, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
#                     "x_grasp": 0.27, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}
#     elif target == "salt bottle":
#         grasp_params = {"x_prep": 0.434, "y_prep": 0.561, "z_prep": 0.523, "qx_prep": 0.725, "qy_prep": 0.688, "qz_prep": 0.023, "qw_prep": -0.007,
#                     "x_grasp": 0.434, "y_grasp": 0.561, "z_grasp": 0.223, "qx_grasp": 0.725, "qy_grasp": 0.688, "qz_grasp": 0.023, "qw_grasp": -0.007}

#     else:
#         if all_points_arr is None:
#             print("❌ Failed to perceive target points in both cameras.")
#             success = False
#             return grasped_thing, grasp_params, success
#         #     maybe other method to determine the grasp strategy
#         else:
#         #     ................
#         #     now we have world_points of target (all_points_arr) , which is a array , each element is a point in world coordinates
#         #     put the value into gf_params by grasp strategy with points cloud or something else   
#             obb_corners, rotation_matrix, center = compute_obb(all_points_arr)

#             visualize_obb_and_center(all_points_arr, all_colors_arr, obb_corners, center)

#             grasp_generator = GraspGeneration(center, rotation_matrix)
#             pose1_pos, pose1_orn, pose2_pos, pose2_orn = grasp_generator.final_compute_poses(all_points_arr, all_colors_arr, visualize=True, grasp_type='otherthings')    
#             # print("Pose 1 - Position:", pose1_pos, "Orientation:", pose1_orn)
#             # print("Pose 2 - Position:", pose2_pos, "Orientation:", pose2_orn)
            
#             if pose1_pos is None or pose1_orn is None or pose2_pos is None or pose2_orn is None:
#                 print("❌ Failed to compute grasp poses from OBB.")
#                 success = False
#                 return grasped_thing, grasp_params, success

#             grasp_params = {
#                 "x_prep": float(pose1_pos[0]), "y_prep": float(pose1_pos[1]), "z_prep": float(pose1_pos[2]),
#                 "qx_prep": float(pose1_orn[0]), "qy_prep": float(pose1_orn[1]), "qz_prep": float(pose1_orn[2]), "qw_prep": float(pose1_orn[3]),
#                 "x_grasp": float(pose2_pos[0]), "y_grasp": float(pose2_pos[1]), "z_grasp": float(pose2_pos[2]),
#                 "qx_grasp": float(pose2_orn[0]), "qy_grasp": float(pose2_orn[1]), "qz_grasp": float(pose2_orn[2]), "qw_grasp": float(pose2_orn[3])
#             }
#             print("Grasp parameters computed from OBB:", grasp_params)      
    
#     try:
#         # Check all grasp_params values
#         if any(v is None for v in grasp_params.values()):
#             raise ValueError("Missing grasp parameters in go_params.")
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
        
        
#     # time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Grasp other things action executed successfully.")
#             grasped_thing = target
#             break
#         else:
#             print("❌ Grasp other things action failed, retrying...")
    
#     return grasped_thing, grasp_params, success
    
# def action_stir(stir_time):
#     """Stir a pot for a specified duration.

#     Args:
#         stir_time: in seconds
    
#     Returns:
#         success: bool, whether the stir was successful
    
#     Raises:
#         ValueError: If stir_time is not provided.
#     """
#     success = False  # Reset success status
#     print("Execute stir action")
#     # time.sleep(5)
#     # success = True
#     try:
#         if stir_time is None:
#             raise ValueError("Missing stir time.")
#         success = call_ros2_service("/stir_service", "action_interfaces/srv/Stir", {
#             "center_x": 0.62,
#             "center_y": -0.37,
#             "center_z": 0.42,
#             "radius": 0.05,
#             "start_angle_deg": 0.0,
#             "move_down_offset": 0.1,
#             "speed": 0.5,
#             "stir_time": stir_time
#         })
#     except ValueError as e:
#         print(e)
#     time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Stir action executed successfully.")
#             break
#         else:
#             print("❌ Stir action failed, retrying...")
#     return success

# def action_reset():
#     """Reset the robot to its home position.

#     Args:
#         None
            
#     Returns:
#         success: bool, whether the reset was successful
    
#     Raises:
#         None
#     """
#     success = False  # Reset success status
#     print("Execute reset action")
#     # time.sleep(5)
#     # success = True
#     success = call_ros2_service("/reset_service", "std_srvs/srv/Trigger", {})

#     time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Reset action executed successfully.")
#             break
#         else:
#             print("❌ Reset action failed, retrying...")
#     return success

# def action_add(add_times):
#     """Add an ingredient a specified number of times.

#     Args:
#         add_times: integer, number of times to add the ingredient

#     Returns:
#         success: bool, whether the add was successful

#     Raises:
#         ValueError: If add_times is not provided.
#     """
#     success = False  # Reset success status
#     print("Execute add action")
    
#     try:
#         if add_times is None:
#             raise ValueError("Missing add times.")
#         success = call_ros2_service("/add_service", "action_interfaces/srv/Add", 
#             {"times": add_times})
#     except ValueError as e:
#         print(e)

#     time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Add action executed successfully.")
#             break
#         else:
#             print("❌ Add action failed, retrying...")
#     return success

# def action_return_back(grasp_params):
#     """Return the robot to its original position after placing an object.

#     Args:
#         grasp_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
#                       "x_grasp", "y_grasp", "z_grasp", "qx_grasp", "qy_grasp", "qz_grasp", "qw_grasp"

#     Returns:
#         grasped_thing: string, always "" because the robot has placed the object down
#         rb_params: dict with keys "x_prep", "y_prep", "z_prep", "qx_prep", "qy_prep", "qz_prep", "qw_prep",
#                    "x_place", "y_place", "z_place", "qx_place", "qy_place", "qz_place", "qw_place"
#         success: bool, whether the return back was successful

#     Raises:
#         ValueError: If rb_params are not provided or incomplete.
#     """
#     rb_params = {
#         "x_prep": None, "y_prep": None, "z_prep": None,
#         "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
#         "x_place": None, "y_place": None, "z_place": None,
#         "qx_place": None, "qy_place": None, "qz_place": None, "qw_place": None
#     }
#     grasped_thing = ""
#     success = False  # Reset success status
#     print("Execute back_move action, moving back to original point")
#     # time.sleep(5)
#     # success = True
    
#     if any(v is not None for v in grasp_params.values()):
#         rb_params["x_prep"] = grasp_params["x_prep"] 
#         rb_params["y_prep"] = grasp_params["y_prep"]
#         rb_params["z_prep"] = grasp_params["z_prep"]
#         rb_params["qx_prep"] = grasp_params["qx_prep"]
#         rb_params["qy_prep"] = grasp_params["qy_prep"]
#         rb_params["qz_prep"] = grasp_params["qz_prep"]
#         rb_params["qw_prep"] = grasp_params["qw_prep"]
#         rb_params["x_place"] = grasp_params["x_grasp"]
#         rb_params["y_place"] = grasp_params["y_grasp"]
#         rb_params["z_place"] = grasp_params["z_grasp"]
#         rb_params["qx_place"] = grasp_params["qx_grasp"]
#         rb_params["qy_place"] = grasp_params["qy_grasp"]
#         rb_params["qz_place"] = grasp_params["qz_grasp"]
#         rb_params["qw_place"] = grasp_params["qw_grasp"]
#     else:
#         print("No return_back parameters provided, please check! ")
        
    

#     try:
#         # Check if there are any unassigned parameters in rb_params
#         if any(v is None for v in rb_params.values()):
#             raise ValueError("Missing return_back parameters in rb_params.")
            
#         else:
#             success = call_ros2_service(
#                 "/return_back_service",
#                 "action_interfaces/srv/ReturnBack",
#                 {
#                     "x_prep":   rb_params["x_prep"],
#                     "y_prep":   rb_params["y_prep"],
#                     "z_prep":   rb_params["z_prep"],
#                     "qx_prep":  rb_params["qx_prep"],
#                     "qy_prep":  rb_params["qy_prep"],
#                     "qz_prep":  rb_params["qz_prep"],
#                     "qw_prep":  rb_params["qw_prep"],
#                     "x_place":  rb_params["x_place"],
#                     "y_place":  rb_params["y_place"],
#                     "z_place":  rb_params["z_place"],
#                     "qx_place": rb_params["qx_place"],
#                     "qy_place": rb_params["qy_place"],
#                     "qz_place": rb_params["qz_place"],
#                     "qw_place": rb_params["qw_place"],
#                 }
#             )
#     except ValueError as e:
#         print(e)
#     time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Return back action executed successfully.")
#             grasped_thing = ""  # Reset grasped thing after returning back
#             break
#         else:
#             print("❌ Return back action failed, retrying...")
#     return grasped_thing, rb_params, success

# def action_open():
#     """Open the robot's gripper.

#     Args:
#         None
            
#     Returns:
#         success: bool, whether the open was successful
    
#     Raises:
#         None
#     """
#     success = False  # Reset success status
#     print("Execute open action")
#     success = call_ros2_service("/open_service", "std_srvs/srv/Trigger", {})
#     time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Open action executed successfully.")
#             break
#         else:
#             print("❌ Open action failed, retrying...")
#     return success

# def action_close():
#     """Close the robot's gripper.
    
#     Args:
#         None
    
#     Returns:
#         success: bool, whether the close was successful
    
#     Raises:
#         None
#     """
#     success = False  # Reset success status
#     print("Execute close action")
#     success = call_ros2_service("/close_service", "std_srvs/srv/Trigger", {})
#     time.sleep(3)  # Wait for service call to complete
#     while True:
#         if success:
#             print("✅ Close action executed successfully.")
#             break
#         else:
#             print("❌ Close action failed, retrying...")
#     return success


