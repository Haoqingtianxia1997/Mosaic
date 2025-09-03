from src.execute.element_action import *

def execute_action_sequence(actions, vlm_client):
    """Wrapper function for backward compatibility."""
    executor = ActionExecutor()
    return executor.execute_action_sequence(actions, vlm_client)








# def execute_action_sequence(actions, vlm_client):
#     """
#     Execute a sequence of actions serially, waiting for each service to complete successfully before proceeding to the next.
#     """
  
#     # photo and depth image paths
#     CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        
#     IMAGE_FOLDER_PATH = os.path.abspath(os.path.join(
#         CURRENT_DIR, "../../../manipulation_ws/saved_images"
#     ))

#     l_img_path = os.path.join(IMAGE_FOLDER_PATH, "l_rgb.png")
#     l_depth_path = os.path.join(IMAGE_FOLDER_PATH, "l_depth.npy")
#     r_img_path = os.path.join(IMAGE_FOLDER_PATH, "r_rgb.png")
#     r_depth_path = os.path.join(IMAGE_FOLDER_PATH, "r_depth.npy")


#     #declare parameters for each action type
#     move_target_point = None # perceived target point
#     target_max_z_point = None # highest point in z-axis
#     target_center_point = None # centroid point
#     center_world_points = None # perceived image target point in world coordinates
#     all_points_arr = None # all points in world coordinates
#     all_colors_arr = None # all colors in world coordinates

#     target = None # current target, used to update the target in each action
    
#     move_params = {"move_x" : None, "move_y" : None, "move_z" : None, "move_qx" : None, "move_qy" : None, "move_qz" : None, "move_qw" : None}

#     grasp_params = {"x_prep": None, "y_prep": None, "z_prep": None, "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
#                                 "x_grasp": None, "y_grasp": None, "z_grasp": None, "qx_grasp": None, "qy_grasp": None, "qz_grasp": None, "qw_grasp": None}# grasp 
    
#     rb_params = {"x_prep": None, "y_prep": None, "z_prep": None, "qx_prep": None, "qy_prep": None, "qz_prep": None, "qw_prep": None,
#                                 "x_place": None, "y_place": None, "z_place": None, "qx_place": None, "qy_place": None, "qz_place": None, "qw_place": None}# return back
#     stir_time = 30
#     add_times = 2
    
#     # state flags
#     grasped_thing = ""

#     success = True  # Used to track the success status of each action

#     for i, action in enumerate(actions):

#         # Check if the service was successful
#         if not success:
#             print(f"⛔ Aborting action sequence due to failure at step {i}.")
#             break

#         try:
#             print(f"\n▶️ Executing action {i+1}/{len(actions)}: {action}")
#             act_type = action["type"]

#             if action["target"] is not None:
#                 target = action["target"]
#                 print(f"🔍 Target: {target}")
#             params = action.get("parameters", {})

#             if "stir_time" in params:
#                 if params["stir_time"] is not None:
#                     stir_time = params["stir_time"]
#                     print(f"🔧 Parameters: {params}")
#                 else:
#                     stir_time =  stir_time
            
#             if "add_times" in params:
#                 if params["add_times"] is not None:
#                     add_times = params["add_times"]
#                     print(f"🔧 Add times: {add_times}")
#                 else:
#                     add_times = add_times
            
            
#             if act_type == "perceive":
#                 move_params, all_points_arr, all_colors_arr, _, _, _, success = action_perceive(target, grasped_thing, vlm_client, l_img_path, l_depth_path, r_img_path, r_depth_path)
                
#             elif act_type == "move":
#                 success = action_move(move_params, target, grasped_thing)

#             elif act_type == "grasp_otherthings":
#                 grasped_thing, grasp_params, success = action_grasp(target, all_points_arr, all_colors_arr, grasped_thing, grasp_params)
                            
#             elif act_type == "stir":
#                 success = action_stir(stir_time)
                
#             elif act_type == "reset":
#                 success = action_reset()

#             elif act_type == "add":
#                 success = action_add(add_times)

#             elif act_type == "return_back":
#                 grasped_thing, rb_params, success = action_return_back(grasp_params)
                        
#             elif act_type == "open":
#                 success = action_open()

#             elif act_type == "close":
#                 success = action_close()
            
#             else:
#                 print(f"⚠️ Unknown action type: {act_type}")
#                 success = False

#             time.sleep(0.5)  # Optional delay

#         except Exception as e:
#             print("❌ Exception inside execute_action_sequence:")
#             traceback.print_exc()
#             raise
#         print("✅ Action sequence completed.")





