import subprocess
import json
import time
import yaml
import numpy as np
import cv2
# from src.VLM_agent.agent import VLM_agent 
# from src.pixel_world.pixel_and_world import pixels_to_world_left, pixels_to_world_right, world_to_pixels_left, world_to_pixels_right

def call_ros2_service(service_name, service_type, args_dict):
    # 把字典转成一行的 YAML 字符串
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

# # move service test
# move_params = {"move_x" : 0.5, "move_y" : 0.4, "move_z" : 0.3, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
# call_ros2_service("/move_service", "action_interfaces/srv/Move", {"x": move_params["move_x"] , "y": move_params["move_y"], "z": move_params["move_z"], "qx": move_params["move_qx"], "qy": move_params["move_qy"], "qz": move_params["move_qz"], "qw": move_params["move_qw"]})


# # grasp flavoring service test
# gf_params = {"x_prep": 0.5, "y_prep": 0.0, "z_prep": 0.5, "qx_prep": 1.0, "qy_prep": 0.0, "qz_prep": 0.0, "qw_prep": 0.0,
#                                  "x_grasp": 0.5, "y_grasp": 0.0, "z_grasp": 0.4, "qx_grasp": 1.0, "qy_grasp": 0.0, "qz_grasp": 0.0, "qw_grasp": 0.0}
# call_ros2_service("/grasp_service", "action_interfaces/srv/Grasp", {"x_prep": gf_params["x_prep"], "y_prep": gf_params["y_prep"], "z_prep": gf_params["z_prep"], "qx_prep": gf_params["qx_prep"], "qy_prep": gf_params["qy_prep"], "qz_prep": gf_params["qz_prep"], "qw_prep": gf_params["qw_prep"], 
#                                                                                           "x_grasp": gf_params["x_grasp"], "y_grasp": gf_params["y_grasp"], "z_grasp": gf_params["z_grasp"], "qx_grasp": gf_params["qx_grasp"], "qy_grasp": gf_params["qy_grasp"], "qz_grasp": gf_params["qz_grasp"], "qw_grasp": gf_params["qw_grasp"]})


# # grasp other things service test
# go_params = {"x_prep": 0.5, "y_prep": 0.0, "z_prep": 0.5, "qx_prep": 1.0, "qy_prep": 0.0, "qz_prep": 0.0, "qw_prep": 0.0,
#                                  "x_grasp": 0.5, "y_grasp": 0.0, "z_grasp": 0.4, "qx_grasp": 1.0, "qy_grasp": 0.0, "qz_grasp": 0.0, "qw_grasp": 0.0}
# call_ros2_service("/grasp_service", "action_interfaces/srv/Grasp", {"x_prep": go_params["x_prep"], "y_prep": go_params["y_prep"], "z_prep": go_params["z_prep"], "qx_prep": go_params["qx_prep"], "qy_prep": go_params["qy_prep"], "qz_prep": go_params["qz_prep"], "qw_prep": go_params["qw_prep"], 
#                                                                                           "x_grasp": go_params["x_grasp"], "y_grasp": go_params["y_grasp"], "z_grasp": go_params["z_grasp"], "qx_grasp": go_params["qx_grasp"], "qy_grasp": go_params["qy_grasp"], "qz_grasp": go_params["qz_grasp"], "qw_grasp": go_params["qw_grasp"]})


# # stir service test
# stir_time = 30
# success = call_ros2_service("/stir_service", "action_interfaces/srv/Stir", {
#                 "center_x": 0.5,
#                 "center_y": -0.4,
#                 "center_z": 0.4,
#                 "radius": 0.08,
#                 "start_angle_deg": 0.0,
#                 "move_down_offset": 0.1,
#                 "speed": 0.5,
#                 "stir_time": stir_time
#             })
# print(f"Stir action success: {success}")


# # reset service test
# success = call_ros2_service("/reset_service", "std_srvs/srv/Trigger", {})
# print(f"Reset action success: {success}")


# # add service test
# add_times = 5
# success = call_ros2_service("/add_service", "action_interfaces/srv/Add", {
#                 "times": add_times,
#             })
# print(f"Add action success: {success}")


# # return back service test
# rb_params = {"x_prep": 0.5, "y_prep": 0.4, "z_prep": 0.4, "qx_prep": 1.0, "qy_prep": 0.0, "qz_prep": 0.0, "qw_prep": 0.0,
#                                  "x_place": 0.5, "y_place": 0.4, "z_place": 0.3, "qx_place": 1.0, "qy_place": 0.0, "qz_place": 0.0, "qw_place": 0.0}
# success = call_ros2_service("/return_back_service", "action_interfaces/srv/ReturnBack", {"x_prep": rb_params["x_prep"], "y_prep": rb_params["y_prep"], "z_prep": rb_params["z_prep"], "qx_prep": rb_params["qx_prep"], "qy_prep": rb_params["qy_prep"], "qz_prep": rb_params["qz_prep"], "qw_prep": rb_params["qw_prep"], 
#                                                                                           "x_place": rb_params["x_place"], "y_place": rb_params["y_place"], "z_place": rb_params["z_place"], "qx_place": rb_params["qx_place"], "qy_place": rb_params["qy_place"], "qz_place": rb_params["qz_place"], "qw_place": rb_params["qw_place"]})


# # open service test
# success = call_ros2_service("/open_service", "std_srvs/srv/Trigger", {})
# print(f"Open action success: {success}")



# close service test
success = call_ros2_service("/close_service", "std_srvs/srv/Trigger", {})
print(f"Close action success: {success}")