import subprocess
import subprocess
import re
import shlex
import json
import time
import os
import yaml
import numpy as np
import cv2
import open3d as o3d
import numpy as np
import traceback
from src.VLM_agent.agent import VLM_agent  
from src.pixel_world.pixel_and_world import pixels_to_world_left, pixels_to_world_right, world_to_pixels_left, world_to_pixels_right
from src.transcribe.tts import run_tts, play_text_to_speech

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

def call_fk_service():
    """
    通过 shell 调用 `ros2 service call /fk_service action_interfaces/srv/Fk "{}"`
    返回 (x, y, z) 浮点数元组
    """
    cmd = 'ros2 service call /fk_service action_interfaces/srv/Fk "{}"'
    # 用 shlex.split 避免引号问题
    result = subprocess.run(
        shlex.split(cmd),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    if result.returncode != 0:
        raise RuntimeError(f"ros2 命令失败: {result.stderr}")

    # 在输出里找 x=..., y=..., z=...
    # 兼容下面两种格式：
    #   action_interfaces.srv.Fk_Response(x=0.39, y=0.29, z=0.30)
    #   x: 0.39
    pattern = r'[xyz]\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)'
    matches = re.findall(pattern, result.stdout)
    if len(matches) >= 3:
        x, y, z = map(float, matches[:3])
        return x, y, z
    else:
        # 第二种 YAML 风格
        yaml_pat = r'^\s*[xyz]:\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)\s*$'
        found = {}
        for line in result.stdout.splitlines():
            m = re.match(yaml_pat, line)
            if m:
                key = line.strip().split(':')[0]
                found[key] = float(m.group(1))
        if {'x', 'y', 'z'} <= found.keys():
            return found['x'], found['y'], found['z']
        raise ValueError("未能从输出中解析到 x y z")

def get_cam_world_points(
    target,
    rgb_path,
    depth_path,
    pixels_to_world_func,
    agent_image_path=None,
):
    """
    target: 感知目标
    rgb_path: 彩色图路径
    depth_path: 深度图.npy路径
    pixels_to_world_func:嵌入转换函数（如 pixels_to_world_left/right）
    agent_image_path: 若要单独指定传给VLM_agent的图片，可以用，否则为rgb_path
    """
    img_path = agent_image_path if agent_image_path else rgb_path
    if_find, response,  box_center_point, seg_center_point, all_seg_points = VLM_agent(target, image_path=img_path)
    if not if_find:
        print(f"❌ Failed to perceive target: {target}")
        return if_find, response, None, None
    
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
            # 兼容像 [x, y] 这种格式
            points = [tuple(points)]
        
        pixel_points = np.array(points, dtype=int)
        u = pixel_points[:, 0]
        v = pixel_points[:, 1]
        depths = depth_img[v, u]
        target_point = pixel_points.tolist()
        world_points, _ = pixels_to_world_func(target_point, depths, rgb_img=rgb_img)
        result.append(world_points)
    
    center_world_points, all_world_points = result
  
    return if_find, response, center_world_points, all_world_points

def merge_points_icp(world_points_r, world_points_l, threshold=0.02, visualize=False):
    """
    对左点云用ICP配准到右点云，然后合并两部分。
    world_points_r, world_points_l: list/ndarray, Nx3，右和左相机点云
    threshold: ICP收敛距离
    visualize: 是否弹窗预览点云
    返回: merged_points, transformation (左对齐到右的变换矩阵)
    """
    # 构建Open3D点云
    pcd_r = o3d.geometry.PointCloud()
    pcd_l = o3d.geometry.PointCloud()
    pcd_r.points = o3d.utility.Vector3dVector(np.array(world_points_r))
    pcd_l.points = o3d.utility.Vector3dVector(np.array(world_points_l))
    
    # 做ICP配准
    reg = o3d.pipelines.registration.registration_icp(
        pcd_l, pcd_r, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    pcd_l.transform(reg.transformation)
    
    # 合并点云
    merged_points = np.vstack([
        np.asarray(pcd_r.points),
        np.asarray(pcd_l.points)
    ])
    if visualize:
        pcd_merged = o3d.geometry.PointCloud()
        pcd_merged.points = o3d.utility.Vector3dVector(merged_points)
        o3d.visualization.draw_geometries([pcd_merged], window_name='ICP合并后点云')
    return merged_points, reg.transformation

def execute_action_sequence(actions):
    """
    串行执行动作序列，每一步等待其服务执行完且成功后才进行下一步。
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
    world_points_l = None # world points in left camera
    world_points_r = None # world points in right camera
    target_max_z_point = None # z轴方向上的最高点
    target_center_point = None # 质心点
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
    
    success = True  # 用于跟踪每个动作的成功状态


    for i, action in enumerate(actions):

        # 检查服务是否成功
        if not success:
            print(f"⛔ Aborting action sequence due to failure at step {i+1}.")
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
                else:
                    stir_time =  stir_time
            print(f"🔧 Parameters: {params}")
            if "add_times" in params:
                if params["add_times"] is not None:
                    add_times = params["add_times"]
                else:
                    add_times = add_times
            print(f"🔧 Add times: {add_times}")
            
            if act_type == "perceive":
                success = False  # 重置成功状态
                if target ==  grasped_thing:
                    success = True
                    print(f"✅ Target '{target}' already grasped, skipping perception.")
                    continue            
                elif target == "user person":
                    move_params = {"move_x" : 0.775, "move_y" : 0.081, "move_z" : 0.185, "move_qx" : -0.706, "move_qy" : 0.708, "move_qz" : 0.002, "move_qw" : 0.008}
                    success = True
                    print("✅ Perceived user person, moving to target point.")
                    continue
                elif target == "spoon":
                    move_params = {"move_x" : 0.406, "move_y" : -0.313, "move_z" : 0.4, "move_qx" : 0.999, "move_qy" : 0.023, "move_qz" : 0.026, "move_qw" : 0.001}
                    grasp_params = {"x_prep": 0.406, "y_prep": -0.313, "z_prep": 0.25, "qx_prep": 0.999, "qy_prep": 0.023, "qz_prep": 0.026, "qw_prep": 0.001,
                            "x_grasp": 0.406, "y_grasp": -0.313, "z_grasp": 0.2, "qx_grasp": 0.999, "qy_grasp": 0.023, "qz_grasp": 0.026, "qw_grasp": 0.001}
                    success = True
                    print("✅ Perceived spoon, moving to target point.")
                    continue
                elif target == "soup pot":
                    move_params = {"move_x" : 0.6, "move_y" : -0.3, "move_z" : 0.4, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
                    success = True
                    print("✅ Perceived soup pot, moving to target point.")
                    continue
                        
                else:  
                    print(f"Perceiving target: {target}")
                    
                    if_find_r, response_r, center_world_points_r, all_world_points_r = get_cam_world_points(
                    target,
                    rgb_path= r_img_path,
                    depth_path= r_depth_path,
                    pixels_to_world_func = pixels_to_world_right,
                    )

                    # if_find_r, response_r, center_world_points_r, all_world_points_r = None, None, None, None
                    
           
                    if_find_l, response_l, center_world_points_l, all_world_points_l = get_cam_world_points(
                        target,
                        rgb_path= l_img_path,
                        depth_path= l_depth_path,
                        pixels_to_world_func = pixels_to_world_left,
                    )

                    # if_find_l, response_l, center_world_points_l, all_world_points_l = None, None, None, None


                    if all_world_points_r is not None and all_world_points_l is not None:
                        print(f"World point in right camera: {all_world_points_r}, in left camera: {all_world_points_l}")                
                        # 合并点云举例
                        all_points = list(all_world_points_r) + list(all_world_points_l)
                        center_world_points = (center_world_points_l + center_world_points_r)/2
                        success = True
                        
                    elif all_world_points_r is not None and all_world_points_l is None:
                        print(f"World point in right camera: {all_world_points_r}, in left camera: None")
                        all_points = list(all_world_points_r)
                        center_world_points = center_world_points_r
                        success = True
                        
                    elif all_world_points_r is None and all_world_points_l is not None:
                        print(f"World point in right camera: None, in left camera: {all_world_points_l}")
                        all_points = list(all_world_points_l)
                        center_world_points = center_world_points_l
                        success = True
                    else:
                        print("❌ Failed to perceive target points in both cameras.")
                        play_text_to_speech("Sorry, I can't find that. Please try again.", language='en')

                        success = False
                        continue
                    
                
                    # 计算质心
                    all_points_arr = np.array(all_points)
                    
                    all_points_arr = all_points_arr[~np.isnan(all_points_arr).any(axis=1)]
                    if len(all_points_arr) == 0:
                        print("❌ All points are invalid (contain NaNs)")
                        success = False


                    # 计算中心点
                    center_world_points = center_world_points[0]
                    print ("center_world_points:", center_world_points)

                    # 计算质心
                    target_center_point = all_points_arr.mean(axis=0)
                    print("质心：", target_center_point)
                    
                    # 计算z轴方向上的最高点
                    max_z_index = np.argmax(all_points_arr[:,2])
                    target_max_z_point = all_points_arr[max_z_index]
                    print("z轴方向上的最高点：", target_max_z_point)
                    
                    # 计算 move_target_point
                    move_target_point = center_world_points.copy()
                    move_target_point[2] += 0.3  # 提升0.3米

                    print("移动目标点：", move_target_point)

                    move_params["move_x"] = float(move_target_point[0])
                    move_params["move_y"] = float(move_target_point[1])
                    move_params["move_z"] = float(move_target_point[2])
                    move_params["move_qx"] = 1.0
                    move_params["move_qy"] = 0.0
                    move_params["move_qz"] = 0.0
                    move_params["move_qw"] = 0.0
                    
                    # move_params = {"move_x" : 0.5, "move_y" : 0.6, "move_z" : 0.4, "move_qx" : 1.0, "move_qy" : 0.0, "move_qz" : 0.0, "move_qw" : 0.0}
                    # success = True
                
            elif act_type == "move":
                success = False  # 重置成功状态
                if target is  grasped_thing:
                    continue  
                current_position = [0.0, 0.0, 0.0]  # 初始化当前位姿

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
                                "z": float(current_position[2] + 0.3),  # 提升0.3米
                                "qx": 1.0,
                                "qy": 0.0,
                                "qz": 0.0,
                                "qw": 0.0,
                            }
                        )
                    except ValueError as e:
                        print(e)
                    # time.sleep(3)  # 等待服务调用完成
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
                # time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Move action executed successfully.")
                        break
                    else:
                        print("❌ Move action failed, retrying...")

            elif act_type == "grasp_flavoring":
                success = False  # 重置成功状态
                if target is  grasped_thing:
                    continue 
                
                print("execute grasp action")
                # time.sleep(5)
                # success = True
                
                
                ##TODO##  
                # if world_points_r is not None or world_points_l is not None:
                #     merged_points, trans = merge_points_icp(world_points_r, world_points_l, threshold=0.02, visualize=True)
                # elif world_points_r is not None and world_points_l is None:
                #     merged_points = world_points_r
                # elif world_points_r is None and world_points_l is not None:
                #     merged_points = world_points_l
                # else:
                #     print("❌ Failed to perceive target points in both cameras.")
                #     # maybe other method to determine the grasp strategy
                # now we have world_points of target , which is a list of tuples, each tuple is a point in world coordinates
                # put the value into gf_params by grasp strategy with points cloud or something else
                if target != "spoon": 
                    grasp_params = {"x_prep": 0.5, "y_prep": 0.6, "z_prep": 0.3, "qx_prep": 1.0, "qy_prep": 0.0, "qz_prep": 0.0, "qw_prep": 0.0,
                                "x_grasp": 0.5, "y_grasp": 0.6, "z_grasp": 0.2, "qx_grasp": 1.0, "qy_grasp": 0.0, "qz_grasp": 0.0, "qw_grasp": 0.0}
                
                
                try:
                    # 检查所有值
                    if any(v is None for v in grasp_params.values()):
                        raise ValueError("Missing grasp parameters.")
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
                time.sleep(3)  # 等待服务调用完成   
                while True:
                    if success:
                        print("✅ Grasp flavoring action executed successfully.")
                        grasped_thing = target
                        break
                    else:
                        print("❌ Grasp flavoring action failed, retrying...")

            elif act_type == "grasp_otherthings":
                success = False  # 重置成功状态
                if target is  grasped_thing:
                    continue 
                print("execute grasp action")
                # time.sleep(5)
                # success = True
                
                
                ##TODO## 
                
                # if world_points_r is not None or world_points_l is not None:
                #     merged_points, trans = merge_points_icp(world_points_r, world_points_l, threshold=0.02, visualize=True)
                # elif world_points_r is not None and world_points_l is None:
                #     merged_points = world_points_r
                # elif world_points_r is None and world_points_l is not None:
                #     merged_points = world_points_l
                # else:
                #     print("❌ Failed to perceive target points in both cameras.")
                #     # maybe other method to determine the grasp strategy
                    
                # now we have world_points of target , which is a list of tuples, each tuple is a point in world coordinates 
                # put the value into go_params by grasp strategy with points cloud or something else
                if target != "spoon": 
                    grasp_params = {"x_prep": 0.5, "y_prep": 0.6, "z_prep": 0.3, "qx_prep": 1.0, "qy_prep": 0.0, "qz_prep": 0.0, "qw_prep": 0.0,
                                "x_grasp": 0.5, "y_grasp": 0.6, "z_grasp": 0.2, "qx_grasp": 1.0, "qy_grasp": 0.0, "qz_grasp": 0.0, "qw_grasp": 0.0}
                
                
                
                try:
                    # 检查所有 go_params 的值
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
                    
                    
                time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Grasp other things action executed successfully.")
                        grasped_thing = target
                        break
                    else:
                        print("❌ Grasp other things action failed, retrying...")
                            
            elif act_type == "stir":
                success = False  # 重置成功状态
                print("execute stir action")
                # time.sleep(5)
                # success = True
                try:
                    if stir_time is None:
                        raise ValueError("Missing stir time.")
                    success = call_ros2_service("/stir_service", "action_interfaces/srv/Stir", {
                        "center_x": 0.6,
                        "center_y": -0.3,
                        "center_z": 0.4,
                        "radius": 0.08,
                        "start_angle_deg": 0.0,
                        "move_down_offset": 0.1,
                        "speed": 0.5,
                        "stir_time": stir_time
                    })
                except ValueError as e:
                    print(e)
                time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Stir action executed successfully.")
                        break
                    else:
                        print("❌ Stir action failed, retrying...")     
                
            elif act_type == "reset":
                success = False  # 重置成功状态
                print("execute reset action")
                # time.sleep(5)
                # success = True
                success = call_ros2_service("/reset_service", "std_srvs/srv/Trigger", {})
                
                time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Reset action executed successfully.")
                        break
                    else:
                        print("❌ Reset action failed, retrying...")

            elif act_type == "add":
                success = False  # 重置成功状态
                print("execute add action")
                # time.sleep(5)
                # success = True
                
                try:
                    if add_times is None:
                        raise ValueError("Missing add times.")
                    success = call_ros2_service("/add_service", "action_interfaces/srv/Add", 
                        {"times": add_times})
                except ValueError as e:
                    print(e)
                    
                time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Add action executed successfully.")
                        break
                    else:
                        print("❌ Add action failed, retrying...")
                
            elif act_type == "return_back":
                success = False  # 重置成功状态
                print("execute back_move action, moving back to original point")
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
                    # 检查 rb_params 是否有未赋值参数
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
                time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Return back action executed successfully.")
                        break
                    else:
                        print("❌ Return back action failed, retrying...")
                        
            elif act_type == "open":
                success = False  # 重置成功状态
                print("execute open action")
                success = call_ros2_service("/open_service", "std_srvs/srv/Trigger", {})  
                time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Open action executed successfully.")
                        break
                    else:
                        print("❌ Open action failed, retrying...")

            elif act_type == "close":
                success = False  # 重置成功状态
                print("execute close action")
                success = call_ros2_service("/close_service", "std_srvs/srv/Trigger", {})
                time.sleep(3)  # 等待服务调用完成
                while True:
                    if success:
                        print("✅ Close action executed successfully.")
                        break
                    else:
                        print("❌ Close action failed, retrying...")
            
            else:
                print(f"⚠️ Unknown action type: {act_type}")
                success = False

            time.sleep(0.5)  # 可选延迟
        
        except Exception as e:
            print("❌ Exception inside execute_action_sequence:")
            traceback.print_exc()
            raise
        print("✅ Action sequence completed.")

# ✅ 测试用例：手动构造动作序列
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



# # 举例 your_msgs/srv/Move.srv
# string target
# ---
# bool success
# string message

# 服务端必须返回：
# return Move.Response(success=True, message="Moved to apple.")