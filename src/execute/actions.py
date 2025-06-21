import subprocess
import json
import time
from src.VLM_agent.agent import VLM_agent  

def call_ros2_service(service_name, service_type, args_dict):
    """
    调用 ROS 2 服务，通过 subprocess 调用 CLI，等待结果并返回是否成功。
    """
    arg_str = json.dumps(args_dict).replace('"', '\\"')
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

        # 检查是否包含 success: True
        if "success: true" in result.lower():
            return True
        else:
            print("❌ Service reported failure.")
            return False
    except subprocess.CalledProcessError as e:
        print("❌ Service call failed:")
        print(e.output)
        return False



def execute_action_sequence(actions):
    """
    串行执行动作序列，每一步等待其服务执行完且成功后才进行下一步。
    """
    target_point = None
    for i, action in enumerate(actions):
        print(f"\n▶️ Executing action {i+1}/{len(actions)}: {action}")
        act_type = action["type"]
        target = action["target"]
        params = action.get("parameters", {})
        

        if act_type == "perceive":
            print("execute perceive action")
            success, pixel_point = VLM_agent(target, image_path="images/example3.jpg")
            target_point = pixel_point  # 获取目标的像素点坐标
            time.sleep(5)
            if success:
                print(f"✅ Perceived target '{target}' at pixel point: {pixel_point}")

            # success = call_ros2_service("/robot/perceive", "your_msgs/srv/Perceive", {"target": target})
        elif act_type == "move":
            print("execute move action, moving to target point:", target_point)
            time.sleep(5)
            success = True
            # success = call_ros2_service("/robot/move", "your_msgs/srv/Move", {"target": target})
        elif act_type == "grasp":
            print("execute grasp action")
            time.sleep(5)
            success = True
            # success = call_ros2_service("/robot/grasp", "your_msgs/srv/Grasp", {"target": target})
        elif act_type == "stir":
            print("execute stir action")
            time.sleep(5)
            success = True
            # success = call_ros2_service("/robot/stir", "your_msgs/srv/Stir", {
            #     "target": target,
            #     "duration": params.get("duration", 5)
            # })
        elif act_type == "place":
            print("execute place action")
            time.sleep(5)
            success = True
            # success = call_ros2_service("/robot/place", "your_msgs/srv/Place", {"target": target})
        elif act_type == "reset":
            print("execute reset action")
            time.sleep(5)
            success = True
            # success = call_ros2_service("/robot/reset", "your_msgs/srv/Reset", {"target": target})
        elif act_type == "add":
            print("execute add action")
            time.sleep(5)
            success = True
            # success = call_ros2_service("/robot/add", "your_msgs/srv/Add", {
            #     "target": target,
            #     "amount": params.get("amount", "")
            # })
        elif act_type == "back_move":
            print("execute back_move action, moving back to original point:", target_point)
            time.sleep(5)
            success = True
            # success = call_ros2_service("/robot/back_move", "your_msgs/srv/back_move", {
            #     "target": target,
            #     "amount": params.get("amount", "")
            # })
        else:
            print(f"⚠️ Unknown action type: {act_type}")
            success = False

        # 检查服务是否成功
        if not success:
            print(f"⛔ Aborting action sequence due to failure at step {i+1}.")
            break

        time.sleep(0.5)  # 可选延迟
    
    print("✅ Action sequence completed.")

# ✅ 测试用例：手动构造动作序列
if __name__ == "__main__":
    actions = [
        {"type": "perceive", "target": "apple"},
        {"type": "move", "target": "apple"},
        {"type": "grasp", "target": "apple"},
        {"type": "move", "target": "plate"},
        {"type": "place", "target": "plate"},
        {"type": "reset", "target": "home"}
    ]

    execute_action_sequence(actions)



# # 举例 your_msgs/srv/Move.srv
# string target
# ---
# bool success
# string message

# 服务端必须返回：
# return Move.Response(success=True, message="Moved to apple.")