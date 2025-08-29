from src.VLM_agent.OwlViT_FastSAM_SAM import find_object_central_pixel
from src.mistral_ai.vlm import run_mistral_vlm



def VLM_agent(user_prompt: str, image_path, name, client) -> list:
    """
    启动 VLM Agent，处理视觉任务。
    """  
    print("🟢 Starting VLM Agent...")
    if_find ,response, target_label, target_text = run_mistral_vlm(user_prompt, image_path, client)  # 调用 VLM 模型处理视觉任务
    
    if not if_find:
        print("❌ No target found at the moment.")
        return if_find, response, None, None, None


    # target_label, target_text = user_prompt, user_prompt
    # if_find =True 
    # response = ""

    target_prompt, box_center_point, seg_center_point, all_seg_points, bbox, score = find_object_central_pixel(target_label, target_text, image_path, True, False, name)  # 调用函数处理图像中的目标检测
    print(f"🔍 Detected target: {target_label}")
    print(f"📍 Target prompt: {target_prompt}")
    print(f"📏 Bounding box: {bbox}")
    print(f"🎯 Box center point: {box_center_point}")
    print(f"🎯 Segmentation center point: {seg_center_point}")
    print(f"📊 Detection score: {score}")

    print("✅ VLM Agent completed.")

    return if_find, response,  box_center_point, seg_center_point, all_seg_points