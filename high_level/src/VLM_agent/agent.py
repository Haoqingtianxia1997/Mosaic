from src.VLM_agent.OwlViT_FastSAM_SAM import find_object_central_pixel
from src.mistral_ai.vlm import run_mistral_vlm



def VLM_agent(user_prompt: str, image_path, name, client, segmenter=None) -> list:
    """
    Start the VLM agent to process the user's prompt and image.
    """  
    print("🟢 Starting VLM Agent...")
    # if_find ,response, target_label, target_text = run_mistral_vlm(user_prompt, image_path, client)  # Call VLM model to process visual tasks

    # if not if_find:
    #     print("❌ No target found at the moment.")
    #     return if_find, response, None, None, None


    # target_label, target_text = user_prompt, user_prompt
    # if_find =True 
    # response = ""

    target_prompt, box_center_point, seg_center_point, all_seg_points, bbox, score = find_object_central_pixel(user_prompt, user_prompt, image_path, True, False, name, segmenter)  # 调用函数处理图像中的目标检测
    print(f"🔍 Detected target: {user_prompt}")
    print(f"📍 Target prompt: {target_prompt}")
    print(f"📏 Bounding box: {bbox}")
    print(f"🎯 Box center point: {box_center_point}")
    print(f"🎯 Segmentation center point: {seg_center_point}")
    print(f"📊 Detection score: {score}")

    print("✅ VLM Agent completed.")

    return True, "",  box_center_point, seg_center_point, all_seg_points