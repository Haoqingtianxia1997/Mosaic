from src.mistral_ai.mistral import Mistralmodel
from mistral_ai.prompts.plan_prompt import system_prompt, example, assistant_prompt
from mistral_ai.prompts.intention_prompt import intention_system_prompt, intention_example, intention_assistant_prompt
from src.utils import get_last_text_line ,get_full_text, safe_extract_json_and_response_for_llm, safe_extract_json_and_response_for_intention_llm
import json
from pathlib import Path
from typing import Union, Any
import re
import time


def run_mistral_llm(client):
    # client = Mistralmodel()
    transcribed_text = get_full_text("./src/transcribe/transcription.txt")

    subtasks = client.chat_with_text(
        transcribed_text,
        system_prompt=system_prompt,
        example=example,
        assistant_prompt=assistant_prompt
    )

    # 新的安全提取方式
    response, json_blocks = safe_extract_json_and_response_for_llm(str(subtasks))

    if not response:
        print("❌ No response from LLM. Please try again.")
        return False

    # 保存 response 文本
    Path("./src/mistral_ai/scripts/llm_script.txt").write_text(response, encoding="utf-8")

    print("🤖 LLM Response:")
    print(">>> Subtask list:\n", response)
    print(">>> JSON:\n", json.dumps(json_blocks[0], indent=2, ensure_ascii=False) if json_blocks else "None")

    # 保存 JSON 动作结构
    if json_blocks:
        with open("./src/mistral_ai/scripts/llm_script.json", "w", encoding="utf-8") as jf:
            json.dump(json_blocks[0], jf, ensure_ascii=False, indent=2)
    
    return True


def run_mistral_llm_direct(text: Union[str, Any], client, max_retries=5, wait_sec=3):
    for i in range(max_retries):
        subtasks = client.chat_with_text(
            text,
            system_prompt=intention_system_prompt,
            example=intention_example,
            assistant_prompt=intention_assistant_prompt
        )
        print(f"第{i+1}次尝试，LLM返回内容：{subtasks}")
        if subtasks:  # 有内容就继续后面逻辑
            break
        print(f"LLM没返回内容，第{i+1}次重试，等待{wait_sec}秒...")
        time.sleep(wait_sec)
    else:
        raise RuntimeError("LLM一直没返回内容，超出最大重试次数")

    # 新的安全提取方式
    response, content, json_blocks = safe_extract_json_and_response_for_intention_llm(str(subtasks))

    print("🤖 LLM Response:")
    print(">>> Subtask list:\n", response)
    print(">>> Content:\n", content)
    print(">>> JSON:\n", json.dumps(json_blocks[0], indent=2, ensure_ascii=False) if json_blocks else "None")

    return response, content, json_blocks

# if __name__ == "__main__":
#     run_mistral_llm()