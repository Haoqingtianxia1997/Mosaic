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
    transcribed_text = get_full_text("./src/transcribe/speech.txt")

    subtasks = client.chat_with_text(
        transcribed_text,
        system_prompt=system_prompt,
        example=example,
        assistant_prompt=assistant_prompt
    )

    subtasks = re.sub(r':\s*None', ': ""', str(subtasks))

    # safe extraction method
    response, json_blocks = safe_extract_json_and_response_for_llm(str(subtasks))

    if not response:
        print("❌ No response from LLM. Please try again.")
        return False

    # Save response text
    Path("./src/mistral_ai/scripts/llm_script.txt").write_text(response, encoding="utf-8")

    print("🤖 LLM Response:")
    print(">>> Subtask list:\n", response)
    print(">>> JSON:\n", json.dumps(json_blocks[0], indent=2, ensure_ascii=False) if json_blocks else "None")

    # Save JSON action structure
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
        print(f"Attempt {i+1}, LLM output: {subtasks}")
        if subtasks:  # If there is content, continue with the next logic
            break
        print(f"LLM did not return content, retrying {i+1} time(s), waiting {wait_sec} seconds...")
        time.sleep(wait_sec)
    else:
        raise RuntimeError("LLM did not return content, exceeded maximum retry attempts")

    subtasks = re.sub(r':\s*None', ': ""', str(subtasks))

    # Safe extraction method
    response, content, json_blocks = safe_extract_json_and_response_for_intention_llm(str(subtasks))

    print("🤖 LLM Response:")
    print(">>> Subtask list:\n", response)
    print(">>> Content:\n", content)
    print(">>> JSON:\n", json.dumps(json_blocks[0], indent=2, ensure_ascii=False) if json_blocks else "None")

    return response, content, json_blocks

# if __name__ == "__main__":
#     run_mistral_llm()