import threading
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from transcribe.stt import run_stt, NEW_TEXT_EVENT, VoiceTranscriber
from transcribe.tts import run_tts
from mistral_ai.llm import run_mistral_llm
from execute.actions import execute_action_sequence
from mistral_ai.vlm import run_mistral_vlm
from src.mistral_ai.mistral import Mistralmodel
from src.execute.element_action import *

SPEECH_FILE = "src/transcribe/speech.txt"
TRANS_FILE = "src/transcribe/transcription.txt"
VLM_FILE = "src/mistral_ai/scripts/vlm_script.txt"
VLM_JSON_FILE = "src/mistral_ai/scripts/vlm_script.json"
LLM_FILE = "src/mistral_ai/scripts/llm_script.txt"
LLM_JSON_FILE = "src/mistral_ai/scripts/llm_script.json"
import json

transcriber=VoiceTranscriber()
executor = ActionExecutor()

def stt_thread():
    # run in background
    run_stt(transcriber=transcriber)

# def intention_dection_thread():
#     intention_predict("yolov8x-oiv7.pt")




if __name__ == "__main__":
    # clear transcription.txt
    with open(SPEECH_FILE, "w", encoding="utf-8") as f:
        f.write("")
    with open(TRANS_FILE, "w", encoding="utf-8") as f:
        f.write("")
    with open(VLM_FILE, "w", encoding="utf-8") as f:
        f.write("")
    with open(VLM_JSON_FILE , "w", encoding="utf-8") as f:
        f.write("")
    with open(LLM_FILE, "w", encoding="utf-8") as f:
        f.write("")
    with open(LLM_JSON_FILE, "w", encoding="utf-8") as f:
        f.write("")

    # 1. start stt thread
    threading.Thread(target=stt_thread, daemon=True).start()
    # 2. start intention detection thread
    # threading.Thread(target=intention_detection_thread, daemon=True).start()

    print("🟢 New task thread started.")
    last_text = ""
    print("🟢 STT thread started. Waiting for new speech...")

    # 3. start Mistral model
    llm_client = Mistralmodel()
    vlm_client = Mistralmodel()

    while True:

        # # 2. wait for new recording to complete
        # NEW_TEXT_EVENT.wait()
        # NEW_TEXT_EVENT.clear()
        # 3. read latest text
        try:
            with open(SPEECH_FILE, "r", encoding="utf-8") as f:
                text = f.read().strip()
        except FileNotFoundError:
            continue

        if not text:
            continue
        # 4. ignore if same as last time

        if text == last_text:
            continue
        
        last_text = text
        if_success = run_mistral_llm(llm_client)
        # run_tts(LLM_FILE)

        if not if_success:
            print("❌ Planning LLM processing failed. Please try again.")
            continue

        # 5. read JSON action list and execute
        try:
            with open(LLM_JSON_FILE, "r", encoding="utf-8") as f:
                llm_data = json.load(f)
                actions = llm_data.get("actions", [])
                if actions:
                    print(f"🦾 Executing {len(actions)} actions...")
                    execute_action_sequence(actions, vlm_client, executor=executor)
                else:
                    print("ℹ️ No actions to execute.")
        except Exception as e:
            print(f"❌ Failed to load or execute actions: {e}")
