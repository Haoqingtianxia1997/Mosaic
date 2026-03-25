import cv2
from matplotlib import text
import rclpy
from rclpy.node import Node
from action_interfaces.msg import FileStatus
from action_interfaces.msg import Labels
import os
import sys
import json
import argparse
from pathlib import Path
from intention_utils.intention import Intention
HIGH_LEVEL_PATH = os.path.abspath(os.path.join(__file__, "../../../../../high_level/src"))
if HIGH_LEVEL_PATH not in sys.path:
    sys.path.append(HIGH_LEVEL_PATH)
from transcribe.tts import play_text_to_speech
from transcribe.stt import VoiceTranscriber

from mistral_ai.mistral import Mistralmodel
from mistral_ai.llm import run_mistral_llm_direct


class offline_intention_llm:
    def __init__(self, participant_code="unknown"):
        self.client = Mistralmodel()
        self.intention = Intention()
        self.participant_code = participant_code
        
        CUR_DIR = os.path.dirname(os.path.abspath(__file__))

        self.saved_intention_input_root = os.path.abspath(
            os.path.join(CUR_DIR, '../../../../manipulation_ws/saved_intention_data')
        )
        self.saved_intention_input_path = self._resolve_input_dir(self.saved_intention_input_root)

        self.file_path = os.path.abspath(
            os.path.join(CUR_DIR, '../../../../high_level/src/transcribe/transcription.txt')
        )

        print('Offline Intention LLM has been started.')
        print(f'participant_code: {self.participant_code}')
        print(f'input folder: {self.saved_intention_input_path}')

    def _resolve_input_dir(self, root_dir):
        candidates = [
            os.path.join(root_dir, f"{self.participant_code}_folder"),
            os.path.join(root_dir, self.participant_code),
            root_dir,
        ]
        for path in candidates:
            if os.path.isdir(path):
                return path
        raise FileNotFoundError(f"No valid input directory found under: {root_dir}")

    def _to_output_str(self, value):
        if value is None:
            return "None"
        if isinstance(value, list):
            return ", ".join(map(str, value)) if value else "None"
        text = str(value).strip()
        return text if text else "None"

    def _load_selected_input_json(self, selected_json_name=None):
        if selected_json_name:
            selected_path = os.path.join(self.saved_intention_input_path, selected_json_name)
            if not os.path.isfile(selected_path):
                raise FileNotFoundError(f"JSON not found: {selected_path}")
        else:
            json_files = [
                os.path.join(self.saved_intention_input_path, name)
                for name in os.listdir(self.saved_intention_input_path)
                if name.endswith('.json')
            ]
            if not json_files:
                raise FileNotFoundError(
                    f"No JSON files found in: {self.saved_intention_input_path}"
                )
            selected_path = max(json_files, key=os.path.getmtime)

        with open(selected_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # New format stores fields under intention_llm_input; keep old-format fallback.
        input_data = data.get('intention_llm_input', data)

        cmd_str = self._to_output_str(input_data.get('speech_command'))
        gesture_str = self._to_output_str(input_data.get('gesture_label'))
        gaze_str = self._to_output_str(input_data.get('gaze_label'))
        scenario_labels_str = self._to_output_str(input_data.get('scenario_labels'))
        print(f"Loaded intention input JSON: {selected_path}")
        return cmd_str, gesture_str, gaze_str, scenario_labels_str

    
    def ablation_process(self, selected_json_name=None, use_gesture_label=True, use_gaze_label=True):
            cmd_str, gesture_str, gaze_str, scenario_labels_str = self._load_selected_input_json(
                selected_json_name
            )

            if not use_gesture_label:
                gesture_str = "None"
            if not use_gaze_label:
                gaze_str = "None"

            input = (
                f"I have a speech command: {cmd_str}, "
                f"gesture label: {gesture_str} and "
                f"gaze label: {gaze_str}."
                f"scenario labels: {scenario_labels_str}."
            )

            response, audio_response, content, json_blocks = run_mistral_llm_direct(
                input,
                self.client,
                verbose=False,
            )

            if audio_response:
                play_text_to_speech(audio_response, language='en')

            if response != "":
                with open(self.file_path, 'w', encoding='utf-8') as f:
                    f.write(response)
                    
            print(f"Intention input: {input}")
            print(f"=====: {response}, Content: {content}, json blocks: {json_blocks}")
            
            




def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("selected_json_name", nargs="?", default=None)
    parser.add_argument("--participant_code", type=str, default="unknown")
    parser.add_argument("--no-gesture", action="store_true", help="Do not use gesture label")
    parser.add_argument("--no-gaze", action="store_true", help="Do not use gaze label")
    cli_args = parser.parse_args(args=args)

    test = offline_intention_llm(participant_code=cli_args.participant_code)
    test.ablation_process(
        selected_json_name=cli_args.selected_json_name,
        use_gesture_label=not cli_args.no_gesture,
        use_gaze_label=not cli_args.no_gaze,
    )

if __name__ == '__main__':
    main()