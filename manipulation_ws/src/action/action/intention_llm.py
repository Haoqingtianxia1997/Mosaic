import cv2
from matplotlib import text
import rclpy
from rclpy.node import Node
from action_interfaces.msg import FileStatus
from action_interfaces.msg import Labels
import os
import sys
import json
from datetime import datetime
from pathlib import Path
from intention_utils.intention import Intention
HIGH_LEVEL_PATH = os.path.abspath(os.path.join(__file__, "../../../../../high_level/src"))
if HIGH_LEVEL_PATH not in sys.path:
    sys.path.append(HIGH_LEVEL_PATH)
from transcribe.tts import play_text_to_speech
from transcribe.stt import VoiceTranscriber

from mistral_ai.mistral import Mistralmodel
from mistral_ai.llm import run_mistral_llm_direct

# def ask_label_tts(labels, transcriber):
#     labels = list(set(labels))
#     label_str = ", ".join(labels)
#     if len(labels) == 0:
#         last_query_result =""
#     elif len(labels) == 1:
#         tts_text = f"Are you looking for {label_str}?"
#         play_text_to_speech(tts_text, language='en')
#         stt_text = transcriber.auto_record_and_transcribe(5)
#         print(f"📝 STT Result: {stt_text}")


#         if stt_text and ("yes" in stt_text.lower() or
#                         labels[0].lower() in stt_text.lower()):
#             play_text_to_speech(
#                 "OK！",
#                 language='en'
#             )
#             last_query_result = f'please give me "{labels[0]}"'
#         else:
#             last_query_result =""

#     else:
#         tts_text = f"There are {label_str}. What do you want?"
#         play_text_to_speech(tts_text, language='en')
#         stt_text = transcriber.auto_record_and_transcribe(5)
#         print(f"📝 STT Result: {stt_text}")

        # found = ""
        # if stt_text:
        #     stt_lower = stt_text.lower()
        #     for item in labels:
        #         if item.lower() in stt_lower:
        #             found = item
        #             break
        # if found:
        #     last_query_result = f'please give me "{found}"'
            
        #     play_text_to_speech(
        #         "OK！",
        #         language='en'
        #     )
        # else:
        #     last_query_result = ""
        
    #     last_query_result = stt_text if stt_text else ""
        
    #     play_text_to_speech(
    #         "OK！",
    #         language='en'
    #     )
    
    # print(last_query_result)
    # return last_query_result

class IntentionLLM(Node):
    def __init__(self):
        super().__init__('intention_llm')

        # self.transcriber = VoiceTranscriber()
        self.client = Mistralmodel()
        self.intention = Intention()
        
        CUR_DIR = os.path.dirname(os.path.abspath(__file__))
       
        self.r_rgb_path = os.path.abspath(
            os.path.join(CUR_DIR, '../../../../manipulation_ws/saved_images/r_rgb.png')
        )
        
        self.scenario_img_path = os.path.abspath(
            os.path.join(CUR_DIR, '../../../../manipulation_ws/saved_images/r_scenario.png')
        )
        
        self.saved_intention_input_path = os.path.abspath(
            os.path.join(CUR_DIR, '../../../../manipulation_ws/saved_intention_input')
        )
        os.makedirs(self.saved_intention_input_path, exist_ok=True)
        
        self.file_path = os.path.abspath(
            os.path.join(CUR_DIR, '../../../../high_level/src/transcribe/transcription.txt')
        )


        self.file_status_sub = self.create_subscription(
            FileStatus,
            'file_status',
            self.file_status_cb,
            1
        )
        self.speech_changed = False
        self.new_file_content = None

        self.label_sub = self.create_subscription(
            Labels,
            'label_output',
            self.label_cb,
            1
        )
        self.latest_gesture_labels = None
        self.latest_gaze_labels = None
        self.all_labels = None

        self.gesture_history = []  # Store the latest 50 gesture_labels
        self.gaze_history = []     # Store the latest 50 gaze_labels
        self.max_history_size = 50
        
        self.get_logger().info('Intention LLM Node has been started.')

    def _save_output_json(self, output_text, cmd_str, gesture_str, gaze_str, scenario_labels_str):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        payload = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "speech_command": cmd_str,
            "gesture_label": gesture_str,
            "gaze_label": gaze_str,
            "scenario_labels": scenario_labels_str,
            "output": output_text,
        }
        file_path = os.path.join(self.saved_intention_input_path, f"intention_input_{ts}.json")
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f"Saved intention input JSON: {file_path}")

    def label_cb(self, msg):
        # Add to history
        self.gesture_history.append(msg.gesture_labels)
        self.gaze_history.append(msg.gaze_labels)

        # Keep history size within limit
        if len(self.gesture_history) > self.max_history_size:
            self.gesture_history.pop(0)
        if len(self.gaze_history) > self.max_history_size:
            self.gaze_history.pop(0)

        # Find the latest non-empty labels from history
        self.latest_gesture_labels = self._find_latest_non_empty(self.gesture_history)
        self.latest_gaze_labels = self._find_latest_non_empty(self.gaze_history)
        
        self.get_logger().info(f"Received gesture labels: {msg.gesture_labels}, Received gaze labels: {msg.gaze_labels}")
        self.get_logger().info(f"Latest non-empty gesture labels: {self.latest_gesture_labels}, Latest non-empty gaze labels: {self.latest_gaze_labels}")
    
    def _find_latest_non_empty(self, history_list):
        """Find the latest non-empty labels from history"""
        # Start searching from the latest
        for labels in reversed(history_list):
            if labels and len(labels) > 0:  # Check if non-empty and not empty list
                return labels
        return None

    def file_status_cb(self, msg):
        self.speech_changed = msg.changed
        self.new_file_content = msg.content
        print(f"File status changed: {self.speech_changed}, New content: {self.new_file_content}")
        if self.speech_changed:
            self.get_logger().info(f"File changed: {self.speech_changed}, Content: {self.new_file_content}")


        if self.speech_changed == True and self.new_file_content is not None and self.new_file_content != "": #and self.latest_gaze_labels is not None and self.latest_gesture_labels is not None
            cmd_str = self.new_file_content if self.new_file_content else "None"
            gesture_str = ", ".join(self.latest_gesture_labels) if self.latest_gesture_labels else "None"
            gaze_str = ", ".join(self.latest_gaze_labels) if self.latest_gaze_labels else "None"
            
            
            r_rgb = cv2.imread(self.r_rgb_path)
            scenario_labels =self.intention.get_scenario_yolo_labels(r_rgb, self.scenario_img_path)
            scenario_labels_str = ", ".join(scenario_labels) if scenario_labels else "None"
            
            output = (
                f"I have a speech command: {cmd_str}, "
                f"gesture label: {gesture_str} and "
                f"gaze label: {gaze_str}."
                f"scenario labels: {scenario_labels_str}."
            )
            self._save_output_json(output, cmd_str, gesture_str, gaze_str, scenario_labels_str)
            
            
            response, audio_response, content, json_blocks = run_mistral_llm_direct(
                output,
                self.client,
            )
            
            # Path("./src/mistral_ai/scripts/llm_script.txt").write_text(audio_response, encoding="utf-8")
            
            if audio_response:
                play_text_to_speech(audio_response, language='en')
                
            if response != "":
                with open(self.file_path, 'w', encoding='utf-8') as f:
                    f.write(response)

            self.latest_gesture_labels, self.latest_gaze_labels, self.new_file_content = None, None, None
        else:
            response, content, json_blocks = "", "", ""
            output = None   
            # elif self.latest_gaze_labels is not None and self.latest_gesture_labels is not None :
            #     self.all_labels = list(set(self.latest_gesture_labels + self.latest_gaze_labels))
            #     if len(self.all_labels) > 0:
            #         cmd = ask_label_tts(self.all_labels, self.transcriber)
            #         cmd_str = cmd if cmd else "None"
            #         gesture_str = ", ".join(self.latest_gesture_labels) if self.latest_gesture_labels else "None"
            #         gaze_str = ", ".join(self.latest_gaze_labels) if self.latest_gaze_labels else "None"

            #         output = (
            #             f"I have a speech command: '{cmd_str}', "
            #             f"gesture label: '{gesture_str}' and "
            #             f"gaze label: '{gaze_str}'."
            #         )
            #         response, content, json_blocks = run_mistral_llm_direct(
            #             output,
            #             self.client,
            #         )

            #         if response != "":
            #             with open(self.file_path, 'w', encoding='utf-8') as f:
            #               f.write(response)
            #         self.latest_gesture_labels, self.latest_gaze_labels, cmd_str = None, None, None
            #     else:
            #         response = None
            #         content = None
            #         json_blocks = None
            #         output = None
            print(f"=====: {response}, Content: {content}, json blocks: {json_blocks}")
            print(f"📝 Intention Output: {output}")
            




def main(args=None):
    rclpy.init(args=args)
    node = IntentionLLM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
