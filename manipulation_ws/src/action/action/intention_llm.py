import rclpy
from rclpy.node import Node
from action_interfaces.msg import FileStatus
from action_interfaces.msg import Labels
import os
import sys
HIGH_LEVEL_PATH = os.path.abspath(os.path.join(__file__, "../../../../../high_level/src"))
if HIGH_LEVEL_PATH not in sys.path:
    sys.path.append(HIGH_LEVEL_PATH)
from transcribe.tts import play_text_to_speech
from transcribe.stt import VoiceTranscriber

from mistral_ai.mistral import Mistralmodel
from mistral_ai.llm import run_mistral_llm_direct

def ask_label_tts(labels, transcriber):
    labels = list(set(labels))
    label_str = ", ".join(labels)
    if len(labels) == 0:
        last_query_result =""
    elif len(labels) == 1:
        tts_text = f"Are you looking for {label_str}?"
        play_text_to_speech(tts_text, language='en')
        stt_text = transcriber.auto_record_and_transcribe(5)
        print(f"📝 STT Result: {stt_text}")


        if stt_text and ("yes" in stt_text.lower() or
                        labels[0].lower() in stt_text.lower()):
            play_text_to_speech(
                "OK！",
                language='en'
            )
            last_query_result = f'please give me "{labels[0]}"'
        else:
            last_query_result =""

    else:
        tts_text = f"There are {label_str}. What do you want?"
        play_text_to_speech(tts_text, language='en')
        stt_text = transcriber.auto_record_and_transcribe(5)
        print(f"📝 STT Result: {stt_text}")

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
        
        last_query_result = stt_text if stt_text else ""
        
        play_text_to_speech(
            "OK！",
            language='en'
        )
    
    print(last_query_result)
    return last_query_result

class IntentionLLM(Node):
    def __init__(self):
        super().__init__('intention_llm')

        self.transcriber = VoiceTranscriber()
        self.client = Mistralmodel()

        self.file_status_sub = self.create_subscription(
            FileStatus,
            'file_status',
            self.file_status_cb,
            10
        )
        self.speech_changed = False
        self.new_file_content = None

        self.label_sub = self.create_subscription(
            Labels,
            'label_output',
            self.label_cb,
            10
        )
        self.latest_gesture_labels = None
        self.latest_gaze_labels = None
        self.all_labels = None
        self.get_logger().info('Intention LLM Node has been started.')

    def label_cb(self, msg):
        self.latest_gesture_labels = msg.gesture_labels
        self.latest_gaze_labels = msg.gaze_labels
        self.get_logger().info(f"Received gesture labels: {self.latest_gesture_labels}, Received gaze labels: {self.latest_gaze_labels}")

    def file_status_cb(self, msg):
        self.speech_changed = msg.changed
        self.new_file_content = msg.content
        self.get_logger().info(f"File changed: {self.speech_changed}, Content: {self.new_file_content}")
        

        if self.latest_gaze_labels is None and self.latest_gesture_labels is None:
            
            self.get_logger().warning("Labels not received yet, cannot process intention.")

        else:
            if self.speech_changed == True:
                cmd_str = self.new_file_content if self.new_file_content else "None"
                gesture_str = ", ".join(self.latest_gesture_labels) if self.latest_gesture_labels else "None"
                gaze_str = ", ".join(self.latest_gaze_labels) if self.latest_gaze_labels else "None"

                output = (
                    f"I have a speech command: {cmd_str}, "
                    f"gesture label: {gesture_str} and "
                    f"gaze label: {gaze_str}."
                )
                response, content, json_blocks = run_mistral_llm_direct(
                    output,
                    self.client,
                )

            else:
                self.all_labels = list(set(self.latest_gesture_labels + self.latest_gaze_labels))
                if len(self.all_labels) > 0:
                    cmd = ask_label_tts(self.all_labels, self.transcriber)
                    cmd_str = cmd if cmd else "None"
                    gesture_str = ", ".join(self.latest_gesture_labels) if self.latest_gesture_labels else "None"
                    gaze_str = ", ".join(self.latest_gaze_labels) if self.latest_gaze_labels else "None"

                    output = (
                        f"I have a speech command: '{cmd_str}', "
                        f"gesture label: '{gesture_str}' and "
                        f"gaze label: '{gaze_str}'."
                    )
                    response, content, json_blocks = run_mistral_llm_direct(
                        output,
                        self.client,
                    )

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