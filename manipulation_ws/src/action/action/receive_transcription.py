import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


OUTPUT_PATH = os.path.expanduser("~/mosaic/high_level/src/transcribe/speech.txt")


def save_transcript_entry(output_path: str, text: str, timestamp: str | None = None) -> dict:
    entry = {
        "timestamp": timestamp or datetime.now().isoformat(timespec="milliseconds"),
        "text": text,
    }
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(entry, f, ensure_ascii=False, indent=2)
    return entry


class TranscriptionReceiver(Node):
    def __init__(self):
        super().__init__("transcription_receiver")
        self.declare_parameter("output_path", OUTPUT_PATH)
        self._output_path = self.get_parameter("output_path").get_parameter_value().string_value

        self._sub = self.create_subscription(
            String,
            "/transcription",
            self._callback,
            10,
        )
        self.get_logger().info(f"Listening on /transcription, saving to {self._output_path}")

    def _callback(self, msg: String) -> None:
        entry = save_transcript_entry(self._output_path, msg.data)
        self.get_logger().info(f"Saved: {entry}")


def main(args=None):
    rclpy.init(args=args)
    node = TranscriptionReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
