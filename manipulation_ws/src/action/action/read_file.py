import rclpy
from rclpy.node import Node
from action_interfaces.msg import FileStatus
import time
import json
from datetime import datetime

def load_transcript_entry(input_path: str) -> tuple[str, str]:
    """
    Load transcript file and return (timestamp, text).
    Supports new JSON format and legacy plain-text format.
    """
    try:
        with open(input_path, "r", encoding="utf-8") as f:
            raw = f.read().strip()
    except FileNotFoundError:
        return "", ""

    if not raw:
        return "", ""

    # New format: {"timestamp": "...", "text": "..."}
    try:
        data = json.loads(raw)
        if isinstance(data, dict):
            timestamp = str(data.get("timestamp", "")).strip()
            text = str(data.get("text", "")).strip()
            return timestamp, text
    except json.JSONDecodeError:
        pass

    # Legacy format: "[timestamp] text"
    if raw.startswith("[") and "]" in raw:
        close_idx = raw.find("]")
        timestamp = raw[1:close_idx].strip()
        text = raw[close_idx + 1:].strip()
        return timestamp, text

    # Legacy format: plain text only
    return "", raw

class ReadFile(Node):
    def __init__(self):
        super().__init__('read_file')
        self.file_path = '../high_level/src/transcribe/speech.txt'
        self.publisher = self.create_publisher(FileStatus, 'file_status', 10)
        timer_period = 0.06 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.first_ts_epoch = None
        self.first_now=None
        self.if_first = False
        
    def _timestamp_to_epoch(self, timestamp: str) -> float | None:
        """Convert ISO timestamp text to epoch seconds."""
        if not timestamp:
            return None
        try:
            # Support trailing 'Z' form if present.
            normalized = timestamp.replace('Z', '+00:00')
            return datetime.fromisoformat(normalized).timestamp()
        except Exception:
            return None
    
    def timer_callback(self):
        now = time.time()
        try:
            timestamp, content = load_transcript_entry(self.file_path)
        except Exception as e:
            self.get_logger().warning(f'Cannot read file: {e}')
            timestamp = ''
            content = ''

        msg_changed = False
        ts_epoch = self._timestamp_to_epoch(timestamp)
        if ts_epoch is not None:
            if not self.if_first:
                self.first_ts_epoch = ts_epoch
                self.first_now = now
                self.if_first = True
            ts_epoch -= self.first_ts_epoch
            now -= self.first_now
            if now  > ts_epoch:
                dt = now - ts_epoch
                if dt < 0.1:
                    msg_changed = True
        

        msg = FileStatus()
        msg.changed = msg_changed
        msg.content = f"{content}+{ts_epoch}"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReadFile()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()