#!/usr/bin/env python3
"""
test_mode_control.py – switch test mode for all participant data scripts
(1.main.py, bag_record.py, intention_llm.py) from one terminal.

  test       → saved data names get the test_ prefix
  test_over  → back to normal naming
  status     → show current mode
  q          → quit (scripts keep the last state)

Publishes std_msgs/Bool on /test_mode with latched QoS, so scripts started
later still get the current state. Starts in normal mode.

Usage:
  python3 src/action/action/test_mode_control.py
"""

import sys
from pathlib import Path

import rclpy
from std_msgs.msg import Bool

HIGH_LEVEL_PATH = str((Path(__file__).resolve().parent / "../../../../high_level/src").resolve())
if HIGH_LEVEL_PATH not in sys.path:
    sys.path.append(HIGH_LEVEL_PATH)
import test_mode


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("test_mode_control")
    pub = node.create_publisher(Bool, test_mode.TOPIC, test_mode.qos())

    active = False
    pub.publish(Bool(data=active))
    print("Normal mode. Commands: test | test_over | status | q")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "test":
                active = True
            elif cmd == "test_over":
                active = False
            elif cmd == "status":
                pass
            elif cmd in ("q", "quit", "exit"):
                break
            elif cmd:
                print("Unknown command, use: test | test_over | status | q")
                continue
            else:
                continue
            if cmd != "status":
                pub.publish(Bool(data=active))
            print("🧪 TEST mode (test_ prefix)" if active else "✅ Normal mode")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
