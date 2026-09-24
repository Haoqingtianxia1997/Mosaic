import threading
import time

# reset_switch state machine:
#   False --(reset_switch True received)--> True --(open/reset done, mark_reset_done)--> False
# While True, further True messages are ignored (e.g. a publisher running at -r 10).
_reset_requested = threading.Event()


def _on_reset_switch(msg):
    if msg.data and not _reset_requested.is_set():
        print("🔴 reset_switch received, remaining actions will be skipped.")
        _reset_requested.set()


def reset_requested():
    return _reset_requested.is_set()


def mark_reset_done():
    _reset_requested.clear()
    print("🟢 Reset done, waiting for next reset_switch.")


def _spin(topic_name):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('reset_switch_subscriber')
    node.create_subscription(Bool, topic_name, _on_reset_switch, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print(f"📡 Subscribed to {topic_name}")

    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        time.sleep(0.01)


def start_switch_subscriber(topic_name="/reset_switch"):
    """Subscribe to the reset switch topic in a daemon thread."""
    thread = threading.Thread(target=_spin, args=(topic_name,), daemon=True)
    thread.start()
    return thread
