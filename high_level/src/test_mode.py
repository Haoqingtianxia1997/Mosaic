"""Test mode for scripts that save data into a participant folder.

State comes from the /test_mode topic (std_msgs/Bool), published by
manipulation_ws/src/action/action/test_mode_control.py in its own terminal.
While it is True, data saved by these scripts gets the `test_` prefix.
"""
import threading
import time

TOPIC = "/test_mode"
PREFIX = "test_"
_active = threading.Event()


def qos():
    """Latched QoS, so a script started later still gets the current state."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def prefix():
    """Return the file name prefix for data saved right now."""
    return PREFIX if _active.is_set() else ""


def _on_test_mode(msg):
    if msg.data == _active.is_set():
        return
    if msg.data:
        _active.set()
        print("🧪 Test mode ON: saved data names get the 'test_' prefix.")
    else:
        _active.clear()
        print("✅ Test mode OFF: saved data names back to normal.")


def attach(node):
    """Subscribe an existing rclpy node to the test mode topic."""
    from std_msgs.msg import Bool
    return node.create_subscription(Bool, TOPIC, _on_test_mode, qos())


def start_subscriber():
    """For scripts without a spinning node: subscribe in a daemon thread."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('test_mode_subscriber')
    attach(node)

    def _spin():
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.01)

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    print(f"📡 Subscribed to {TOPIC}")
    return thread
