import rclpy
from rclpy.node import Node
from action_interfaces.msg import FileStatus
import time

class ReadFile(Node):
    def __init__(self):
        super().__init__('read_file')
        self.file_path = '../high_level/src/transcribe/speech.txt'
        self.last_content = ''
        self.last_change_time = 0.0
        self.duration_true = 0.1 
        self.publisher = self.create_publisher(FileStatus, 'file_status', 10)
        timer_period = 0.2 
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        now = time.time()
        try:
            with open(self.file_path, 'r') as f:
                content = f.read()
        except Exception as e:
            self.get_logger().warning(f'Cannot read file: {e}')
            content = ''

        if content != self.last_content:
            self.last_change_time = now
            # self.get_logger().info('File content changed!')
        self.last_content = content

        msg = FileStatus()
        msg.changed = (now - self.last_change_time) < self.duration_true
        msg.content = content
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReadFile()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()