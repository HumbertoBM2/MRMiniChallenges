# Importing libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import time

# Create a node that generates a sinusodial signal using a 10Hz rate 
class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher = self.create_publisher(Float32, '/signal', 10)
        self.time_publisher = self.create_publisher(Float32, '/time', 10)
        self.timer = self.create_timer(0.1, self.publish_signal)  # 10Hz
        self.start_time = time.time()

    def publish_signal(self):
        current_time = time.time() - self.start_time
        signal_value = np.sin(current_time)
        # Publish the data in the topic
        self.signal_publisher.publish(Float32(data=signal_value))
        self.time_publisher.publish(Float32(data=current_time))
        # Print the value of the signal at a current time
        self.get_logger().info(f'Time: {current_time:.2f}, Signal: {signal_value:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
