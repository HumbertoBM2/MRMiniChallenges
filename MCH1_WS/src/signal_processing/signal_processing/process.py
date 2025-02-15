# Import libraries 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

# Processing the sine wave
class SignalProcessor(Node):
    def __init__(self):
        super().__init__('process')
        self.subscription_signal = self.create_subscription(Float32, '/signal', self.process_signal, 10)
        self.subscription_time = self.create_subscription(Float32, '/time', self.store_time, 10)
        self.processed_signal_publisher = self.create_publisher(Float32, '/proc_signal', 10)
        self.phase_shift = np.pi / 4  # Phase shift
        self.offset = 1.0  # Make the sine wave to always be positive
        self.amplitude_factor = 0.5  # Reduce amplitude by half
        self.time = 0.0

    def store_time(self, msg):
        self.time = msg.data

    def process_signal(self, msg):
        original_signal = msg.data

        # Apply phase shift, offset, and amplitude reduction
        shifted_signal = np.sin(self.time + self.phase_shift)  
        processed_signal = self.amplitude_factor * (shifted_signal + self.offset)  

        # Publish the processed signal to the topic
        self.processed_signal_publisher.publish(Float32(data=processed_signal))
        self.get_logger().info(f'Processed Signal: {processed_signal:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SignalProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
