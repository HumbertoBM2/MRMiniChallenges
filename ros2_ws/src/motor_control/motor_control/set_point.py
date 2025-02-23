# Import libraries
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')
        # Retrieve sine wave parameters
        self.amplitude = 2.0
        self.omega = 1.0
        # Change the topic to 'set_point' instead of 'motor_input_u'
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_cb)
        # Create a message and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        # Print initialization in the terminal
        self.get_logger().info("SetPoint Node Started 🚀")

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        self.signal_publisher.publish(self.signal_msg)

def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()
    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()
