# Import libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PIDController(Node):

    def __init__(self):
        super().__init__('ctrl')
        # Declare parameters
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.5)
        self.declare_parameter('Kd', 0.02)
        self.declare_parameter('sample_time', 0.02)
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.sample_time = self.get_parameter('sample_time').value
        # Initialize variables to prevent attribute errors
        self.integral = 0.0
        self.prev_error = 0.0
        self.control_signal = 0.0
        self.set_point = 0.0  
        self.motor_speed = 0.0 
        # Subscribe to topics
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.motor_speed_sub = self.create_subscription(Float32, 'motor_speed_y', self.motor_speed_callback, 10)
        # Declare publisher
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        # Timer
        self.timer = self.create_timer(self.sample_time, self.control_loop)
        # Print initialization in the terminal
        self.get_logger().info("PID Controller Node Started 🚀")

    def set_point_callback(self, msg):
        self.set_point = msg.data

    def motor_speed_callback(self, msg):
        self.motor_speed = msg.data

    def control_loop(self):
        # Fetch updated parameters from rqt_reconfigure
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        error = self.set_point - self.motor_speed
        # Limit the integral term within a range to ensure anti-windup
        integral_limit = 5.0  
        self.integral += error * self.sample_time
        self.integral = max(min(self.integral, integral_limit), -integral_limit)
        derivative = (error - self.prev_error) / self.sample_time
        # PID equation
        raw_control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # Low-Pass Filter to Smooth Out Noise
        alpha = 0.05  
        self.control_signal = alpha * raw_control_signal + (1 - alpha) * self.control_signal
        # Publish the control signal in its corresponding topic
        motor_input_msg = Float32()
        motor_input_msg.data = self.control_signal
        self.motor_input_pub.publish(motor_input_msg)
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

# Execute Node
if __name__ == '__main__':
    main()
