import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PIDController(Node):
    def __init__(self):
        super().__init__('ctrl')

        # Declare and get parameters
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('Kd', 0.01)
        self.declare_parameter('sample_time', 0.02)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.sample_time = self.get_parameter('sample_time').value

        # Initialize PID variables
        self.integral = 0.0
        self.prev_error = 0.0

        # ROS 2 Subscribers and Publishers
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.motor_speed_sub = self.create_subscription(Float32, 'motor_speed_y', self.motor_speed_callback, 10)
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input_u', 10)

        # Timer
        self.timer = self.create_timer(self.sample_time, self.control_loop)

        # Variables
        self.set_point = 0.0
        self.motor_speed = 0.0

        self.get_logger().info("PID Controller Node Started 🚀")

    def set_point_callback(self, msg):
        self.set_point = msg.data

    def motor_speed_callback(self, msg):
        self.motor_speed = msg.data

    def control_loop(self):
        error = self.set_point - self.motor_speed
        self.integral += error * self.sample_time
        derivative = (error - self.prev_error) / self.sample_time

        # PID Control Equation
        control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Publish the control signal
        motor_input_msg = Float32()
        motor_input_msg.data = control_signal
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

if __name__ == '__main__':
    main()
