// Include libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

// Class to generate a sine signal with a 10Hz frequency
class SignalGenerator : public rclcpp::Node {
public:
    SignalGenerator() : Node("signal_generator"), time_(0.0) {
        signal_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/signal", 10);
        time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/time", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&SignalGenerator::publish_signal, this));
    }

private:
    void publish_signal() {
        std_msgs::msg::Float32 signal_msg;
        signal_msg.data = std::sin(time_);
        std_msgs::msg::Float32 time_msg;
        time_msg.data = time_;
        signal_publisher_->publish(signal_msg); // Publish signal value to the topic
        time_publisher_->publish(time_msg); // Publish time value to the topic
        RCLCPP_INFO(this->get_logger(), "Time: %.2f, Signal: %.2f", time_, signal_msg.data); // Print the sine value at a given time to the terminal
        time_ += 0.1;  // Increase time (10Hz step)
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr signal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}
