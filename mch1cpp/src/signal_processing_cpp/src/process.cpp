// Include libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

// Class to process the sine signal
class SignalProcessor : public rclcpp::Node {
public:
    SignalProcessor() : Node("process"), time_(0.0), phase_shift_(M_PI / 4), amplitude_factor_(0.5), offset_(1.0) { // Select time shift factor, make the wave positive and redice its amplitude
        signal_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/signal", 10, std::bind(&SignalProcessor::process_signal, this, std::placeholders::_1));
        
        time_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/time", 10, std::bind(&SignalProcessor::store_time, this, std::placeholders::_1));

        processed_signal_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/proc_signal", 10);
    }

private:
    void store_time(const std_msgs::msg::Float32::SharedPtr msg) {
        time_ = msg->data;
    }

    void process_signal(const std_msgs::msg::Float32::SharedPtr msg) {
        double original_signal = msg->data;
        // Process the signal with the desired transformations
        double shifted_signal = std::sin(time_ + phase_shift_);
        double processed_signal = amplitude_factor_ * (shifted_signal + offset_);
        std_msgs::msg::Float32 processed_msg;
        processed_msg.data = processed_signal;
        processed_signal_publisher_->publish(processed_msg); // Publish the new sine wave value to the topic
        RCLCPP_INFO(this->get_logger(), "Processed Signal: %.2f", processed_signal); // Print the processed signal value in the terminal
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr signal_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr time_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processed_signal_publisher_;

    double time_;
    const double phase_shift_;
    const double amplitude_factor_;
    const double offset_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}
