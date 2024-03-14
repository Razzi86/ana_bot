#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <regex>
#include <sstream>
#include <vector>

// THIS DOESN'T WORK. FAILED EXPERIMENT WITH RESEARCH + CHATGPT. TODO: Watch PID tutorial

class PIDController : public rclcpp::Node
{
public:
    PIDController() : Node("pid_controller"), kp_(1.0), ki_(0.0), kd_(0.0),
                      setpoint_(0.0), error_(0.0),
                      integral_(0.0), derivative_(0.0), last_error_(0.0)
    {
        encoder_left_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "encoder_left_data", 10, std::bind(&PIDController::encoderLeftCallback, this, std::placeholders::_1));
        encoder_right_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "encoder_right_data", 10, std::bind(&PIDController::encoderRightCallback, this, std::placeholders::_1));
        control_publisher_ = this->create_publisher<std_msgs::msg::String>("control_command", 10);

        // Replace with actual setpoint value
        setpoint_ = 100.0; // Example setpoint
    }

    void controlLoop()
    {
        // Calculate error (example: using left encoder average)
        error_ = setpoint_ - left_encoder_avg_;

        // Integral term calculation
        integral_ += error_;

        // Derivative term calculation
        derivative_ = error_ - last_error_;

        // Apply PID formula
        double control_value = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;

        // Update last error
        last_error_ = error_;

        // Publish control command
        std_msgs::msg::String control_msg;
        control_msg.data = std::to_string(control_value);
        control_publisher_->publish(control_msg);
    }

private:
    // PID parameters
    double kp_, ki_, kd_;

    // State variables
    double setpoint_;
    double left_encoder_avg_, right_encoder_avg_;
    double error_;
    double integral_;
    double derivative_;
    double last_error_;

    // ROS2 Subscribers and Publishers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr encoder_left_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr encoder_right_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_publisher_;

    void encoderLeftCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::regex front_regex("front: (\\d+)");
        std::regex back_regex("back: (\\d+)");
        std::smatch front_match, back_match;

        if (std::regex_search(msg->data, front_match, front_regex) && 
            std::regex_search(msg->data, back_match, back_regex)) 
        {
            try {
                int front_left = std::stoi(front_match[1].str());
                int back_left = std::stoi(back_match[1].str());
                left_encoder_avg_ = (front_left + back_left) / 2.0;
                controlLoop();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing left encoder data: %s", e.what());
            }
        }
    }

    void encoderRightCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::regex front_regex("front: (\\d+)");
        std::regex back_regex("back: (\\d+)");
        std::smatch front_match, back_match;

        if (std::regex_search(msg->data, front_match, front_regex) && 
            std::regex_search(msg->data, back_match, back_regex)) 
        {
            try {
                int front_right = std::stoi(front_match[1].str());
                int back_right = std::stoi(back_match[1].str());
                right_encoder_avg_ = (front_right + back_right) / 2.0;
                // Optionally use right_encoder_avg_ in your control logic
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing right encoder data: %s", e.what());
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
