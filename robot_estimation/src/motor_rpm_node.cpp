#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class MotorRpmNode : public rclcpp::Node
{
public:
    MotorRpmNode() : Node("motor_rpm_node")
    {
        // Publishers to wheel RPM topics
        left_rpm_pub_ = create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_rpm_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

        // Subscriber to motor command array
        motor_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/motor_commands",
            10,
            std::bind(&MotorRpmNode::motorCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Motor RPM Node started.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rpm_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_sub_;

    void motorCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2)
        {
            RCLCPP_WARN(get_logger(), "motor_commands must contain at least 2 elements!");
            return;
        }

        double left_rpm = msg->data[0];
        double right_rpm = msg->data[1];

        // Make messages
        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;

        left_msg.data = left_rpm;
        right_msg.data = right_rpm;

        // Publish
        left_rpm_pub_->publish(left_msg);
        right_rpm_pub_->publish(right_msg);

        RCLCPP_DEBUG(get_logger(),
                     "Published -> Left: %.2f rpm | Right: %.2f rpm",
                     left_rpm, right_rpm);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorRpmNode>());
    rclcpp::shutdown();
    return 0;
}
