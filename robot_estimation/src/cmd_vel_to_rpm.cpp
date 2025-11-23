#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class CmdVelToRpm : public rclcpp::Node
{
public:
    CmdVelToRpm() : Node("cmd_vel_to_rpm")
    {
        // Declare robot parameters
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.1);
        wheel_separation_ = declare_parameter<double>("wheel_separation", 0.7);

        // Publishers
        left_pub_  = create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

        // Subscriber
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToRpm::cmdCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "cmd_vel_to_rpm node started.");
    }

private:
    double wheel_radius_;
    double wheel_separation_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;       // forward velocity
        double w = msg->angular.z;      // yaw rate

        // Compute wheel angular speeds (rad/s)
        double w_left  = (v / wheel_radius_) - (w * wheel_separation_ / (2.0 * wheel_radius_));
        double w_right = (v / wheel_radius_) + (w * wheel_separation_ / (2.0 * wheel_radius_));

        // Convert to RPM
        double left_rpm  = w_left  * 60.0 / (2 * M_PI);
        double right_rpm = w_right * 60.0 / (2 * M_PI);

        std_msgs::msg::Float64 l_msg;
        std_msgs::msg::Float64 r_msg;
        l_msg.data = left_rpm;
        r_msg.data = right_rpm;

        left_pub_->publish(l_msg);
        right_pub_->publish(r_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToRpm>());
    rclcpp::shutdown();
    return 0;
}
