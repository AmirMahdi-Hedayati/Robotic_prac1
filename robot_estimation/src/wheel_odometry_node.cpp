#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class WheelOdometryNode : public rclcpp::Node
{
public:
    WheelOdometryNode() : Node("wheel_odometry")
    {
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.1);
        wheel_base_   = declare_parameter<double>("wheel_base", 0.7);

        frame_id_       = declare_parameter<std::string>("frame_id", "odom");
        child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");

        left_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/left_wheel_rpm", 10,
            std::bind(&WheelOdometryNode::leftCallback, this, std::placeholders::_1));

        right_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/right_wheel_rpm", 10,
            std::bind(&WheelOdometryNode::rightCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel_encoder/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        last_time_ = now();

        RCLCPP_INFO(get_logger(), "Wheel Odometry Node Started");
    }

private:
    double wheel_radius_, wheel_base_;
    std::string frame_id_, child_frame_id_;

    double left_rpm_ = 0.0, right_rpm_ = 0.0;

    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

    rclcpp::Time last_time_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void leftCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        left_rpm_ = msg->data;
        computeOdometry();
    }

    void rightCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        right_rpm_ = msg->data;
        computeOdometry();
    }

    void computeOdometry()
    {
        rclcpp::Time current_time = now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0) return;

        // rpm → rad/s
        double wl = (left_rpm_  * 2.0 * M_PI) / 60.0;
        double wr = (right_rpm_ * 2.0 * M_PI) / 60.0;

        // linear and angular velocity of robot
        double v = wheel_radius_ * (wl + wr) / 2.0;
        double w = wheel_radius_ * (wr - wl) / wheel_base_;

        // integrate
        x_     += v * cos(theta_) * dt;
        y_     += v * sin(theta_) * dt;
        theta_ += w * dt;

        theta_ = normalizeAngle(theta_);

        publishOdometry(current_time, v, w);

        last_time_ = current_time;
    }

    double normalizeAngle(double a)
    {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    void publishOdometry(const rclcpp::Time &stamp, double v, double w)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id  = child_frame_id_;

        // position
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // twist
        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = w;

        odom_pub_->publish(odom);

        // TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = frame_id_;
        tf_msg.child_frame_id  = child_frame_id_;

        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }
};
    
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
