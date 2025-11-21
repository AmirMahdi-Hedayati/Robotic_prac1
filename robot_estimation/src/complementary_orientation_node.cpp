#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class ComplementaryOrientation : public rclcpp::Node
{
public:
    ComplementaryOrientation() : Node("complementary_orientation")
    {
        alpha_ = declare_parameter<double>("alpha", 0.98);

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu_corrected",
            rclcpp::SensorDataQoS(),
            std::bind(&ComplementaryOrientation::imuCallback, this, std::placeholders::_1)
        );

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_encoder/odom",   // اگر تاپیکت چیز دیگری است، فقط اینجا عوض کن
            rclcpp::SensorDataQoS(),
            std::bind(&ComplementaryOrientation::odomCallback, this, std::placeholders::_1)
        );

        pub_orientation_ =
            create_publisher<std_msgs::msg::Float64>("/estimation/orientation", 10);

        last_time_ = now();
    }

private:
    double alpha_;
    double fused_yaw_ = 0.0;
    double odom_yaw_ = 0.0;

    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_orientation_;

    // -------- Odometry Callback --------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &q = msg->pose.pose.orientation;
        tf2::Quaternion qt(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(qt).getRPY(roll, pitch, yaw);

        odom_yaw_ = yaw;
    }

    // -------- IMU Callback (main update) --------
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0) dt = 0.001;

        last_time_ = current_time;

        double gyro_z = msg->angular_velocity.z;

        // 1) Predict from gyro
        double yaw_gyro = fused_yaw_ + gyro_z * dt;

        // Normalize:
        yaw_gyro = normalizeAngle(yaw_gyro);

        // 2) Fuse with odometry
        fused_yaw_ =
            alpha_ * yaw_gyro +
            (1.0 - alpha_) * odom_yaw_;

        fused_yaw_ = normalizeAngle(fused_yaw_);

        // 3) Publish result
        std_msgs::msg::Float64 msg_out;
        msg_out.data = fused_yaw_;

        pub_orientation_->publish(msg_out);
    }

    // Keep angle in [-pi, pi]
    double normalizeAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComplementaryOrientation>());
    rclcpp::shutdown();
    return 0;
}
