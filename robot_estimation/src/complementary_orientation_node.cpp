#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class ComplementaryIMU : public rclcpp::Node
{
public:
    ComplementaryIMU() : Node("complementary_imu")
    {
        alpha_ = declare_parameter<double>("alpha", 0.98);

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu_corrected",
            rclcpp::SensorDataQoS(),
            std::bind(&ComplementaryIMU::imuCallback, this, std::placeholders::_1));

        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/estimation/orientation", 10);

        last_time_ = this->now();

        RCLCPP_INFO(get_logger(), "Complementary IMU node started, alpha = %.3f", alpha_);
    }

private:
    double alpha_;
    double roll_  = 0.0;
    double pitch_ = 0.0;
    double yaw_   = 0.0;

    bool first_sample_ = true;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // -------------------- dt --------------------
        rclcpp::Time now_time = msg->header.stamp;
        if (now_time.nanoseconds() == 0)
            now_time = this->now();

        double dt = (now_time - last_time_).seconds();
        if (dt <= 0.0 || dt > 0.1)
            dt = 0.01;
        last_time_ = now_time;

        // -------------------- شتاب‌سنج --------------------
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // نرمال‌سازی بردار شتاب → جهت گرانش
        double acc_norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (acc_norm < 1e-6)
            return; // محافظت از تقسیم بر صفر

        double ax_g = ax / acc_norm;
        double ay_g = ay / acc_norm;
        double az_g = az / acc_norm;

        // زاویه‌ها از جهت گرانش نرمال‌شده
        double roll_acc  = std::atan2(ay_g, az_g);
        double pitch_acc = std::atan2(-ax_g, std::sqrt(ay_g*ay_g + az_g*az_g));

        // -------------------- مقدار اولیه --------------------
        if (first_sample_)
        {
            roll_  = roll_acc;
            pitch_ = pitch_acc;
            yaw_   = 0.0;
            first_sample_ = false;
        }

        // -------------------- ژایرو + فیلتر مکمل --------------------
        double gx = msg->angular_velocity.x;
        double gy = msg->angular_velocity.y;
        double gz = msg->angular_velocity.z;

        roll_  = alpha_ * (roll_  + gx * dt) + (1.0 - alpha_) * roll_acc;
        pitch_ = alpha_ * (pitch_ + gy * dt) + (1.0 - alpha_) * pitch_acc;
        yaw_  += gz * dt;

        normalizeAngle(roll_);
        normalizeAngle(pitch_);
        normalizeAngle(yaw_);

        // -------------------- کواترنیون و خروجی --------------------
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now_time;
        pose.header.frame_id = "base_link";

        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pub_pose_->publish(pose);
    }

    void normalizeAngle(double &a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComplementaryIMU>());
    rclcpp::shutdown();
    return 0;
}
