#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <string>

class ImuMotionOdom : public rclcpp::Node
{
public:
    ImuMotionOdom() : Node("imu_motion_odom")
    {
        // پارامترها
        imu_topic_         = declare_parameter<std::string>("imu_topic", "/imu_corrected");
        orientation_topic_ = declare_parameter<std::string>("orientation_topic", "/estimation/orientation");
        odom_topic_        = declare_parameter<std::string>("odom_topic", "/imu_motion/odom");
        odom_frame_id_     = declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_id_     = declare_parameter<std::string>("base_frame_id", "base_link");

        // حالت اولیه
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
        v_ = 0.0;
        omega_z_ = 0.0;
        has_orientation_ = false;
        has_time_ = false;

        // Subscribers
        sub_orientation_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            orientation_topic_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&ImuMotionOdom::orientationCallback, this, std::placeholders::_1)
        );

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&ImuMotionOdom::imuCallback, this, std::placeholders::_1)
        );

        // Publisher
        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

        // TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(), "ImuMotionOdom node started.");
    }

private:
    // پارامترها
    std::string imu_topic_;
    std::string orientation_topic_;
    std::string odom_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;

    // حالت ربات
    double x_, y_, yaw_;
    double v_;        // سرعت خطی روی محور x بدنه
    double omega_z_;  // سرعت زاویه‌ای (از IMU)

    bool has_orientation_;
    bool has_time_;
    rclcpp::Time last_time_;

    geometry_msgs::msg::Quaternion last_orientation_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_orientation_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ---------- Orientation callback ----------
    void orientationCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // این orientation خروجی فیلتر قبلی توست → مستقیم استفاده می‌کنیم
        last_orientation_ = msg->pose.orientation;

        tf2::Quaternion q;
        tf2::fromMsg(last_orientation_, q);
        q.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        yaw_ = wrapToPi(yaw);

        has_orientation_ = true;
    }

    // ---------- IMU callback (motion model) ----------
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!has_orientation_) {
            // هنوز orientation نداریم → نمی‌توانیم روی نقشه آپدیت کنیم
            return;
        }

        rclcpp::Time t = msg->header.stamp;

        if (!has_time_) {
            last_time_ = t;
            has_time_ = true;
            return;
        }

        double dt = (t - last_time_).seconds();
        if (dt <= 1e-5) {
            dt = 1e-5;
        }
        last_time_ = t;

        // شتاب روی محور x بدنه (فرض بر این است که imu_corrected گرانش را حذف کرده و low-pass شده)
        double ax_body = msg->linear_acceleration.x;

        // سرعت زاویه‌ای حول z → برای twist
        omega_z_ = msg->angular_velocity.z;

        // انتگرال‌گیری سرعت خطی در محور x بدنه
        v_ += ax_body * dt;

        // آپدیت موقعیت روی فریم odom
        double cos_yaw = std::cos(yaw_);
        double sin_yaw = std::sin(yaw_);

        x_ += v_ * cos_yaw * dt;
        y_ += v_ * sin_yaw * dt;

        // انتشار Odom + TF
        publishOdom(t);
    }

    // ---------- publish odom & TF ----------
    void publishOdom(const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        // موقعیت
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // orientation را همان چیزی می‌گذاریم که complementary filter داده
        odom.pose.pose.orientation = last_orientation_;

        // covariance خیلی ساده (می‌توانی به دلخواه تنظیم کنی)
        for (int i = 0; i < 36; ++i) {
            odom.pose.covariance[i] = 0.0;
        }
        odom.pose.covariance[0]  = 0.05;  // var x
        odom.pose.covariance[7]  = 0.05;  // var y
        odom.pose.covariance[35] = 0.01;  // var yaw

        // twist
        odom.twist.twist.linear.x  = v_;
        odom.twist.twist.angular.z = omega_z_;

        pub_odom_->publish(odom);

        // TF: odom → base_link
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = odom_frame_id_;
        tf_msg.child_frame_id = base_frame_id_;

        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = last_orientation_;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    static double wrapToPi(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a <= -M_PI) a += 2.0 * M_PI;
        return a;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuMotionOdom>());
    rclcpp::shutdown();
    return 0;
}
