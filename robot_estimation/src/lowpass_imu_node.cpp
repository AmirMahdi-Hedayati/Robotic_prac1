#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

class LowPassImuNode : public rclcpp::Node
{
public:
    LowPassImuNode() : Node("lowpass_imu_node")
    {
        // ---------- Parameters ----------
        fs_ = declare_parameter<double>("fs", 100.0);   // Sampling rate
        fc_ = declare_parameter<double>("fc", 5.0);     // Cutoff frequency

        alpha_ = (2.0 * M_PI * fc_) / (2.0 * M_PI * fc_ + fs_);

        RCLCPP_INFO(this->get_logger(),
                    "Lowpass IMU Filter initialized: fs=%.2f  fc=%.2f  alpha=%.4f",
                    fs_, fc_, alpha_);

        // ---------- Subscriber ----------
        sub_imu_raw_ = create_subscription<sensor_msgs::msg::Imu>(
            "/zed/zed_node/imu/data_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&LowPassImuNode::imuCallback, this, std::placeholders::_1)
        );

        // ---------- Publisher ----------
        pub_imu_filtered_ =
            create_publisher<sensor_msgs::msg::Imu>("/imu_filtered", 10);
    }

private:
    double fs_, fc_, alpha_;
    bool first_sample_ = true;

    // Store previous values
    double prev_acc_x_ = 0, prev_acc_y_ = 0, prev_acc_z_ = 0;
    double prev_gyro_x_ = 0, prev_gyro_y_ = 0, prev_gyro_z_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_filtered_;

    // ---------- Main Callback ----------
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto filtered = *msg;  // copy message fully

        double acc_x = msg->linear_acceleration.x;
        double acc_y = msg->linear_acceleration.y;
        double acc_z = msg->linear_acceleration.z;

        double gyro_x = msg->angular_velocity.x;
        double gyro_y = msg->angular_velocity.y;
        double gyro_z = msg->angular_velocity.z;

        // First sample → no filter yet
        if (first_sample_)
        {
            prev_acc_x_ = acc_x;
            prev_acc_y_ = acc_y;
            prev_acc_z_ = acc_z;

            prev_gyro_x_ = gyro_x;
            prev_gyro_y_ = gyro_y;
            prev_gyro_z_ = gyro_z;

            first_sample_ = false;
        }
        else
        {
            // Apply low-pass to acceleration
            prev_acc_x_ = alpha_ * acc_x + (1 - alpha_) * prev_acc_x_;
            prev_acc_y_ = alpha_ * acc_y + (1 - alpha_) * prev_acc_y_;
            prev_acc_z_ = alpha_ * acc_z + (1 - alpha_) * prev_acc_z_;

            // Apply low-pass to gyroscope
            prev_gyro_x_ = alpha_ * gyro_x + (1 - alpha_) * prev_gyro_x_;
            prev_gyro_y_ = alpha_ * gyro_y + (1 - alpha_) * prev_gyro_y_;
            prev_gyro_z_ = alpha_ * gyro_z + (1 - alpha_) * prev_gyro_z_;
        }

        // Put filtered values back
        filtered.linear_acceleration.x = prev_acc_x_;
        filtered.linear_acceleration.y = prev_acc_y_;
        filtered.linear_acceleration.z = prev_acc_z_;

        filtered.angular_velocity.x = prev_gyro_x_;
        filtered.angular_velocity.y = prev_gyro_y_;
        filtered.angular_velocity.z = prev_gyro_z_;

        pub_imu_filtered_->publish(filtered);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowPassImuNode>());
    rclcpp::shutdown();
    return 0;
}
