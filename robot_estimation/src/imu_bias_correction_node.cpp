#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuBiasCorrectionNode : public rclcpp::Node
{
public:
    ImuBiasCorrectionNode() : Node("imu_bias_correction_node")
    {
        calib_samples_ = declare_parameter<int>("calib_samples", 200);

        RCLCPP_INFO(this->get_logger(),
                    "IMU gyro bias correction started. Collecting %d samples (robot must be still)...",
                    calib_samples_);

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu_filtered",
            rclcpp::SensorDataQoS(),
            std::bind(&ImuBiasCorrectionNode::imuCallback, this, std::placeholders::_1));

        pub_imu_corrected_ =
            create_publisher<sensor_msgs::msg::Imu>("/imu_corrected", 10);
    }

private:
    int sample_count_ = 0;
    int calib_samples_;

    double bias_gyro_x_ = 0;
    double bias_gyro_y_ = 0;
    double bias_gyro_z_ = 0;

    double sum_gyro_x_ = 0;
    double sum_gyro_y_ = 0;
    double sum_gyro_z_ = 0;

    bool calibrated_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_corrected_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto corrected = *msg;

        if (!calibrated_)
        {
            sum_gyro_x_ += msg->angular_velocity.x;
            sum_gyro_y_ += msg->angular_velocity.y;
            sum_gyro_z_ += msg->angular_velocity.z;

            sample_count_++;

            if (sample_count_ >= calib_samples_)
            {
                bias_gyro_x_ = sum_gyro_x_ / calib_samples_;
                bias_gyro_y_ = sum_gyro_y_ / calib_samples_;
                bias_gyro_z_ = sum_gyro_z_ / calib_samples_;

                calibrated_ = true;

                RCLCPP_INFO(this->get_logger(),
                            "GYRO BIAS ESTIMATED:\n"
                            " [%f, %f, %f]",
                            bias_gyro_x_, bias_gyro_y_, bias_gyro_z_);
            }

            return;
        }

        corrected.angular_velocity.x -= bias_gyro_x_;
        corrected.angular_velocity.y -= bias_gyro_y_;
        corrected.angular_velocity.z -= bias_gyro_z_;

        pub_imu_corrected_->publish(corrected);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuBiasCorrectionNode>());
    rclcpp::shutdown();
    return 0;
}
