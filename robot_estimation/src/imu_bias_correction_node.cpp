#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuBiasCorrectionNode : public rclcpp::Node
{
public:
    ImuBiasCorrectionNode() : Node("imu_bias_correction_node")
    {
        declare_parameter<int>("calib_samples", 200);

        calib_samples_ = get_parameter("calib_samples").as_int();
        RCLCPP_INFO(this->get_logger(),
                    "IMU bias correction started. Collecting %d samples...",
                    calib_samples_);

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu_filtered",
            rclcpp::SensorDataQoS(),
            std::bind(&ImuBiasCorrectionNode::imuCallback, this, std::placeholders::_1)
        );

        pub_imu_corrected_ =
            create_publisher<sensor_msgs::msg::Imu>("/imu_corrected", 10);
    }

private:
    int sample_count_ = 0;
    int calib_samples_;

    // bias storage
    double bias_acc_x_ = 0, bias_acc_y_ = 0, bias_acc_z_ = 0;
    double bias_gyro_x_ = 0, bias_gyro_y_ = 0, bias_gyro_z_ = 0;

    // temp accumulators
    double sum_acc_x_ = 0, sum_acc_y_ = 0, sum_acc_z_ = 0;
    double sum_gyro_x_ = 0, sum_gyro_y_ = 0, sum_gyro_z_ = 0;

    bool calibrated_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_corrected_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto corrected = *msg;

        if (!calibrated_)
        {
            sum_acc_x_  += msg->linear_acceleration.x;
            sum_acc_y_  += msg->linear_acceleration.y;
            sum_acc_z_  += msg->linear_acceleration.z;

            sum_gyro_x_ += msg->angular_velocity.x;
            sum_gyro_y_ += msg->angular_velocity.y;
            sum_gyro_z_ += msg->angular_velocity.z;

            sample_count_++;

            if (sample_count_ >= calib_samples_)
            {
                bias_acc_x_ = sum_acc_x_ / calib_samples_;
                bias_acc_y_ = sum_acc_y_ / calib_samples_;
                bias_acc_z_ = sum_acc_z_ / calib_samples_;

                bias_gyro_x_ = sum_gyro_x_ / calib_samples_;
                bias_gyro_y_ = sum_gyro_y_ / calib_samples_;
                bias_gyro_z_ = sum_gyro_z_ / calib_samples_;

                calibrated_ = true;

                RCLCPP_INFO(this->get_logger(),
                            "IMU BIAS ESTIMATED:\n"
                            " Acc Bias = [%.6f, %.6f, %.6f]\n"
                            " Gyro Bias = [%.6f, %.6f, %.6f]",
                            bias_acc_x_, bias_acc_y_, bias_acc_z_,
                            bias_gyro_x_, bias_gyro_y_, bias_gyro_z_);
            }
            return;
        }

        // apply bias correction
        corrected.linear_acceleration.x -= bias_acc_x_;
        corrected.linear_acceleration.y -= bias_acc_y_;
        corrected.linear_acceleration.z -= bias_acc_z_;

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
