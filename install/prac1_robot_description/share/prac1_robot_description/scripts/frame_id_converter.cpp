#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanFrameIdConverter : public rclcpp::Node
{
public:
  LaserScanFrameIdConverter()
  : Node("laser_scan_frame_id_converter_node")
  {
    // Publisher: republish LaserScan with corrected frame_id
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS()   // BestEffort QoS for sensor data
    );

    // Subscriber: listen to Gazebo LaserScan topic
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/gz_lidar/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&LaserScanFrameIdConverter::scanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
                "LaserScan FrameIdConverter running. Listening to /gz_lidar/scan ...");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto new_msg = *msg;                     // Copy original scan
    new_msg.header.frame_id = "rplidar_c1";  // Correct TF frame
    pub_->publish(new_msg);                  // Publish updated scan
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanFrameIdConverter>());
  rclcpp::shutdown();
  return 0;
}
