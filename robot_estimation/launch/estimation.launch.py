from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 1. Lowpass filter
        Node(
            package="robot_estimation",
            executable="lowpass_imu_node",
            name="lowpass_imu",
            parameters=[
                {
                    "fs": 100.0,
                    "fc": 5.0
                }
            ],
            remappings=[
                ("/zed/zed_node/imu/data_raw", "/zed/zed_node/imu/data_raw"),
                ("/imu_filtered", "/imu_filtered")
            ]
        ),

        # 2. Bias correction
        Node(
            package="robot_estimation",
            executable="imu_bias_correction_node",
            name="imu_bias_correction",
            parameters=[
                {}   # اگر هم پارامتری ندارد باید داخل لیست باشد
            ],
            remappings=[
                ("/imu_filtered", "/imu_filtered"),
                ("/imu_corrected", "/imu_corrected")
            ]
        ),

        # 3. Complementary orientation
        Node(
            package="robot_estimation",
            executable="complementary_orientation_node",
            name="complementary_orientation",
            parameters=[
                {
                    "alpha": 0.98
                }
            ],
            remappings=[
                ("/imu_corrected", "/imu_corrected"),
                ("/wheel_encoder/odom", "/wheel_encoder/odom"),
                ("/estimation/orientation", "/estimation/orientation")
            ]
        ),

        # 4. cmd_vel → rpm converter
        Node(
            package="robot_estimation",
            executable="cmd_vel_to_rpm",
            name="cmd_vel_to_rpm",
            parameters=[
                {
                    "wheel_radius": 0.1,
                    "wheel_base": 0.5
                }
            ],
            remappings=[
                ("/cmd_vel", "/cmd_vel"),
                ("/left_wheel_rpm", "/left_wheel_rpm"),
                ("/right_wheel_rpm", "/right_wheel_rpm")
            ]
        ),

        #5. IMU-based odometry (motion model)
        Node(
            package="robot_estimation",
            executable="imu_motion_odom_node",
            name="imu_motion_odom",
            parameters=[
                {
                    "imu_topic": "/imu_corrected",
                    "orientation_topic": "/estimation/orientation",
                    "odom_topic": "/imu_motion/odom",
                    "odom_frame_id": "odom",
                    "base_frame_id": "base_link"
                }
            ],
            remappings=[
                ("/imu_corrected", "/imu_corrected"),
                ("/estimation/orientation", "/estimation/orientation")
            ]
        ),


    ])
