/*
 * RPLIDAR A2M7 ROS2 Node
 * 
 * Author: VbsmRobotic
 * GitHub: https://github.com/VbsmRobotic/rplidar_a2m7_ros2
 */

#ifndef RPLIDAR_A2M7_NODE_HPP
#define RPLIDAR_A2M7_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

// RPLIDAR SDK includes
#include "sdk/include/rplidar.h"

#include <string>
#include <memory>
#include <chrono>
#include <vector>

namespace rplidar_a2m7
{

class RplidarA2M7Node : public rclcpp::Node
{
public:
    explicit RplidarA2M7Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~RplidarA2M7Node();

private:
    // ROS2 Publishers and Subscribers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // TF2 Static Transform Broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    // RPLIDAR SDK
    rp::standalone::rplidar::RPlidarDriver* rplidar_driver_;
    
    // ROS2 Timers
    rclcpp::TimerBase::SharedPtr scan_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Parameters
    std::string serial_port_;
    int serial_baudrate_;
    std::string frame_id_;
    bool inverted_;
    bool angle_compensate_;
      double max_distance_;
      double min_distance_;
      double scan_frequency_;
      bool use_infinity_for_unmeasured_;
      
      // RPLIDAR A2M7 specific parameters
    std::string scan_mode_;
    int point_number_;
    bool flip_x_axis_;
    
    // Internal state
    bool is_scanning_;
    bool is_connected_;
    std::chrono::steady_clock::time_point last_scan_time_;
    
    // RPLIDAR scan data
    std::vector<rp::standalone::rplidar::RplidarScanMode> supported_scan_modes_;
    
    // Methods
    void declare_parameters();
    void load_parameters();
    void setup_ros2_interfaces();
    void scan_timer_callback();
    void status_timer_callback();
    void publish_scan();
    void publish_status();
    
    // RPLIDAR A2M7 specific methods
    void initialize_rplidar_a2m7();
    void configure_scan_mode();
    void handle_scan_data();
};

} // namespace rplidar_a2m7

#endif // RPLIDAR_A2M7_NODE_HPP
