/*
 * RPLIDAR A2M7 ROS2 Node Implementation
 * 
 * Author: VbsmRobotic
 * Email: vahid.behtaji2013@gmail.com
 * GitHub: https://github.com/VbsmRobotic/rplidar_a2m7_ros2
 */

#include "rplidar_a2m7/rplidar_a2m7_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <cmath>

namespace rplidar_a2m7
{

RplidarA2M7Node::RplidarA2M7Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("rplidar_a2m7_node", options), is_scanning_(false), is_connected_(false), rplidar_driver_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RPLIDAR A2M7 Node");
    
    // Declare and load parameters
    declare_parameters();
    load_parameters();
    
    // Setup ROS2 interfaces
    setup_ros2_interfaces();
    
    // Initialize RPLIDAR A2M7
    initialize_rplidar_a2m7();
    
    RCLCPP_INFO(this->get_logger(), "RPLIDAR A2M7 Node initialized successfully");
}

RplidarA2M7Node::~RplidarA2M7Node()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down RPLIDAR A2M7 Node");
    
    // Stop scanning and disconnect
    if (is_scanning_ && rplidar_driver_) {
        rplidar_driver_->stop();
        rplidar_driver_->stopMotor();
    }
    
    if (rplidar_driver_) {
        rp::standalone::rplidar::RPlidarDriver::DisposeDriver(rplidar_driver_);
        rplidar_driver_ = nullptr;
    }
}

void RplidarA2M7Node::declare_parameters()
{
    // Serial communication parameters
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("serial_baudrate", 256000);
    
    // Frame and scan parameters
    this->declare_parameter("frame_id", "laser");
    this->declare_parameter("inverted", false);
    this->declare_parameter("angle_compensate", true);
    this->declare_parameter("max_distance", 16.0);
    this->declare_parameter("min_distance", 0.1);
    this->declare_parameter("scan_frequency", 10.0);
    this->declare_parameter("use_infinity_for_unmeasured", false);
    
    // A2M7 specific parameters
    this->declare_parameter("scan_mode", "Sensitivity");
    this->declare_parameter("point_number", 15900);
    this->declare_parameter("flip_x_axis", false);
}

void RplidarA2M7Node::load_parameters()
{
    // Load parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    serial_baudrate_ = this->get_parameter("serial_baudrate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    inverted_ = this->get_parameter("inverted").as_bool();
    angle_compensate_ = this->get_parameter("angle_compensate").as_bool();
    max_distance_ = this->get_parameter("max_distance").as_double();
    min_distance_ = this->get_parameter("min_distance").as_double();
    scan_frequency_ = this->get_parameter("scan_frequency").as_double();
    use_infinity_for_unmeasured_ = this->get_parameter("use_infinity_for_unmeasured").as_bool();
    scan_mode_ = this->get_parameter("scan_mode").as_string();
    point_number_ = this->get_parameter("point_number").as_int();
    flip_x_axis_ = this->get_parameter("flip_x_axis").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Serial Port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Baud Rate: %d", serial_baudrate_);
    RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Scan Mode: %s", scan_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Point Number: %d", point_number_);
    RCLCPP_INFO(this->get_logger(), "  Use Infinity for Unmeasured: %s", use_infinity_for_unmeasured_ ? "YES (WARNING: Breaks Navigation)" : "NO (Navigation-Friendly)");
}

void RplidarA2M7Node::setup_ros2_interfaces()
{
    // Create publishers with RELIABLE QoS for RViz2 compatibility
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(1).reliable());
    
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "rplidar_a2m7/status", rclcpp::QoS(1).reliable());
    
    // Create static transform broadcaster for TF
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Publish static transform from base_link to laser
    auto static_transform = geometry_msgs::msg::TransformStamped();
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "base_link";
    static_transform.child_frame_id = frame_id_;
    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;
    tf_static_broadcaster_->sendTransform(static_transform);
    
    // Create timers
    auto scan_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / scan_frequency_));
    scan_timer_ = this->create_wall_timer(
        scan_period, std::bind(&RplidarA2M7Node::scan_timer_callback, this));
    
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&RplidarA2M7Node::status_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "ROS2 interfaces setup complete");
}

void RplidarA2M7Node::initialize_rplidar_a2m7()
{
    RCLCPP_INFO(this->get_logger(), "Initializing RPLIDAR A2M7...");
    RCLCPP_INFO(this->get_logger(), "RPLIDAR A2M7 running on ROS 2 package rplidar_a2m7");
    RCLCPP_INFO(this->get_logger(), "SDK Version: '1.12.0'");
    
    // Create RPLIDAR driver
    rplidar_driver_ = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    
    if (!rplidar_driver_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create RPLIDAR driver");
        return;
    }
    
    // Connect to RPLIDAR
    u_result result = rplidar_driver_->connect(serial_port_.c_str(), serial_baudrate_);
    if (IS_FAIL(result)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to RPLIDAR on %s at %d baud", 
                     serial_port_.c_str(), serial_baudrate_);
        rp::standalone::rplidar::RPlidarDriver::DisposeDriver(rplidar_driver_);
        rplidar_driver_ = nullptr;
        return;
    }
    
    is_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to RPLIDAR A2M7 on %s", serial_port_.c_str());
    
    // Get device info
    rplidar_response_device_info_t device_info;
    result = rplidar_driver_->getDeviceInfo(device_info);
    if (IS_OK(result)) {
        RCLCPP_INFO(this->get_logger(), "RPLIDAR S/N: %s", device_info.serialnum);
        RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%d", device_info.firmware_version >> 8, device_info.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", device_info.hardware_version);
    }
    
    // Get health status
    rplidar_response_device_health_t health_info;
    result = rplidar_driver_->getHealth(health_info);
    if (IS_OK(result)) {
        RCLCPP_INFO(this->get_logger(), "RPLidar health status: %d", health_info.status);
    }
    
    // Get supported scan modes
    result = rplidar_driver_->getAllSupportedScanModes(supported_scan_modes_);
    if (IS_OK(result)) {
        RCLCPP_INFO(this->get_logger(), "Supported scan modes: %zu", supported_scan_modes_.size());
    }
    
    // Start motor
    result = rplidar_driver_->startMotor();
    if (IS_FAIL(result)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start RPLIDAR motor");
        return;
    }
    
    // Start scanning
    result = rplidar_driver_->startScan(false, true);
    if (IS_FAIL(result)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start RPLIDAR scanning");
        return;
    }
    
    is_scanning_ = true;
    last_scan_time_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "Current scan mode: %s, max_distance: %.1f m, Point number: %d, angle_compensate: %d, flip_x_axis: %d", 
                scan_mode_.c_str(), max_distance_, point_number_, angle_compensate_ ? 1 : 0, flip_x_axis_ ? 1 : 0);
}

void RplidarA2M7Node::configure_scan_mode()
{
    RCLCPP_INFO(this->get_logger(), "Configuring scan mode: %s", scan_mode_.c_str());
}

void RplidarA2M7Node::scan_timer_callback()
{
    if (is_scanning_) {
        publish_scan();
    }
}

void RplidarA2M7Node::status_timer_callback()
{
    publish_status();
}

void RplidarA2M7Node::publish_scan()
{
    if (!is_scanning_ || !is_connected_ || !rplidar_driver_) {
        return;
    }
    
    auto scan_msg = sensor_msgs::msg::LaserScan();
    
    // Set header
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = frame_id_;
    
    // Set scan parameters
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = 2.0 * M_PI / point_number_;
    scan_msg.time_increment = 1.0 / (scan_frequency_ * point_number_);
    scan_msg.scan_time = 1.0 / scan_frequency_;
    scan_msg.range_min = min_distance_;
    scan_msg.range_max = max_distance_;
    
    // Get scan data from RPLIDAR
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t node_count = sizeof(nodes) / sizeof(nodes[0]);
    
    u_result result = rplidar_driver_->grabScanDataHq(nodes, node_count);
    if (IS_FAIL(result)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Failed to grab scan data from RPLIDAR");
        return;
    }
    
    // Process scan data
    scan_msg.ranges.resize(point_number_);
    scan_msg.intensities.resize(point_number_);
    
    // Initialize ranges based on user preference
    for (int i = 0; i < point_number_; ++i) {
        if (use_infinity_for_unmeasured_) {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        } else {
            scan_msg.ranges[i] = max_distance_;  // Navigation-friendly
        }
        scan_msg.intensities[i] = 0.0;
    }
    
    // Convert RPLIDAR data to LaserScan format
    for (size_t i = 0; i < node_count; ++i) {
        // Only skip points that are truly invalid (distance is 0 or quality is 0)
        if (nodes[i].dist_mm_q2 == 0 || nodes[i].quality == 0) continue;
        
        // Convert distance from mm (Q2 format) to meters
        float distance = nodes[i].dist_mm_q2 / 4000.0f;  // Q2 format: divide by 4
        
        // Apply distance limits for navigation
        if (distance < min_distance_ || distance > max_distance_) {
            continue;  // Skip out-of-range points
        }
        
        // Convert angle from Q14 format to radians
        float angle = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;  // Convert to degrees
        angle = angle * M_PI / 180.0f;  // Convert to radians
        
        // Normalize angle to [-π, π]
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        
        // Convert to scan index
        int scan_index = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
        
        if (scan_index >= 0 && scan_index < point_number_) {
            scan_msg.ranges[scan_index] = distance;
            scan_msg.intensities[scan_index] = nodes[i].quality;
        }
    }
    
    // Debug: Count valid ranges for navigation
    int valid_ranges = 0;
    for (int i = 0; i < point_number_; ++i) {
        if (scan_msg.ranges[i] < max_distance_) {
            valid_ranges++;
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Scan data: %d valid ranges out of %d total, node_count: %zu", 
                 valid_ranges, point_number_, node_count);
    
    // Publish scan
    scan_pub_->publish(scan_msg);
    
    // Update timing
    last_scan_time_ = std::chrono::steady_clock::now();
}

void RplidarA2M7Node::publish_status()
{
    auto status_msg = std_msgs::msg::String();
    std::string status_text = "RPLIDAR A2M7: Active, Scanning: " + 
                             std::string(is_scanning_ ? "Yes" : "No") + 
                             ", Mode: " + scan_mode_;
    status_msg.data = status_text;
    status_pub_->publish(status_msg);
}

void RplidarA2M7Node::handle_scan_data()
{
    // This method would handle actual RPLIDAR data processing
    // For now, it's handled in publish_scan()
}

} // namespace rplidar_a2m7

// Main function for standalone executable
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rplidar_a2m7::RplidarA2M7Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
