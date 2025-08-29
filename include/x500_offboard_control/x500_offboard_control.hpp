#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <atomic>
#include <cmath>
#include <optional>
#include <algorithm>

/**
 * @brief x500 Offboard Control Node for PX4 drone control
 *
 * This node provides offboard control capabilities for PX4 drones with:
 * - Automatic takeoff sequence (with PX4<->Odom offset compensation)
 * - ENU to NED coordinate conversion
 * - Navigation waypoint following
 * - Position control using odometry feedback
 */
class X500OffboardControl : public rclcpp::Node
{
public:
    explicit X500OffboardControl();
    ~X500OffboardControl() = default;

private:
    // ROS2 Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // ROS2 Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_subscriber_;

    // Timer for control loop
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    std::atomic<uint64_t> offboard_setpoint_counter_{0};
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    px4_msgs::msg::VehicleLocalPosition::SharedPtr current_local_pos_;
    px4_msgs::msg::VehicleStatus vehicle_status_{};
    px4_msgs::msg::TrajectorySetpoint::SharedPtr current_waypoint_{};
    std::shared_ptr<px4_msgs::msg::TrajectorySetpoint> last_valid_setpoint_;
    rclcpp::Time last_waypoint_time_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
    static constexpr double WAYPOINT_TIMEOUT_SECONDS_ = 1.0;

    // Offset ODOM->PX4 in NED
    struct Offset { double x{0.0}, y{0.0}, z{0.0}; };
    Offset odom_to_px4_offset_{};
    static constexpr double OFFSET_ALPHA_ = 0.01; // low-pass smoothing factor

    // Takeoff parameters
    static constexpr double TAKEOFF_ALTITUDE = 1.0;     
    static constexpr double ALTITUDE_TOLERANCE = 0.1;  

    struct Position { double x, y, z; };
    std::optional<Position> takeoff_start_position_{};  // ENU
    std::optional<double> takeoff_target_altitude_{};   // ENU z target
    bool takeoff_completed_{false};

    // Callback functions
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void waypoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void timer_callback();

    // Offset estimator (called from odom_callback)
    void update_offset();

    // Coordinate transformation ENU -> NED
    geometry_msgs::msg::Pose enu_to_ned_pose(const geometry_msgs::msg::Pose& enu_pose);

    // Control helpers
    void arm();
    void disarm();
    void engage_offboard_mode();
    void land();

    // Publishing functions
    void publish_offboard_control_heartbeat_signal();
    void publish_takeoff_setpoint();
    void publish_hover_setpoint();
    void publish_vehicle_command(uint16_t command,
                                 float param1 = 0.0f,
                                 float param2 = 0.0f,
                                 float param3 = 0.0f,
                                 float param4 = 0.0f,
                                 float param5 = 0.0f,
                                 float param6 = 0.0f,
                                 float param7 = 0.0f);

    // Utility functions
    Position get_current_position() const;
    bool is_takeoff_complete();

    // Logger
    rclcpp::Logger logger_;
};