#include "x500_offboard_control/x500_offboard_control.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

X500OffboardControl::X500OffboardControl()
    : Node("x500_offboard_control")
    , logger_(this->get_logger())
{
    auto qos_profile = rclcpp::QoS(1)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    // Publishers
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
    trajectory_setpoint_publisher_   = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
    vehicle_command_publisher_       = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);

    // Subscribers
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&X500OffboardControl::odom_callback, this, _1));

    vehicle_local_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos_profile,
        std::bind(&X500OffboardControl::vehicle_local_position_callback, this, _1));

    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", qos_profile,
        std::bind(&X500OffboardControl::vehicle_status_callback, this, _1));

    waypoint_subscriber_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/offboard/waypoint", 10, std::bind(&X500OffboardControl::waypoint_callback, this, _1));

    // Timer (5 Hz)
    timer_ = this->create_wall_timer(200ms, std::bind(&X500OffboardControl::timer_callback, this));

    RCLCPP_INFO(logger_, "🚁 x500 Offboard Control Node Initialized!");
}

void X500OffboardControl::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_local_pos_ = msg;
}

void X500OffboardControl::update_offset()
{
    if (!current_odom_ || !current_local_pos_) return;

    // Convert ODOM (ENU) -> NED
    const double ned_from_odom_x = current_odom_->pose.pose.position.y;  // ENU Y -> NED X
    const double ned_from_odom_y = current_odom_->pose.pose.position.x;  // ENU X -> NED Y
    const double ned_from_odom_z = -current_odom_->pose.pose.position.z; // ENU Z -> NED Z

    // PX4 NED
    const double ned_px4_x = current_local_pos_->x;
    const double ned_px4_y = current_local_pos_->y;
    const double ned_px4_z = current_local_pos_->z;

    // Instant error
    const double ex = ned_px4_x - ned_from_odom_x;
    const double ey = ned_px4_y - ned_from_odom_y;
    const double ez = ned_px4_z - ned_from_odom_z;

    // Low-pass filter
    odom_to_px4_offset_.x = OFFSET_ALPHA_ * ex + (1.0 - OFFSET_ALPHA_) * odom_to_px4_offset_.x;
    odom_to_px4_offset_.y = OFFSET_ALPHA_ * ey + (1.0 - OFFSET_ALPHA_) * odom_to_px4_offset_.y;
    odom_to_px4_offset_.z = OFFSET_ALPHA_ * ez + (1.0 - OFFSET_ALPHA_) * odom_to_px4_offset_.z;
}

geometry_msgs::msg::Pose X500OffboardControl::enu_to_ned_pose(const geometry_msgs::msg::Pose& enu_pose)
{
    geometry_msgs::msg::Pose ned_pose;
    ned_pose.position.x = enu_pose.position.y;
    ned_pose.position.y = enu_pose.position.x;
    ned_pose.position.z = -enu_pose.position.z;

    // Orientation conversion ENU -> NED
    tf2::Quaternion enu_quat;
    tf2::fromMsg(enu_pose.orientation, enu_quat);
    tf2::Matrix3x3 m(enu_quat);
    double r, p, y;
    m.getRPY(r, p, y);
    double yaw_ned = -y + M_PI/2.0;
    while (yaw_ned > M_PI) yaw_ned -= 2*M_PI;
    while (yaw_ned < -M_PI) yaw_ned += 2*M_PI;

    tf2::Quaternion ned_q;
    ned_q.setRPY(r, p, yaw_ned);
    ned_pose.orientation = tf2::toMsg(ned_q);

    return ned_pose;
}

void X500OffboardControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = msg;
    update_offset();
    RCLCPP_INFO_THROTTLE(logger_, *get_clock(), 5000,
        "📍 Odom ENU: x=%.2f y=%.2f z=%.2f | Offset NED: [%.2f %.2f %.2f]",
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z,
        odom_to_px4_offset_.x, odom_to_px4_offset_.y, odom_to_px4_offset_.z);
}

void X500OffboardControl::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    vehicle_status_ = *msg;
}

void X500OffboardControl::waypoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
    geometry_msgs::msg::Pose enu_pose;
    enu_pose.position.x = msg->position[0];
    enu_pose.position.y = msg->position[1];
    enu_pose.position.z = msg->position[2];
    tf2::Quaternion enu_q; enu_q.setRPY(0.0, 0.0, msg->yaw); enu_pose.orientation = tf2::toMsg(enu_q);

    auto ned_pose = enu_to_ned_pose(enu_pose);

    // Apply offset (in NED)
    ned_pose.position.x += odom_to_px4_offset_.x;
    ned_pose.position.y += odom_to_px4_offset_.y;
    ned_pose.position.z += odom_to_px4_offset_.z;

    current_waypoint_ = std::make_shared<px4_msgs::msg::TrajectorySetpoint>();
    current_waypoint_->position[0] = ned_pose.position.x;
    current_waypoint_->position[1] = ned_pose.position.y;
    current_waypoint_->position[2] = ned_pose.position.z;

    std::copy(msg->velocity.begin(), msg->velocity.end(), current_waypoint_->velocity.begin());
    std::copy(msg->acceleration.begin(), msg->acceleration.end(), current_waypoint_->acceleration.begin());
    std::copy(msg->jerk.begin(), msg->jerk.end(), current_waypoint_->jerk.begin());

    tf2::Quaternion ned_q; tf2::fromMsg(ned_pose.orientation, ned_q);
    double r,p,yaw_ned; tf2::Matrix3x3(ned_q).getRPY(r,p,yaw_ned);
    current_waypoint_->yaw = yaw_ned;
    current_waypoint_->yawspeed = msg->yawspeed;
}

X500OffboardControl::Position X500OffboardControl::get_current_position() const
{
    if (current_odom_) {
        return { current_odom_->pose.pose.position.x,
                 current_odom_->pose.pose.position.y,
                 current_odom_->pose.pose.position.z };
    }
    return {0.0, 0.0, 0.0};
}

void X500OffboardControl::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(logger_, "🔧 Arm command sent");
}

void X500OffboardControl::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    RCLCPP_INFO(logger_, "🔧 Disarm command sent");
}

void X500OffboardControl::engage_offboard_mode()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(logger_, "🎮 Switching to offboard mode");
}

void X500OffboardControl::land()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(logger_, "⏬ Switching to land mode");
}

void X500OffboardControl::publish_offboard_control_heartbeat_signal()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true; msg.velocity = false; msg.acceleration = false; msg.attitude = false; msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void X500OffboardControl::publish_takeoff_setpoint()
{
    if (!current_odom_) return;

    if (!takeoff_start_position_) {
        takeoff_start_position_ = { current_odom_->pose.pose.position.x,
                                    current_odom_->pose.pose.position.y,
                                    current_odom_->pose.pose.position.z };
        takeoff_target_altitude_ = takeoff_start_position_->z + TAKEOFF_ALTITUDE; // ENU up
        RCLCPP_INFO(logger_, "📍 Takeoff init ENU: [%.2f %.2f %.2f] -> target z=%.2f",
                    takeoff_start_position_->x, takeoff_start_position_->y, takeoff_start_position_->z,
                    takeoff_target_altitude_.value());
    }

    // Build ENU target at (x0,y0,z0+dz)
    geometry_msgs::msg::Pose enu_pose;
    enu_pose.position.x = takeoff_start_position_->x;
    enu_pose.position.y = takeoff_start_position_->y;
    enu_pose.position.z = takeoff_target_altitude_.value();
    tf2::Quaternion q; q.setRPY(0,0,0); enu_pose.orientation = tf2::toMsg(q);

    // Convert to NED then apply offset ONCE
    auto ned_pose = enu_to_ned_pose(enu_pose);
    ned_pose.position.x += odom_to_px4_offset_.x;
    ned_pose.position.y += odom_to_px4_offset_.y;
    ned_pose.position.z += odom_to_px4_offset_.z;

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position[0] = ned_pose.position.x;
    msg.position[1] = ned_pose.position.y;
    msg.position[2] = ned_pose.position.z;

    tf2::Quaternion ned_q; tf2::fromMsg(ned_pose.orientation, ned_q);
    double r,p,y; tf2::Matrix3x3(ned_q).getRPY(r,p,y);
    msg.yaw = y;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    trajectory_setpoint_publisher_->publish(msg);
}

bool X500OffboardControl::is_takeoff_complete()
{
    if (!current_odom_ || !takeoff_target_altitude_) return false;
    const double curr_alt = current_odom_->pose.pose.position.z; // ENU up
    const double target_alt = takeoff_target_altitude_.value();
    const double diff = std::fabs(curr_alt - target_alt);
    const bool reached = diff < ALTITUDE_TOLERANCE;

    if (reached && !takeoff_completed_) {
        takeoff_completed_ = true;
        auto p = get_current_position();
        RCLCPP_INFO(logger_, "✅ Takeoff completed! z=%.2f target=%.2f pos=[%.2f %.2f %.2f]",
                    curr_alt, target_alt, p.x, p.y, p.z);
    }
    return takeoff_completed_;
}

void X500OffboardControl::publish_vehicle_command(uint16_t command,
                                                  float param1, float param2, float param3,
                                                  float param4, float param5, float param6, float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1; msg.param2 = param2; msg.param3 = param3;
    msg.param4 = param4; msg.param5 = param5; msg.param6 = param6; msg.param7 = param7;
    msg.target_system = 1; msg.target_component = 1;
    msg.source_system = 1; msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void X500OffboardControl::timer_callback()
{
    publish_offboard_control_heartbeat_signal();

    if (!current_odom_) {
        RCLCPP_DEBUG_THROTTLE(logger_, *this->get_clock(), 5000, "⏳ Waiting for odometry data...");
        return;
    }

    if (offboard_setpoint_counter_ == 10) {
        engage_offboard_mode();
        arm();
    }

    if (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        if (current_waypoint_) {
            current_waypoint_->timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_setpoint_publisher_->publish(*current_waypoint_);
            
            // Convert NED to ENU for waypoint coordinates
            double waypoint_enu_x = current_waypoint_->position[1];   // NED Y -> ENU X
            double waypoint_enu_y = current_waypoint_->position[0];   // NED X -> ENU Y  
            double waypoint_enu_z = -current_waypoint_->position[2];  // NED -Z -> ENU Z
            
            // Convert yaw from NED to ENU frame
            double waypoint_yaw_enu = current_waypoint_->yaw + M_PI/2;  // NED to ENU yaw conversion
            if (waypoint_yaw_enu > M_PI) waypoint_yaw_enu -= 2*M_PI;   // Normalize to [-π, π]
            
            // For RPY display (assuming roll=0, pitch=0 for trajectory setpoint)
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = waypoint_yaw_enu;
            
            RCLCPP_INFO_THROTTLE(logger_, *get_clock(), 3000,
                "🧭 Current waypoint | Timestamp: %lu μs\n"
                "   Target NED: [%.3f %.3f %.3f] yaw=%.2f (%.1f°)\n"
                "   Target ENU: [%.3f %.3f %.3f] RPY=[%.2f %.2f %.2f] (%.1f° %.1f° %.1f°)\n",
                current_waypoint_->timestamp,
                current_waypoint_->position[0], current_waypoint_->position[1], current_waypoint_->position[2],
                current_waypoint_->yaw, current_waypoint_->yaw * 180.0 / M_PI,
                waypoint_enu_x, waypoint_enu_y, waypoint_enu_z,
                roll, pitch, yaw,
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
                
        } else if (!is_takeoff_complete()) {
            publish_takeoff_setpoint();
        } else {
            publish_takeoff_setpoint();
        }
    }

    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
}
