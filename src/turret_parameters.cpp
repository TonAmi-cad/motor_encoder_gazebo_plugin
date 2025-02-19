#include "gazebo_ros_control/turret_control_plugin.hpp"
#include <rclcpp/rclcpp.hpp>

namespace gazebo_ros_control {

void TurretControlPlugin::LoadParameter(sdf::ElementPtr sdf, const std::string& name, std::string& param) {
    if (sdf->HasElement(name)) {
        param = sdf->GetElement(name)->Get<std::string>();
    }
}

void TurretControlPlugin::LoadParameter(sdf::ElementPtr sdf, const std::string& name, double& param) {
    if (sdf->HasElement(name)) {
        param = sdf->GetElement(name)->Get<double>();
    }
}

void TurretControlPlugin::LoadAccelerationProfile(sdf::ElementPtr sdf, AccelerationProfile& profile) {
    if (sdf->HasElement("acceleration_profile")) {
        std::string profile_str = sdf->GetElement("acceleration_profile")->Get<std::string>();
        if (profile_str == "cubic") {
            profile = AccelerationProfile::CUBIC_POLYNOMIAL;
        } else if (profile_str == "s_curve") {
            profile = AccelerationProfile::S_CURVE;
        } else if (profile_str == "trapezoidal") {
            profile = AccelerationProfile::TRAPEZOIDAL;
        } else {
            profile = AccelerationProfile::QUINTIC_POLYNOMIAL;
        }
    }
}

void TurretControlPlugin::LoadRotationDirection(sdf::ElementPtr sdf, RotationDirection& direction) {
    if (sdf->HasElement("rotation_direction")) {
        std::string direction_str = sdf->GetElement("rotation_direction")->Get<std::string>();
        if (direction_str == "cw" || direction_str == "clockwise") {
            direction = RotationDirection::CLOCKWISE;
        } else if (direction_str == "ccw" || direction_str == "counterclockwise") {
            direction = RotationDirection::COUNTERCLOCKWISE;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("turret_control"),
                "Invalid rotation_direction value: %s. Using default.", 
                direction_str.c_str());
        }
    }
}

void TurretControlPlugin::LoadParameters(sdf::ElementPtr sdf) {
    // Required parameters
    if (!sdf->HasElement("joint")) {
        RCLCPP_ERROR(rclcpp::get_logger("turret_control"), "Plugin missing <joint> element");
        return;
    }
    params_.joint_name = sdf->GetElement("joint")->Get<std::string>();

    // Load optional parameters
    LoadParameter(sdf, "command_topic", params_.command_topic);
    LoadParameter(sdf, "trajectory_duration", params_.trajectory_duration);
    LoadParameter(sdf, "max_acceleration", params_.max_acceleration);
    LoadParameter(sdf, "max_velocity", params_.max_velocity);
    LoadParameter(sdf, "s_curve_steepness", params_.s_curve_steepness);
    LoadParameter(sdf, "initial_position", params_.initial_position);
    LoadParameter(sdf, "state_topic", params_.state_topic);
    LoadParameter(sdf, "encoder_resolution", params_.encoder_resolution);
    LoadParameter(sdf, "hold_time", params_.hold_time);
    
    // Load acceleration profile
    LoadAccelerationProfile(sdf, params_.profile);

    // Load rotation direction
    LoadRotationDirection(sdf, params_.rotation_direction);

    // Validate parameters
    if (params_.trajectory_duration <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("turret_control"), 
                     "Invalid trajectory_duration: %f. Must be positive.", 
                     params_.trajectory_duration);
        return;
    }

    if (params_.max_acceleration <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("turret_control"), 
                     "Invalid max_acceleration: %f. Must be positive.", 
                     params_.max_acceleration);
        return;
    }

    if (params_.max_velocity <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("turret_control"), 
                     "Invalid max_velocity: %f. Must be positive.", 
                     params_.max_velocity);
        return;
    }

    if (params_.encoder_resolution <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("turret_control"), 
                     "Invalid encoder_resolution: %f. Must be positive.", 
                     params_.encoder_resolution);
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("turret_control"), 
                "Loaded parameters successfully for joint: %s", 
                params_.joint_name.c_str());
}

}  // namespace gazebo_ros_control 