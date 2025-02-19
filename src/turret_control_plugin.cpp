#include "gazebo_ros_control/turret_control_plugin.hpp"
#include "gazebo_ros_control/turret_control_config.hpp"
#include <cmath>
#include <std_msgs/msg/float64.hpp>
#include "gazebo_ros_control/acceleration_profiles.hpp"
#include "gazebo_ros_control/action/turret_control.hpp"

namespace gazebo_ros_control
{

TurretControlPlugin::TurretControlPlugin():
  // Gazebo objects
  model_(nullptr),
  joint_(nullptr),
  update_connection_(),
  
  // State variables
  target_velocity_(0.0),
  current_velocity_(0.0),
  start_velocity_(0.0),
  trajectory_start_time_(0),
  is_trajectory_active_(false),
  target_position_(0.0),
  current_position_(0.0),
  start_position_(0.0),
  last_command_time_(0),
  initial_reference_(0.0),
  current_state_(TurretState::IDLE),
  has_active_goal_(false),
  
  // ROS objects
  ros_node_(nullptr),
  twist_sub_(nullptr),
  angle_sub_(nullptr),
  state_pub_(nullptr),
  state_timer_(nullptr),
  action_server_(nullptr),
  current_goal_handle_(nullptr)
{
    trajectory_calculator_ = std::make_shared<TrajectoryCalculator>();
}

TurretControlPlugin::~TurretControlPlugin()
{
  this->update_connection_.reset();
}

void TurretControlPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;

  // Load parameters from SDF
  LoadParameters(sdf);

  // Get joint
  joint_ = model_->GetJoint(params_.joint_name);
  if (!joint_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("turret_control"),
      "Joint named %s does not exist.", params_.joint_name.c_str());
    return;
  }

  // Initialize joint position
  InitializeJoint();

  // Create ROS node
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Create subscription to angle command topic
  angle_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
    params_.command_topic, 10,
    std::bind(&TurretControlPlugin::OnAngleCommand, this, std::placeholders::_1));

  // Create publisher for current state
  state_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>(
    params_.state_topic, 10);

  // Create timer for publishing state
  state_timer_ = ros_node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TurretControlPlugin::PublishState, this));

  // Connect to simulation update event
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&TurretControlPlugin::UpdateChild, this));

  // Создаем action server напрямую
  action_server_ = rclcpp_action::create_server<TurretAction>(
    ros_node_,
    "turret_control",
    std::bind(&TurretControlPlugin::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TurretControlPlugin::handleCancel, this, std::placeholders::_1),
    std::bind(&TurretControlPlugin::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    rclcpp::get_logger("turret_control"),
    "Turret control plugin loaded successfully with profile: %d", static_cast<int>(params_.profile));
}

double TurretControlPlugin::ApplyRotationDirection(double position) const {
    // Простая инверсия для CCW
    if (params_.rotation_direction == RotationDirection::COUNTERCLOCKWISE) {
        return -position;
    }
    return position;
}

double TurretControlPlugin::NormalizeAngle(double angle) {
    // Normalize angle to [0, 2π]
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

void TurretControlPlugin::OnAngleCommand(const std_msgs::msg::Float64::SharedPtr msg)
{
    double direction_adjusted_command = AngleUtils::applyRotationDirection(msg->data, params_.rotation_direction);
    double original_command = msg->data;
    
    double command_degrees = AngleUtils::radiansToDegrees(direction_adjusted_command);
    double current_degrees = GetCurrentAngleDegrees();
    
    target_position_ = initial_reference_ + direction_adjusted_command;
    
    trajectory_start_time_ = model_->GetWorld()->SimTime().Double();
    start_position_ = joint_->Position(0);
    last_command_time_ = trajectory_start_time_;
    is_trajectory_active_ = true;
    current_state_ = TurretState::MOVING_TO_TARGET;

    RCLCPP_INFO(
        rclcpp::get_logger("turret_control"),
        "Command received - Original: %.2f rad, Direction Adjusted: %.2f rad, Current: %.2f°",
        original_command,
        direction_adjusted_command,
        current_degrees);
}

double TurretControlPlugin::CalculateTrajectory(double t, double start, double end)
{
  switch (params_.profile) {
    case AccelerationProfile::CUBIC_POLYNOMIAL:
      return CubicPolynomial(t, start, end, params_.trajectory_duration);
    case AccelerationProfile::S_CURVE:
      return SCurve(t, start, end, params_.trajectory_duration, params_.s_curve_steepness);
    case AccelerationProfile::TRAPEZOIDAL:
      return TrapezoidalProfile(t, start, end, params_.trajectory_duration, params_.max_acceleration);
    case AccelerationProfile::QUINTIC_POLYNOMIAL:
    default:
      return QuinticPolynomial(t, start, end, params_.trajectory_duration);
  }
}

double TurretControlPlugin::QuantizeAngle(double angle)
{
  // Convert to degrees, quantize, and convert back to radians
  double degrees = angle * 180.0 / M_PI;
  double quantized_degrees = std::round(degrees / params_.encoder_resolution) * params_.encoder_resolution;
  return quantized_degrees * M_PI / 180.0;
}

void TurretControlPlugin::PublishState()
{
    auto msg = std_msgs::msg::Float64();
    double angle_degrees = GetCurrentAngleDegrees();
    msg.data = AngleUtils::degreesToRadians(angle_degrees);
    state_pub_->publish(msg);

    RCLCPP_DEBUG(
        rclcpp::get_logger("turret_control"),
        "Publishing state - Degrees: %.2f°, Radians: %.2f rad",
        angle_degrees,
        msg.data);
}

void TurretControlPlugin::CheckHoldTimeout()
{
    if (current_state_ == TurretState::HOLDING_POSITION) {
        double current_time = model_->GetWorld()->SimTime().Double();
        if (current_time - last_command_time_ > params_.hold_time) {
            target_position_ = initial_reference_;
            start_position_ = joint_->Position(0);
            trajectory_start_time_ = current_time;
            is_trajectory_active_ = true;
            current_state_ = TurretState::RETURNING_HOME;
            
            RCLCPP_INFO(
                rclcpp::get_logger("turret_control"),
                "Hold timeout reached - returning to 0 degrees");
        }
    } else if (current_state_ == TurretState::MOVING_TO_TARGET) {
        last_command_time_ = model_->GetWorld()->SimTime().Double();
    }
}

void TurretControlPlugin::UpdateChild()
{
    CheckHoldTimeout();

    if (is_trajectory_active_) {
        double current_time = model_->GetWorld()->SimTime().Double();
        double trajectory_time = current_time - trajectory_start_time_;
        
        if (trajectory_time >= params_.trajectory_duration) {
            is_trajectory_active_ = false;
            joint_->SetPosition(0, target_position_);
            
            if (current_state_ == TurretState::MOVING_TO_TARGET) {
                current_state_ = TurretState::HOLDING_POSITION;
            } else if (current_state_ == TurretState::RETURNING_HOME) {
                current_state_ = TurretState::IDLE;
            }
        } else {
            double new_position = trajectory_calculator_->calculateTrajectory(
                trajectory_time,
                start_position_,
                target_position_,
                params_.trajectory_duration,
                params_.profile,
                params_.s_curve_steepness,
                params_.max_acceleration);
            joint_->SetPosition(0, new_position);
        }
    } else if (current_state_ == TurretState::IDLE) {
        double current_pos = joint_->Position(0);
        if (std::abs(current_pos - initial_reference_) > 0.001) {
            target_position_ = initial_reference_;
            start_position_ = current_pos;
            trajectory_start_time_ = model_->GetWorld()->SimTime().Double();
            is_trajectory_active_ = true;
            current_state_ = TurretState::RETURNING_HOME;
            
            RCLCPP_DEBUG(
                rclcpp::get_logger("turret_control"),
                "Detected deviation from 0, correcting position");
        } else {
            joint_->SetPosition(0, initial_reference_);
        }
    }
}

void TurretControlPlugin::InitializeJoint()
{
    if (joint_) {
        initial_reference_ = joint_->Position(0);
        current_position_ = initial_reference_;
        start_position_ = initial_reference_;
        target_position_ = initial_reference_;
        
        is_trajectory_active_ = true;
        trajectory_start_time_ = model_->GetWorld()->SimTime().Double();
        current_state_ = TurretState::RETURNING_HOME;
        
        RCLCPP_INFO(
            rclcpp::get_logger("turret_control"),
            "Initialized joint %s - moving to 0 degrees",
            params_.joint_name.c_str());
    }
}

double TurretControlPlugin::GetCurrentAngleDegrees() const {
    double relative_angle = joint_->Position(0) - initial_reference_;
    double degrees = AngleUtils::radiansToDegrees(relative_angle);
    degrees = AngleUtils::applyRotationDirection(degrees, params_.rotation_direction);
    degrees = fmod(degrees, 360.0);
    if (degrees < 0) {
        degrees += 360.0;
    }
    return std::round(degrees / params_.encoder_resolution) * params_.encoder_resolution;
}

rclcpp_action::GoalResponse TurretControlPlugin::handleGoal(
    const rclcpp_action::GoalUUID &, 
    std::shared_ptr<const TurretAction::Goal> goal)
{
    if (has_active_goal_) {
        RCLCPP_WARN(rclcpp::get_logger("turret_control"), "Rejecting goal, another goal is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(rclcpp::get_logger("turret_control"), "Received goal request: %.2f rad", goal->target_angle);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurretControlPlugin::handleCancel(
    const std::shared_ptr<GoalHandleTurret> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("turret_control"), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurretControlPlugin::handleAccepted(const std::shared_ptr<GoalHandleTurret> goal_handle)
{
    std::thread{std::bind(&TurretControlPlugin::executeGoal, this, goal_handle)}.detach();
}

void TurretControlPlugin::executeGoal(const std::shared_ptr<GoalHandleTurret> goal_handle)
{
    current_goal_handle_ = goal_handle;
    has_active_goal_ = true;

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TurretAction::Feedback>();
    auto result = std::make_shared<TurretAction::Result>();

    // Apply rotation direction to target
    double direction_adjusted_command = ApplyRotationDirection(goal->target_angle);
    target_position_ = initial_reference_ + direction_adjusted_command;
    
    trajectory_start_time_ = model_->GetWorld()->SimTime().Double();
    start_position_ = joint_->Position(0);
    last_command_time_ = trajectory_start_time_;
    is_trajectory_active_ = true;
    current_state_ = TurretState::MOVING_TO_TARGET;

    RCLCPP_INFO(
        rclcpp::get_logger("turret_control"),
        "Executing goal - moving to %.2f degrees",
        goal->target_angle * 180.0 / M_PI);

    while (rclcpp::ok() && is_trajectory_active_) {
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Goal canceled";
            goal_handle->canceled(result);
            has_active_goal_ = false;
            return;
        }

        // Update feedback
        feedback->current_angle = GetCurrentAngleDegrees() * M_PI / 180.0;
        feedback->remaining_angle = goal->target_angle - feedback->current_angle;
        feedback->state = current_state_ == TurretState::MOVING_TO_TARGET ? "MOVING" : 
                         current_state_ == TurretState::HOLDING_POSITION ? "HOLDING" : "IDLE";
        
        goal_handle->publish_feedback(feedback);

        if (!is_trajectory_active_) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (rclcpp::ok()) {
        result->success = true;
        result->final_angle = GetCurrentAngleDegrees() * M_PI / 180.0;
        result->message = "Goal reached successfully";
        goal_handle->succeed(result);
    }

    has_active_goal_ = false;
}

GZ_REGISTER_MODEL_PLUGIN(TurretControlPlugin)

}  // namespace gazebo_ros_control 