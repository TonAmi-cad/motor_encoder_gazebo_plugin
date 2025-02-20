#ifndef GAZEBO_ROS_CONTROL__TURRET_CONTROL_PLUGIN_HPP_
#define GAZEBO_ROS_CONTROL__TURRET_CONTROL_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "gazebo_ros_control/turret_control_config.hpp"
#include "link_msgs/action/turret_control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "gazebo_ros_control/action_server.hpp"
#include "gazebo_ros_control/trajectory_calculator.hpp"
#include "gazebo_ros_control/angle_utils.hpp"

namespace gazebo_ros_control
{

struct TurretControlParameters {
  std::string joint_name = "";
  std::string command_topic = config::DEFAULT_COMMAND_TOPIC;
  std::string state_topic = config::DEFAULT_STATE_TOPIC;
  double trajectory_duration = config::DEFAULT_TRAJECTORY_DURATION;
  double max_acceleration = config::DEFAULT_MAX_ACCELERATION;
  double max_velocity = config::DEFAULT_MAX_VELOCITY;
  double s_curve_steepness = config::DEFAULT_S_CURVE_STEEPNESS;
  double initial_position = config::DEFAULT_INITIAL_POSITION;
  double encoder_resolution = config::DEFAULT_ENCODER_RESOLUTION;
  double hold_time = config::DEFAULT_HOLD_TIME;
  AccelerationProfile profile = config::DEFAULT_PROFILE;
  RotationDirection rotation_direction = config::DEFAULT_ROTATION_DIRECTION;
  std::string action_name = config::DEFAULT_ACTION_NAME;
};

class TurretControlPlugin : public gazebo::ModelPlugin
{
public:
  TurretControlPlugin();
  virtual ~TurretControlPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void LoadParameters(sdf::ElementPtr sdf);
  void OnTwistCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
  void OnAngleCommand(const std_msgs::msg::Float64::SharedPtr msg);
  void UpdateChild();
  void InitializeJoint();
  void PublishState();
  double QuantizeAngle(double angle);
  void CheckHoldTimeout();
  
  // Trajectory generation function
  double CalculateTrajectory(double t, double start, double end);

  // Helper functions for parameter loading
  static void LoadParameter(sdf::ElementPtr sdf, const std::string& name, std::string& param);
  static void LoadParameter(sdf::ElementPtr sdf, const std::string& name, double& param);
  static void LoadAccelerationProfile(sdf::ElementPtr sdf, AccelerationProfile& profile);
  static void LoadRotationDirection(sdf::ElementPtr sdf, RotationDirection& direction);

  double ApplyRotationDirection(double position) const;

  // Add new helper function
  double NormalizeAngle(double angle);

  // Add new helper functions
  double GetCurrentAngleDegrees() const;

  // Action server type definitions
  using TurretAction = link_msgs::action::TurretControl;
  using GoalHandleTurret = rclcpp_action::ServerGoalHandle<TurretAction>;

  // Action server callbacks
  rclcpp_action::Server<TurretAction>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TurretAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleTurret> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleTurret> goal_handle);
  void executeGoal(const std::shared_ptr<GoalHandleTurret> goal_handle);

  // Plugin parameters
  TurretControlParameters params_;

  // Gazebo objects
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_connection_;
  
  // ROS objects
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  
  // State variables
  double target_velocity_;
  double current_velocity_;
  double start_velocity_;
  double trajectory_start_time_;
  bool is_trajectory_active_;
  double target_position_;
  double current_position_;
  double start_position_;
  double last_command_time_;
  TurretState current_state_;

  // Add new member variable
  double initial_reference_;  // Store initial position as reference

  std::shared_ptr<GoalHandleTurret> current_goal_handle_;
  bool has_active_goal_;

  std::shared_ptr<TrajectoryCalculator> trajectory_calculator_;

  // Добавляем новые члены для ноды жизни
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  bool is_initialized_;
  const double HEALTH_CHECK_PERIOD = 1.0; // секунды

  // Добавляем новые методы
  void InitializeHealthCheck();
  void PublishHealth();
};
}  // namespace gazebo_ros_control

#endif  // GAZEBO_ROS_CONTROL__TURRET_CONTROL_PLUGIN_HPP_ 