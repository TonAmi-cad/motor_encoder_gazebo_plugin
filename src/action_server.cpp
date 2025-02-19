#include "gazebo_ros_control/action_server.hpp"
#include <thread>
#include <rclcpp/rclcpp.hpp>

namespace gazebo_ros_control
{

TurretActionServer::TurretActionServer(
    rclcpp::Node::SharedPtr node,
    const std::string& action_name,
    std::function<double()> get_current_angle_cb,
    std::function<void(double)> set_target_position_cb)
    : node_(node),
      action_name_(action_name),
      has_active_goal_(false),
      get_current_angle_(get_current_angle_cb),
      set_target_position_(set_target_position_cb)
{
}

void TurretActionServer::initialize()
{
    action_server_ = rclcpp_action::create_server<TurretAction>(
        node_,
        action_name_,
        std::bind(&TurretActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TurretActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&TurretActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Action server started");
}

rclcpp_action::GoalResponse TurretActionServer::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TurretAction::Goal> goal)
{
    if (has_active_goal_) {
        RCLCPP_WARN(node_->get_logger(), "Rejecting goal, another goal is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(node_->get_logger(), "Received goal request: %.2f rad", goal->target_angle);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurretActionServer::handleCancel(
    const std::shared_ptr<GoalHandleTurret>)
{
    RCLCPP_INFO(node_->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurretActionServer::handleAccepted(const std::shared_ptr<GoalHandleTurret> goal_handle)
{
    std::thread{std::bind(&TurretActionServer::executeGoal, this, goal_handle)}.detach();
}

void TurretActionServer::executeGoal(const std::shared_ptr<GoalHandleTurret> goal_handle)
{
    current_goal_handle_ = goal_handle;
    has_active_goal_ = true;

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TurretAction::Feedback>();
    auto result = std::make_shared<TurretAction::Result>();

    // Set target position through callback
    set_target_position_(goal->target_angle);

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Goal canceled";
            goal_handle->canceled(result);
            has_active_goal_ = false;
            return;
        }

        // Get current angle through callback
        double current_angle = get_current_angle_();
        
        feedback->current_angle = current_angle;
        feedback->remaining_angle = goal->target_angle - current_angle;
        
        goal_handle->publish_feedback(feedback);

        if (std::abs(feedback->remaining_angle) < 0.01) {  // Close enough to target
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (rclcpp::ok()) {
        result->success = true;
        result->final_angle = get_current_angle_();
        result->message = "Goal reached successfully";
        goal_handle->succeed(result);
    }

    has_active_goal_ = false;
}

}  // namespace gazebo_ros_control 