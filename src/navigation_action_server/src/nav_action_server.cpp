#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "navigation_interfaces/action/navigate.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::placeholders;

namespace nav_action_server_lib
{

class NavActionServer : public rclcpp::Node
{
public:
  using Navigate = navigation_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  explicit NavActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_server", options)
  {
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    this->action_server_ = rclcpp_action::create_server<Navigate>(
      this, "navigate",
      std::bind(&NavActionServer::handle_goal, this, _1, _2),
      std::bind(&NavActionServer::handle_cancel, this, _1),
      std::bind(&NavActionServer::handle_accepted, this, _1));

    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%f, y=%f", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    std::thread{std::bind(&NavActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        // FIXED: Using 'world' as global origin and 'mogi_bot/link_chassis' as the target[cite: 3]
        transformStamped = tf_buffer_->lookupTransform(
          "world", 
          "mogi_bot/link_chassis", 
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF: %s", ex.what());
        loop_rate.sleep();
        continue;
      }

      double current_x = transformStamped.transform.translation.x;
      double current_y = transformStamped.transform.translation.y;
      double dist_err = std::sqrt(std::pow(goal->x - current_x, 2) + std::pow(goal->y - current_y, 2));
      double target_angle = std::atan2(goal->y - current_y, goal->x - current_x);
      double cur_yaw = tf2::getYaw(transformStamped.transform.rotation);
      double angle_err = std::atan2(std::sin(target_angle - cur_yaw), std::cos(target_angle - cur_yaw));

      geometry_msgs::msg::Twist msg;
      if (dist_err > 0.25) {
        if (std::abs(angle_err) > 0.1) {
          msg.angular.z = 1.0 * angle_err;
          msg.linear.x = 0.0;
        } else {
          msg.linear.x = 0.5 * dist_err;
          msg.angular.z = 0.0;
        }
      } else {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
        break;
      }

      publisher_->publish(msg);
      feedback->distance_remaining = dist_err;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_server_lib::NavActionServer)