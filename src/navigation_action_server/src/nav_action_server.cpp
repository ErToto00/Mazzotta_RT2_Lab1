#include <memory>
#include <thread>
#include <cmath>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "navigation_interfaces/action/navigate.hpp"
#include "nav_msgs/msg/odometry.hpp" // NEW: For direct odometry
#include "tf2/utils.h"

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
    // Subscribe directly to /odom to get position, bypassing TF entirely
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&NavActionServer::odom_callback, this, _1));

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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Variables to store current position
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  bool odom_received_ = false;
  std::mutex odom_mutex_; // Protects the variables between threads

  // NEW: Callback that updates position every time Gazebo sends an /odom message
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    odom_received_ = true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "New goal received: x=%f, y=%f", goal->x, goal->y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    std::thread{std::bind(&NavActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Starting navigation loop...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      double cx, cy, cyaw;

      // Safely grab the latest position from our Odometry callback
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!odom_received_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for /odom messages...");
          loop_rate.sleep();
          continue;
        }
        cx = current_x_;
        cy = current_y_;
        cyaw = current_yaw_;
      }

      // Calculate errors using the Odometry data
      double dist_err = std::sqrt(std::pow(goal->x - cx, 2) + std::pow(goal->y - cy, 2));
      double target_angle = std::atan2(goal->y - cy, goal->x - cx);
      double angle_err = std::atan2(std::sin(target_angle - cyaw), std::cos(target_angle - cyaw));

      geometry_msgs::msg::Twist msg;
      if (dist_err > 0.25) {
        if (std::abs(angle_err) > 0.1) {
          msg.angular.z = 1.2 * angle_err;
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
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
  }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_server_lib::NavActionServer)