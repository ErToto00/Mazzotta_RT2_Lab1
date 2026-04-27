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
    using namespace std::placeholders;

    this->tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    this->action_server_ = rclcpp_action::create_server<Navigate>(
      this,
      "navigate",
      std::bind(&NavActionServer::handle_goal, this, _1, _2),
      std::bind(&NavActionServer::handle_cancel, this, _1),
      std::bind(&NavActionServer::handle_accepted, this, _1));

    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  }

private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with target x: %f, y: %f, theta: %f", goal->x, goal->y, goal->theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    using namespace std::placeholders;

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&NavActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();

    rclcpp::Rate loop_rate(10);
    const double distance_tolerance = 0.25;
    const double angle_tolerance = 0.1;

    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        // Look up the transform from the fixed frame ('odom' or 'world' or simply the parent frame for turtlesim)
        // In turtlesim, the global frame is usually implicitly 'world' and there might not be a published tf tree by default,
        // unless we use something like turtle_tf2_broadcaster. But assuming the user wants to lookup from 'world' to 'turtle1'.
        // Let's use lookupTransform from 'world' to 'turtle1'. But we don't have a broadcaster here.
        // Wait, does the prompt imply we have tf2 running? Yes, "It must use the tf2 library to monitor the robot's pose".
        // Often in RT2, there's a broadcaster assumed or they just look up "world" to "turtle1".
        transformStamped = tf_buffer_->lookupTransform(
          "world", // turtlesim root is usually without tf, but if there's tf, it's 'world' or 'odom'
          "turtle1", // target frame
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        // If tf is not available yet, log and sleep.
        // Note: For a pure turtlesim without tf broadcasters, this lookup will fail constantly unless we add a broadcaster.
        // To be safe, we'll keep this logic as requested: "use tf2 library to monitor the robot's pose".
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF: %s", ex.what());
        loop_rate.sleep();
        continue;
      }

      /* 
       * Explanation of Frame Transformation for Error Calculation:
       * The `lookupTransform` above gets the current pose of 'turtle1' relative to the 'world' frame.
       * `transformStamped.transform.translation` gives the current (x, y) coordinates.
       * `transformStamped.transform.rotation` gives the current orientation as a quaternion.
       * We compare these current global coordinates to the target (goal->x, goal->y, goal->theta)
       * which are also assumed to be in the 'world' frame, to calculate distance and angle errors.
       */

      double current_x = transformStamped.transform.translation.x;
      double current_y = transformStamped.transform.translation.y;

      double distance_error = std::sqrt(std::pow(goal->x - current_x, 2) +
                                        std::pow(goal->y - current_y, 2));

      // Calculate desired angle to reach the target point
      double target_angle = std::atan2(goal->y - current_y, goal->x - current_x);

      double diff = target_angle - tf2::getYaw(transformStamped.transform.rotation);
      double angle_error = std::atan2(std::sin(diff), std::cos(diff));

      geometry_msgs::msg::Twist msg;
      if (distance_error > distance_tolerance) {
        // Turn towards the goal if the angle error is significant, otherwise move forward
        if (std::abs(angle_error) > angle_tolerance) {
          msg.linear.x = 0.0;
          msg.angular.z = 2.0 * angle_error; // Proportional angular control
        } else {
          msg.linear.x = 1.0 * distance_error; // Proportional linear control
          msg.angular.z = 0.0;
        }
      } else {
        // Goal position reached. Adjust to final theta if needed.
        double final_diff = goal->theta - tf2::getYaw(transformStamped.transform.rotation);
        double final_angle_error = std::atan2(std::sin(final_diff), std::cos(final_diff));
          
        if (std::abs(final_angle_error) > angle_tolerance) {
          msg.linear.x = 0.0;
          msg.angular.z = 2.0 * final_angle_error;
        } else {
          msg.linear.x = 0.0;
          msg.angular.z = 0.0;
          publisher_->publish(msg);
          break; // Succeeded
        }
      }

      publisher_->publish(msg);
      feedback->distance_remaining = distance_error;
      goal_handle->publish_feedback(feedback);

      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        // Stop the robot
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
        return;
      }

      loop_rate.sleep();
    }

    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

} // namespace nav_action_server_lib

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_server_lib::NavActionServer)
