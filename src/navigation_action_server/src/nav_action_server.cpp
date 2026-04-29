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

/* 
 * NavActionServer Class:
 * Implements the Action server for navigation.
 * Handles requests to move the turtle and publishes velocity commands (cmd_vel),
 * continuously monitoring the current position and orientation via TF2.
 */
class NavActionServer : public rclcpp::Node
{
public:
  using Navigate = navigation_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  explicit NavActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_server", options)
  {
    using namespace std::placeholders;

    // Initializes the TF2 buffer and listener to listen for and store spatial transformations (tf)
    this->tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    // Creates the Action server named "navigate" providing the bindings for the necessary callback functions 
    this->action_server_ = rclcpp_action::create_server<Navigate>(
      this,
      "navigate",
      std::bind(&NavActionServer::handle_goal, this, _1, _2),
      std::bind(&NavActionServer::handle_cancel, this, _1),
      std::bind(&NavActionServer::handle_accepted, this, _1));

    // Creates the publisher to send velocity commands (cmd_vel) to the simulated robot 
    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Callback: Invoked when a client sends a new goal. Verifies whether to accept or reject it. 
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with target x: %f, y: %f, theta: %f", goal->x, goal->y, goal->theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Callback: Invoked when the client requests to cancel an ongoing goal. 
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Callback: Invoked as soon as a goal is successfully accepted (from handle_goal). 
  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    using namespace std::placeholders;

    // Launches the actual execution in a separate thread to avoid blocking the ROS node scheduler 
    std::thread{std::bind(&NavActionServer::execute, this, _1), goal_handle}.detach();
  }

  // Function executed in the new thread: implements the control loop (reads tf, calculates error, publishes velocity) 
  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();

    // Execution frequency of the control loop (10 Hz)
    rclcpp::Rate loop_rate(10);
    // Tolerance thresholds to consider the goal (distance and orientation) as reached
    const double distance_tolerance = 0.25;
    const double angle_tolerance = 0.1;

    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        /* Asks TF2 to retrieve the most recent known position/orientation
         * of 'link_chassis' relative to the global 'odom' reference frame. */
        transformStamped = tf_buffer_->lookupTransform(
          "odom",
          "base_link", // target frame
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        // If the transformation is not yet available in the tf tree, waits and runs a new loop 
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF: %s", ex.what());
        if (ex.what() == std::string("Could not find a connection between 'base_link' and 'odom' because they are not part of the same tree.")) {
          RCLCPP_ERROR(this->get_logger(), "Frame 'odom' does not exist.");
          return;
        }
        loop_rate.sleep();
        continue;
      }


      // Extracts the robot's current (x, y) position from the calculated transformation 
      double current_x = transformStamped.transform.translation.x;
      double current_y = transformStamped.transform.translation.y;

      // Calculation of the Euclidean distance error between the turtle and the desired point 
      double distance_error = std::sqrt(std::pow(goal->x - current_x, 2) +
                                        std::pow(goal->y - current_y, 2));

      // Calculation of the angle the turtle must assume to point towards the desired coordinates (atan2(dy, dx))
      double target_angle = std::atan2(goal->y - current_y, goal->x - current_x);

      // Calculation of the angular error by measuring the difference with respect to the current orientation (yaw)
      double diff = target_angle - tf2::getYaw(transformStamped.transform.rotation);
      // Normalizes the angle to always keep it within [-pi, pi]
      double angle_error = std::atan2(std::sin(diff), std::cos(diff));

      geometry_msgs::msg::Twist msg;
      if (distance_error > distance_tolerance) {
        /* Basic proportional control to reach the desired (x, y) position:
         * If the angular error is still significant, stops linear motion and rotates towards the point.
         * Otherwise, advances linearly towards the point. */
        if (std::abs(angle_error) > angle_tolerance) {
          msg.linear.x = 0.0;
          msg.angular.z = 2.0 * angle_error; // Proportional angular control
        } else {
          msg.linear.x = 1.0 * distance_error; // Proportional linear control
          msg.angular.z = 0.0;
        }
      } else {
        // Position (x, y) successfully reached. Aligns the turtle to the final theta angle. 
        double final_diff = goal->theta - tf2::getYaw(transformStamped.transform.rotation);
        double final_angle_error = std::atan2(std::sin(final_diff), std::cos(final_diff));
          
        if (std::abs(final_angle_error) > angle_tolerance) {
          msg.linear.x = 0.0;
          msg.angular.z = 2.0 * final_angle_error;
        } else {
          // The final orientation is also correct. Interrupt the action by publishing zero velocity. 
          msg.linear.x = 0.0;
          msg.angular.z = 0.0;
          publisher_->publish(msg);
          break; // Goal successfully completed
        }
      }

      // Publishes the calculated Twist commands
      publisher_->publish(msg);
      // Sends a feedback update to the client with the remaining distance 
      feedback->distance_remaining = distance_error;
      goal_handle->publish_feedback(feedback);

      // Asynchronously checks if a cancellation request arrived from the client while in the loop
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

}

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_server_lib::NavActionServer)
