#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::placeholders;

class NavActionServer : public rclcpp::Node
{
public:
  using NavigateToPose = turtlesim::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit NavActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_server", options)
  {
    using namespace std::placeholders;

    this->tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "navigate_to_pose",
      std::bind(&NavActionServer::handle_goal, this, _1, _2),
      std::bind(&NavActionServer::handle_cancel, this, _1),
      std::bind(&NavActionServer::handle_accepted, this, _1));

    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    using namespace std::placeholders;

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&NavActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto & result = *goal_handle->get_result();

    rclcpp::Rate loop_rate(10);
    const double distance_tolerance = 0.25;
    const double angle_tolerance = 0.1;

    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_->lookupTransform(
          "turtle1",
          goal->pose.header.frame_id,
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "%s", ex.what());
        loop_rate.sleep();
        continue;
      }

      double x = transformStamped.transform.translation.x;
      double y = transformStamped.transform.translation.y;

      double distance_error = std::sqrt(std::pow(x - goal->pose.pose.position.x, 2) +
                                        std::pow(y - goal->pose.pose.position.y, 2));
      double angle_error = tf2::angles::shortest_angular_distance(
        tf2::getYaw(transformStamped.transform.rotation),
        tf2::getYaw(goal->pose.pose.orientation));

      geometry_msgs::msg::Twist msg;
      if (distance_error > distance_tolerance) {
        msg.linear.x = 0.5 * distance_error;
        msg.angular.z = 0.0;
      } else if (std::abs(angle_error) > angle_tolerance) {
        msg.linear.x = 0.0;
        msg.angular.z = 1.0 * angle_error;
      } else {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        break;
      }

      publisher_->publish(msg);
      feedback->distance_remaining = distance_error;
      goal_handle->publish_feedback(feedback);

      if (goal_handle->is_canceling()) {
        result.successful = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      loop_rate.sleep();
    }

    result.successful = true;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavActionServer>());
  rclcpp::shutdown();
  return 0;
}
