#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/action/navigate_to_pose.hpp"

using namespace std::placeholders;

class NavActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = turtlesim::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&NavActionClient::send_goal, this));
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_sent_{false};

  void send_goal()
  {
    using namespace std::placeholders;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not ready");
      return;
    }

    if (goal_sent_) {
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "turtle1";
    goal_msg.pose.pose.position.x = 5.0;
    goal_msg.pose.pose.position.y = 5.0;
    goal_msg.pose.pose.orientation.z = sin(M_PI / 4.0);
    goal_msg.pose.pose.orientation.w = cos(M_PI / 4.0);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavActionClient::result_callback, this, _1);

    auto future_goal_handle = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    const auto & success = result.result->successful;
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Result received: %d", success);
    } else {
      RCLCPP_WARN(this->get_logger(), "Navigation failed");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<NavActionClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}
