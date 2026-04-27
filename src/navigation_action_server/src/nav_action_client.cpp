#include <memory>
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "navigation_interfaces/action/navigate.hpp"

using namespace std::placeholders;

namespace nav_action_client_lib
{

class NavActionClient : public rclcpp::Node
{
public:
  using Navigate = navigation_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

  explicit NavActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Navigate>(
      this,
      "navigate");

    // Start CLI thread
    cli_thread_ = std::thread(&NavActionClient::cli_loop, this);
  }

  ~NavActionClient()
  {
    if (cli_thread_.joinable()) {
      // Forcing standard input close isn't trivial here, but we can detach
      cli_thread_.detach();
    }
  }

private:
  rclcpp_action::Client<Navigate>::SharedPtr client_ptr_;
  std::thread cli_thread_;
  std::shared_ptr<GoalHandleNavigate> current_goal_handle_;

  void cli_loop()
  {
    while (rclcpp::ok()) {
      std::cout << "\n--- Navigation CLI ---\n";
      std::cout << "Enter 'x y theta' to send a new target, or 'cancel' to cancel current goal: ";
      
      std::string input;
      if (!std::getline(std::cin, input)) {
        continue;
      }

      if (input == "cancel") {
        if (current_goal_handle_) {
          RCLCPP_INFO(this->get_logger(), "Sending cancel request...");
          client_ptr_->async_cancel_goal(current_goal_handle_);
          current_goal_handle_ = nullptr;
        } else {
          RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
        }
        continue;
      }

      double x, y, theta;
      if (sscanf(input.c_str(), "%lf %lf %lf", &x, &y, &theta) == 3) {
        send_goal(x, y, theta);
      } else {
        std::cout << "Invalid input. Please provide three numbers separated by spaces (e.g., '5.0 5.0 0.0').\n";
      }
    }
  }

  void send_goal(double x, double y, double theta)
  {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Navigate::Goal();
    goal_msg.x = x;
    goal_msg.y = y;
    goal_msg.theta = theta;

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%f, y=%f, theta=%f", x, y, theta);

    auto send_goal_options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavActionClient::result_callback, this, _1);

    auto future_goal_handle = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleNavigate::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      current_goal_handle_ = nullptr;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      current_goal_handle_ = goal_handle;
    }
  }

  void feedback_callback(
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const Navigate::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigate::WrappedResult & result)
  {
    current_goal_handle_ = nullptr;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
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

    if (result.result->success) {
      RCLCPP_INFO(this->get_logger(), "Navigation successfully finished.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Navigation failed.");
    }
  }
};

} // namespace nav_action_client_lib

RCLCPP_COMPONENTS_REGISTER_NODE(nav_action_client_lib::NavActionClient)
