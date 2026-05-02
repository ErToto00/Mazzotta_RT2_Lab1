#include <memory>
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "navigation_interfaces/action/navigate.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::placeholders;

namespace nav_action_client_lib
{

/* 
 * NavActionClient Class:
 * Implements the Action client to send navigation requests.
 * Manages user interaction from the command line (CLI) via a dedicated thread.
 */
class NavActionClient : public rclcpp::Node
{
public:
  using Navigate = navigation_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

  explicit NavActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("nav_action_client", options)
  {
    /* Creates an Action client to interface with the "navigate" server */
    this->client_ptr_ = rclcpp_action::create_client<Navigate>(
      this,
      "navigate");

    this->shutdown_pub_ = this->create_publisher<std_msgs::msg::Empty>("/shutdown", 10);

    /* Starts a separate thread for CLI interaction */
    cli_thread_ = std::thread(&NavActionClient::cli_loop, this);
  }

  ~NavActionClient()
  {
    if (cli_thread_.joinable()) {
      cli_thread_.detach();
    }
  }

private:
  rclcpp_action::Client<Navigate>::SharedPtr client_ptr_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr shutdown_pub_;
  std::thread cli_thread_;
  std::shared_ptr<GoalHandleNavigate> current_goal_handle_;

  /* Function executed in the separate thread: handles terminal input */
  void cli_loop()
  {
    while (rclcpp::ok()) {
      std::cout << "\n--- Navigation CLI ---\n";
      std::cout << "Enter 'g' to send a new goal, 'c' to cancel current goal, or 'q' to quit:\n";
      
      std::string input;
      if (!std::getline(std::cin, input)) {
        continue;
      }

      // Checks if the user requested to cancel the current goal
      if (input == "c" || input == "C") {
        if (current_goal_handle_) {
          RCLCPP_INFO(this->get_logger(), "Sending cancel request...");
          client_ptr_->async_cancel_goal(current_goal_handle_);
          current_goal_handle_ = nullptr;
        } else {
          RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
        }
        continue;
      }

      // Exit logic
      if (input == "q" || input == "Q") {
        if (current_goal_handle_) {
          RCLCPP_INFO(this->get_logger(), "Canceling goal before exiting...");
          client_ptr_->async_cancel_goal(current_goal_handle_);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_INFO(this->get_logger(), "Sending shutdown signal to server...");
        shutdown_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        RCLCPP_INFO(this->get_logger(), "Exiting...");
        rclcpp::shutdown();
        break;
      }

      if (input == "g" || input == "G"){
        std::cout << "Enter <x y theta> (e.g., 5.0 2.5 0.0):\n";
        
        std::string coord_input;
        if (std::getline(std::cin, coord_input)) {
          double x, y, theta;
          if (sscanf(coord_input.c_str(), "%lf %lf %lf", &x, &y, &theta) == 3) {
            send_goal(x, y, theta);
          } else {
            std::cout << "Invalid input format. Expected three numbers.\n";
          }
        }
      }
    }
  }

  // Sends the goal request to the server
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
    send_goal_options.result_callback =
      std::bind(&NavActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  // Callback: Handles initial response
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

  // Callback: Executed upon goal completion
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