#include <sstream>

#include "action_bridge_pkg/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp_ns
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_bridge_pkg::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions &options) : Node("ac_node", options)
  {
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    timer_->cancel();

    if (!client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(get_logger(), "Action server is unavailable!");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FibonacciActionClient::result_callback, this, _1);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(get_logger(), "Sent goal");
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr &goal_handle)
  {
    if(!goal_handle)
      RCLCPP_ERROR(get_logger(), "Goal was rejected!");
    // else
    //   RCLCPP_INFO(get_logger(), "Goal was accepted");
  }

  void feedback_callback(GoalHandleFibonacci::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";

    for(auto number : feedback->partial_sequence)
      ss << number << " ";
    
    RCLCPP_INFO(get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted!");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled!");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code!");
        return;
    }

    std::stringstream ss;
    ss << "Result received: ";
    
    for(auto number : result.result->sequence)
      ss << number << " ";
    
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp_ns::FibonacciActionClient)