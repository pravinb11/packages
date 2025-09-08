#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <std_msgs/msg/string.hpp>

#include "action_bridge_pkg/action/fibonacci.hpp"


using namespace std::placeholders;


namespace action_tutorials_cpp_ns
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_bridge_pkg::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions &node_options) : Node("s_and_ac_node", node_options)
  {
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FibonacciActionClient::result_callback, this, _1);
    subscription_ = create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&FibonacciActionClient::topic_callback, this, _1));
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  // rclcpp_action::Client<action_tutorials_interfaces::action::Fibonacci>::SendGoalOptions send_goal_options;
  rclcpp_action::Client<Fibonacci>::SendGoalOptions send_goal_options;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

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

  void topic_callback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO(get_logger(), "Received: '%s'", msg.data.c_str());
    
    if(!client_ptr_->wait_for_action_server(std::chrono::milliseconds(100)))
    {
      RCLCPP_ERROR(get_logger(), "Action server is unavailable!");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    client_ptr_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(get_logger(), "Sent goal");
  }
};
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<action_tutorials_cpp_ns::FibonacciActionClient>(node_options));
  rclcpp::shutdown();
  return 0;
}