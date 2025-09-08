/*
Last updated: 
Last updated on: 17/07/2025
Last updated by: Ratijit
Author: Ratijit Mitra
*/


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// ================================================== Actions
#include <nav2_msgs/action/follow_path.hpp>
// ================================================== Messages
// #include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>


using namespace std::placeholders;


namespace action_bridge_ns
{
class ActionBridge : public rclcpp::Node
{
public:
  using FP = nav2_msgs::action::FollowPath;
  using GH_FP = rclcpp_action::ClientGoalHandle<FP>;

  explicit ActionBridge(const rclcpp::NodeOptions &node_options) : Node("action_bridge_node", node_options)
  {
    ac_ptr = rclcpp_action::create_client<FP>(this, "follow_path");

    send_goal_options = rclcpp_action::Client<FP>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&ActionBridge::goal_response_cb, this, _1);
    send_goal_options.feedback_callback = std::bind(&ActionBridge::feedback_cb, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ActionBridge::result_cb, this, _1);
    
    // odometry_sub = create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&ActionBridge::odometry_cb, this, _1));
    // wp_sub = create_subscription<geometry_msgs::msg::PointStamped>("/way_point", 10, std::bind(&ActionBridge::waypoint_cb, this, _1));
    path_sub = create_subscription<nav_msgs::msg::Path>("/path", 1, std::bind(&ActionBridge::path_cb, this, _1));
  }

private:
  rclcpp_action::Client<FP>::SharedPtr ac_ptr;    // Action Client
  rclcpp_action::Client<FP>::SendGoalOptions send_goal_options;
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
  // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr wp_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
  geometry_msgs::msg::Point cur_pose;

  void goal_response_cb(const GH_FP::SharedPtr &goal_res)
  {
    if(!goal_res)
      RCLCPP_ERROR(get_logger(), "ActionServer: G!");
  }

  void feedback_cb(GH_FP::SharedPtr , const std::shared_ptr<const FP::Feedback> fb)
  {
    // RCLCPP_INFO(get_logger(), "ActionServer: FB");
  }

  void result_cb(const GH_FP::WrappedResult &result)
  {
    switch(result.code)
    {
      // case rclcpp_action::ResultCode::SUCCEEDED:
      //   RCLCPP_INFO(get_logger(), "ActionServer: SUCCEEDED");
      //   break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(get_logger(), "ActionServer: ABORTED");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "ActionServer: CANCELED");
        break;
      default:
        RCLCPP_INFO(get_logger(), "ActionServer: Unknown");
        break;
    }
  }

  // void odometry_cb(const nav_msgs::msg::Odometry &msg)
  // {
  //   cur_pose = msg.pose.pose.position;
    // RCLCPP_INFO(get_logger(), "ActionClient: Odometry = %f, %f, %f", cur_pose.x, cur_pose.y, cur_pose.z);
  // }

  // void waypoint_cb(const geometry_msgs::msg::PointStamped &msg)
  // {
  //   if(!ac_ptr->wait_for_action_server(std::chrono::milliseconds(100)))
  //     RCLCPP_ERROR(get_logger(), "Action Server!");
  //   else
  //   {
  //     nav_msgs::msg::Path path_tmp;
  //     path_tmp.header.stamp = msg.header.stamp;
  //     path_tmp.header.frame_id = "odom";

  //     geometry_msgs::msg::PoseStamped pose_tmp;   // Current Pose
  //     pose_tmp.header.stamp = msg.header.stamp;
  //     pose_tmp.header.frame_id = "odom";
  //     pose_tmp.pose.position = cur_pose;
  //     path_tmp.poses.push_back(pose_tmp);

  //     pose_tmp.pose.position = msg.point;   // Goal Pose
  //     path_tmp.poses.push_back(pose_tmp);

  //     auto goal_msg = FP::Goal();
  //     goal_msg.path = path_tmp;

  //     ac_ptr->async_send_goal(goal_msg, send_goal_options);
  //     RCLCPP_INFO(this->get_logger(), "ActionClient: G...");
  //   }
  // }

  void path_cb(const nav_msgs::msg::Path &msg)
  {
    if(!ac_ptr->wait_for_action_server(std::chrono::milliseconds(100)))
      RCLCPP_ERROR(get_logger(), "Action Server!");
    else
    {
      auto goal_msg = FP::Goal();
      goal_msg.path = msg;

      ac_ptr->async_send_goal(goal_msg, send_goal_options);
      RCLCPP_INFO(this->get_logger(), "ActionClient: G...");
    }
  }
};
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<action_bridge_ns::ActionBridge>(node_options));
  rclcpp::shutdown();
  return 0;
}