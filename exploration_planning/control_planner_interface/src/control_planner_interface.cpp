//
// Created by hjl on 2021/11/23.
//

#include "control_planner_interface/control_planner_interface.h"


namespace interface {

    // ControlPlannerInterface::ControlPlannerInterface(rclcpp::Node &nh, rclcpp::Node &nh_private,
    ControlPlannerInterface::ControlPlannerInterface(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private,
                                                     std::shared_ptr<PCIManager> &pci_manager)
            : nh_(nh), nh_private_(nh_private), pci_manager_(pci_manager){//,
              // planner_client_(nh_, "topo_planner", true) {
        planner_client_ = rclcpp_action::create_client<ExplPlanner>(nh_, "topo_planner");

        // odometry_sub_ = nh_.subscribe("odometry",1, &ControlPlannerInterface::odometryCallback, this);
        odometry_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("odometry", 1, std::bind(&ControlPlannerInterface::odometryCallback, this, std::placeholders::_1));

        if (!loadParams()) {
            RCLCPP_ERROR(rclcpp::get_logger("ControlPlannerInterface"), "control planner interface can not load params. Shut down ROS node.");
            // ros::shutdown();
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "control planner interface construct");
    }

    bool ControlPlannerInterface::loadParams() {

        // const std::string &ns = ros::this_node::getName();
        const std::string &ns = nh_->get_name();

        frame_id_ = "world";
        // if (!ros::param::get(ns + "/frame_id", frame_id_)) {
        if (!nh_->get_parameter(ns + "/frame_id", frame_id_)) {
            RCLCPP_WARN(rclcpp::get_logger("ControlPlannerInterface"), "No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        pci_manager_->setFrameId(frame_id_);

        if (!pci_manager_->loadParams(ns)) return false;

        return true;
    }

    void ControlPlannerInterface::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
        current_pose_ = odom->pose.pose;
        pci_manager_->setCurrentPose(current_pose_);

        pose_is_ready_ = true;
    }

    bool ControlPlannerInterface::init() {
        pose_is_ready_ = false;

        // checking odometry is ready.
        rclcpp::Rate rr(1);
        while (!pose_is_ready_) {
            RCLCPP_WARN(rclcpp::get_logger("ControlPlannerInterface"), "Waiting for odometry.");
            // rclcpp::spin_some(node);
            rclcpp::spin_some(nh_);
            rr.sleep();
        }

        if (!pci_manager_->initialize()) return false;

        return true;
    }

    void ControlPlannerInterface::goToWayPose(geometry_msgs::msg::Pose &pose) {
        pci_manager_->goToWaypoint(pose);
    }

    bool ControlPlannerInterface::isGoalReached() {
        return pci_manager_->isGoalReached();
    }

    void ControlPlannerInterface::cancelCurrentGoal() {
        pci_manager_->cancelCurrentGoal();
    }

    // void ControlPlannerInterface::executePath(const  std::vector<control_planner_interface::Path> &path_segments) {
    void ControlPlannerInterface::executePath(const  std::vector<control_planner_interface::msg::Path> &path_segments) {
        std::vector<geometry_msgs::msg::Pose> modified_path;
        pci_manager_->executePath(path_segments,modified_path);
    }

    // bool ControlPlannerInterface::callForPlanner(const int &iteration_id,  std::vector<control_planner_interface::Path> &path_segments) {
    bool ControlPlannerInterface::callForPlanner(const int &iteration_id,  std::vector<control_planner_interface::msg::Path> &path_segments) {
        // control_planner_interface::ExplorerPlannerGoal goal;
        // goal.iteration_id = iteration_id;
        RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "call for the path to explore..");
        // if (planner_client_.sendGoalAndWait(goal, rclcpp::Duration(60.0), rclcpp::Duration(10.0)) ==
        // if (planner_client_.sendGoalAndWait(goal, rclcpp::Duration::from_seconds(60.0), rclcpp::Duration::from_seconds(10.0)) == actionlib::SimpleClientGoalState::SUCCEEDED) {

            using namespace std::placeholders;

            if(!this->planner_client_->wait_for_action_server())
            {
              RCLCPP_ERROR(nh_->get_logger(), "Action server not available after waiting");
              rclcpp::shutdown();
            }

            auto goal = ExplPlanner::Goal();
            goal.iteration_id = iteration_id;

            RCLCPP_INFO(nh_->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<ExplPlanner>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&ControlPlannerInterface::goal_response_cb, this, _1);
            send_goal_options.feedback_callback = std::bind(&ControlPlannerInterface::feedback_cb, this, _1, _2);
            send_goal_options.result_callback = std::bind(&ControlPlannerInterface::result_cb, this, _1);
            this->planner_client_->async_send_goal(goal, send_goal_options);

        //     RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "the path get.");
        //     path_segments = planner_client_.getResult()->paths;
            return true;
        // } else {
        //     RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "planner timeout.");
        //     planner_client_.cancelAllGoals();
        //     return false;
        // }
    }

    void ControlPlannerInterface::goal_response_cb(const ExplPlannerCGH::SharedPtr &goal_handle)
    {
        // if(!goal_handle)
        //   RCLCPP_ERROR(this->get_logger(), "REJECT");
        // else
        //   RCLCPP_INFO(this->get_logger(), "ACCEPT");
    }

    void ControlPlannerInterface::feedback_cb(ExplPlannerCGH::SharedPtr, const std::shared_ptr<const ExplPlanner::Feedback> feedback)
    {
        // RCLCPP_INFO(this->get_logger(), "Item = %d", feedback->item);
    }

    void ControlPlannerInterface::result_cb(const ExplPlannerCGH::WrappedResult &result)
    {
        switch(result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "the path get.");
            // path_segments = result.result->paths;
            break;
          default:
            // RCLCPP_ERROR(this->get_logger(), "UNKNOWN");
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "Sequence = ");

        // for(auto item : result.result->sequence)    
        //   std::cout << item << " ";

        // std::cout << std::endl;
        // rclcpp::shutdown();
    }

    void ControlPlannerInterface::stopMove() {
        pci_manager_->stopMove();
    }
}