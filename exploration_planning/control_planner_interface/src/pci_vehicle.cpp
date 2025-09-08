//
// Created by hjl on 2021/11/30.
//

#include "control_planner_interface/pci_vehicle.h"

namespace interface {

    // PCIVehicle::PCIVehicle(const rclcpp::Node &nh, const rclcpp::Node &nh_private) : PCIManager(nh, nh_private),
    PCIVehicle::PCIVehicle(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private) : PCIManager(nh, nh_private),
                                                                                           // vehicle_execute_client_(nh_, "vehicle_execute", true),
                                                                                           // vehicle_stop_client_(nh_, "vehicle_stop", true),
                                                                                           n_seq_(0) {
        nh_ = nh;
        vehicle_execute_client_ = rclcpp_action::create_client<VehExec>(nh_, "vehicle_execute");
        is_goal_reached = false;
        cgh = nullptr;
        vehicle_stop_client_ = rclcpp_action::create_client<VehExec>(nh_, "vehicle_stop");

        RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "pci vehicle construct");

    }

    bool PCIVehicle::loadParams(const std::string ns) {

        return true;
    }

    bool PCIVehicle::initialize() {
        pci_status_ = PCIStatus::kReady;

        return true;
    }

    void PCIVehicle::setCurrentPose(const geometry_msgs::msg::Pose &pose) {
        current_pose_.position.x = pose.position.x;
        current_pose_.position.y = pose.position.y;
        current_pose_.position.z = pose.position.z;
        current_pose_.orientation.x = pose.orientation.x;
        current_pose_.orientation.y = pose.orientation.y;
        current_pose_.orientation.z = pose.orientation.z;
        current_pose_.orientation.w = pose.orientation.w;
    }

    void PCIVehicle::setFrameId(const std::string &frame_id) {
        frame_id_ = frame_id;
    }

    // bool PCIVehicle::executePath(const std::vector<control_planner_interface::Path> &path_segments,
    bool PCIVehicle::executePath(const std::vector<control_planner_interface::msg::Path> &path_segments,
                                 std::vector<geometry_msgs::msg::Pose> &modified_path) {
       
        n_seq_++;
        // control_planner_interface::VehicleExecuteGoal path_goal;
        // path_goal.header.seq = n_seq_;
        // path_goal.header.stamp = rclcpp::Time::now();
        // path_goal.header.frame_id = frame_id_;
        // path_goal.paths = path_segments;
        // vehicle_execute_client_.sendGoal(path_goal);

        using namespace std::placeholders;

        if(!this->vehicle_execute_client_->wait_for_action_server())
        {
          RCLCPP_ERROR(rclcpp::get_logger("ControlPlannerInterface"), "Action server not available after waiting");
          rclcpp::shutdown();
        }

        auto path_goal = VehExec::Goal();
        // path_goal.header.seq = n_seq_;
        // path_goal.header.stamp = rclcpp::Time::now();
        path_goal.header.stamp = nh_->get_clock()->now();
        path_goal.header.frame_id = frame_id_;
        path_goal.paths = path_segments;

        RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<VehExec>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&PCIVehicle::goal_response_cb, this, _1);
        send_goal_options.feedback_callback = std::bind(&PCIVehicle::feedback_cb, this, _1, _2);
        send_goal_options.result_callback = std::bind(&PCIVehicle::result_cb, this, _1);
        this->vehicle_execute_client_->async_send_goal(path_goal, send_goal_options);

        // rclcpp::spin_some(node);
        rclcpp::spin_some(nh_);

        return true;
    }

    void PCIVehicle::goal_response_cb(const VehExecCGH::SharedPtr &goal_handle)
    {
        if(!goal_handle)
            // RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            cgh = nullptr;
        else
            // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            cgh = goal_handle;
    }

    void PCIVehicle::feedback_cb(VehExecCGH::SharedPtr, const std::shared_ptr<const VehExec::Feedback> feedback)
    {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";

    // for(auto number : feedback->partial_sequence)
    //   ss << number << " ";

    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void PCIVehicle::result_cb(const VehExecCGH::WrappedResult &result)
    {
        is_goal_reached = false;

        switch(result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            is_goal_reached = true;
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("ControlPlannerInterface"), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("ControlPlannerInterface"), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(rclcpp::get_logger("ControlPlannerInterface"), "Unknown result code");
            return;
        }

        // std::stringstream ss;
        // ss << "Result received: ";

        // for(auto number : result.result->sequence)
        //   ss << number << " ";

        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        // rclcpp::shutdown();
    }

    bool PCIVehicle::goToWaypoint(const geometry_msgs::msg::Pose &pose) {
        // control_planner_interface::Path path;
        control_planner_interface::msg::Path path;
        path.path.push_back(current_pose_);
        path.path.push_back(pose);
        // std::vector<control_planner_interface::Path> path_segments;
        std::vector<control_planner_interface::msg::Path> path_segments;
        path_segments.push_back(path);
        std::vector<geometry_msgs::msg::Pose> modified_path;
        executePath(path_segments,modified_path);

        return true;
    }

    bool PCIVehicle::isGoalReached() {
        // if (vehicle_execute_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        if(is_goal_reached)
            return true;
        else {
            return false;
        }
    }

    void PCIVehicle::cancelCurrentGoal() {
        // vehicle_execute_client_.cancelGoal();
        vehicle_execute_client_->async_cancel_goal(cgh);
    }

    void PCIVehicle::stopMove() {
        // control_planner_interface::VehicleExecuteGoal goal;
        // goal.paths.clear();
        // vehicle_stop_client_.sendGoal(goal);

        using namespace std::placeholders;

        if(!this->vehicle_stop_client_->wait_for_action_server())
        {
          RCLCPP_ERROR(rclcpp::get_logger("ControlPlannerInterface"), "Action server not available after waiting");
          rclcpp::shutdown();
        }

        auto goal = VehExec::Goal();
        goal.paths.clear();

        RCLCPP_INFO(rclcpp::get_logger("ControlPlannerInterface"), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<VehExec>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&PCIVehicle::goal_response_2_cb, this, _1);
        send_goal_options.feedback_callback = std::bind(&PCIVehicle::feedback_2_cb, this, _1, _2);
        send_goal_options.result_callback = std::bind(&PCIVehicle::result_2_cb, this, _1);
        this->vehicle_stop_client_->async_send_goal(goal, send_goal_options);
    }

    void PCIVehicle::goal_response_2_cb(const VehExecCGH::SharedPtr &goal_handle)
    {
        // if(!goal_handle)
            // RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        // else
            // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }

    void PCIVehicle::feedback_2_cb(VehExecCGH::SharedPtr, const std::shared_ptr<const VehExec::Feedback> feedback)
    {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";

    // for(auto number : feedback->partial_sequence)
    //   ss << number << " ";

    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void PCIVehicle::result_2_cb(const VehExecCGH::WrappedResult &result)
    {
        // switch(result.code)
        // {
        //   case rclcpp_action::ResultCode::SUCCEEDED:
        //     break;
        //   case rclcpp_action::ResultCode::ABORTED:
        //     RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        //     return;
        //   case rclcpp_action::ResultCode::CANCELED:
        //     RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        //     return;
        //   default:
        //     RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        //     return;
        // }

        // std::stringstream ss;
        // ss << "Result received: ";

        // for(auto number : result.result->sequence)
        //   ss << number << " ";

        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        // rclcpp::shutdown();
    }
}