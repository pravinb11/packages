//
// Created by hjl on 2021/11/23.
//

#ifndef TOPO_PLANNER_WS_CONTROL_PLANNER_INTERFACE_H
#define TOPO_PLANNER_WS_CONTROL_PLANNER_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
// #include <tf/tf.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/empty.hpp>
#include <chrono>
#include <thread>

#include "control_planner_interface/pci_manager.h"

// #include <actionlib/client/simple_action_client.h>
// #include "control_planner_interface/ExplorerPlannerAction.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_planner_interface/action/explorer_planner.hpp"

using ExplPlanner = control_planner_interface::action::ExplorerPlanner;
using ExplPlannerCGH = rclcpp_action::ClientGoalHandle<ExplPlanner>;
        

namespace interface {

    class ControlPlannerInterface {

    public:

        enum struct RobotMotionState {
            waiting = 0, executing = 1, stoping = 2
        };

        // typedef actionlib::SimpleActionClient<control_planner_interface::ExplorerPlannerAction> ExplorationPlannerClient;

        // rclcpp::Node nh_;
        std::shared_ptr<rclcpp::Node> nh_;
        // rclcpp::Node nh_private_;
        std::shared_ptr<rclcpp::Node> nh_private_;

        std::shared_ptr<PCIManager> pci_manager_;

        // ros::Subscriber odometry_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

        // ExplorationPlannerClient planner_client_;

        // using ExplPlanner = control_planner_interface::action::ExplorerPlanner;
        // using ExplPlannerCGH = rclcpp_action::ClientGoalHandle<ExplPlanner>;
        rclcpp_action::Client<ExplPlanner>::SharedPtr planner_client_;

        std::string frame_id_;
        bool pose_is_ready_;

        double init_x_;
        double init_y_;
        double init_z_;

        geometry_msgs::msg::Pose current_pose_;

        // ControlPlannerInterface(rclcpp::Node &nh, rclcpp::Node &nh_private,
        ControlPlannerInterface(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private,
                                std::shared_ptr<PCIManager> &pci_manager);

        bool loadParams();

        bool init();

        void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);

        // bool callForPlanner(const int &iteration_id, std::vector<control_planner_interface::Path> &path_segments);
        bool callForPlanner(const int &iteration_id, std::vector<control_planner_interface::msg::Path> &path_segments);

        // void executePath(const std::vector<control_planner_interface::Path> &path_segments);
        void executePath(const std::vector<control_planner_interface::msg::Path> &path_segments);

        void goToWayPose(geometry_msgs::msg::Pose &pose);

        bool isGoalReached();

        void cancelCurrentGoal();

        void stopMove();
        void goal_response_cb(const ExplPlannerCGH::SharedPtr &goal_handle);
        void feedback_cb(ExplPlannerCGH::SharedPtr, const std::shared_ptr<const ExplPlanner::Feedback> feedback);
        void result_cb(const ExplPlannerCGH::WrappedResult &result);
    };
}


#endif //TOPO_PLANNER_WS_CONTROL_PLANNER_INTERFACE_H
