//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_TOPO_PLANNER_H
#define ROBO_PLANNER_WS_TOPO_PLANNER_H

#include "rclcpp/rclcpp.hpp"
// #include <tf/tf.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <actionlib/server/simple_action_server.h>
// #include <control_planner_interface/ExplorerPlannerAction.h>
#include <control_planner_interface/action/explorer_planner.hpp>
// #include <control_planner_interface/PlannerMsgs.h>
#include <control_planner_interface/msg/planner_msgs.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>

#include "preprocess/preprocess.h"
#include "rapid_cover_planner/rapid_cover_planner.h"

#include "intf_pkg/srv/explore.hpp"
#include "intf_pkg/srv/set_map_bounds.hpp"

namespace topo_planner{

    class TopoPlanner{
    public:
        // typedef actionlib::SimpleActionServer<control_planner_interface::ExplorerPlannerAction> ExplorerPlannerServer;

        // rclcpp::Node nh_;
        std::shared_ptr<rclcpp::Node> nh_;
        // rclcpp::Node nh_private_;
        std::shared_ptr<rclcpp::Node> nh_private_;

        // ros::Subscriber explore_finish_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_finish_sub_;

        // ros::Publisher iteration_time_pub_;
        // rclcpp::Publisher<visualization_tools::msg::IterationTime>::SharedPtr iteration_time_pub_;

        // ExplorerPlannerServer planner_action_server_;
        int planner_goal_id_;

        // rclcpp::Timer preprocess_timer_;
        rclcpp::TimerBase::SharedPtr preprocess_timer_;
        // rclcpp::Timer msgs_timer_;
        rclcpp::TimerBase::SharedPtr msgs_timer_;
        bool preprocess_inited_;

        // ros::Publisher topo_planner_msgs_pub_;
        // rclcpp::Publisher<control_planner_interface::msg::PlannerMsgs>::SharedPtr topo_planner_msgs_pub_;

        preprocess::Preprocess::Ptr elements_;

        rapid_cover_planner::RapidCoverPlanner::Ptr planner_;
        rclcpp::Service<intf_pkg::srv::Explore>::SharedPtr explore_ss;      // SS = Service Server
        rclcpp::Service<intf_pkg::srv::SetMapBounds>::SharedPtr set_map_bounds_srv_; 
        void explore_cb(const std::shared_ptr<intf_pkg::srv::Explore::Request> request, std::shared_ptr<intf_pkg::srv::Explore::Response> response);

        // TopoPlanner(rclcpp::Node &nh, rclcpp::Node &nh_private);
        TopoPlanner(const std::shared_ptr<rclcpp::Node> &nh, const std::shared_ptr<rclcpp::Node> &nh_private);

        // void plannerCallback(const control_planner_interface::ExplorerPlannerGoalConstPtr &goal);

        std::vector<geometry_msgs::msg::Pose> wayPoseGeneration(rapid_cover_planner::Path &path);

        bool isExplorationFinish();

        bool isCurrentGoalScanned();

        void topo_planner_msgs_publish();

        void explorationFinishCallback(const std_msgs::msg::Bool::ConstSharedPtr &finish);

        void setMapBoundsCb(
            const std::shared_ptr<intf_pkg::srv::SetMapBounds::Request> request,
            std::shared_ptr<intf_pkg::srv::SetMapBounds::Response> response);

    };
}

#endif //ROBO_PLANNER_WS_TOPO_PLANNER_H
