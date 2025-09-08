//
// Created by hjl on 2021/11/26.
//

#ifndef TOPO_PLANNER_WS_EXPLORER_H
#define TOPO_PLANNER_WS_EXPLORER_H

// #include <control_planner_interface/PlannerMsgs.h>
#include <control_planner_interface/msg/planner_msgs.hpp>
#include <control_planner_interface/control_planner_interface.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <intf_pkg/srv/explore.hpp>
#include <intf_pkg/srv/set_map_bounds.hpp>


#include "autonovus_msgs/srv/start_exploration.hpp"
#include "autonovus_msgs/srv/stop_exploration.hpp"
#include "autonovus_msgs/action/start_exploration.hpp"
#include "autonovus_msgs/msg/bounding_box3_d.h"



using namespace std::chrono_literals;

namespace explorer {

    class Explorer {
    public:
        enum struct RunModeType {
            kSim = 0,  // Run in simulation.
            kReal= 1     // Run with real robot.
        };

        enum struct ExecutionPathType {
            kLocalPath = 0,
            kHomingPath = 1,
            kGlobalPath = 2,
            kNarrowEnvPath = 3, // Narrow env.
            kManualPath = 4     // Manually set path.
        };

        // rclcpp::Node nh_;
        std::shared_ptr<rclcpp::Node> nh_;
        // rclcpp::Node nh_private_;
        std::shared_ptr<rclcpp::Node> nh_private_;

        using StartExploration = autonovus_msgs::action::StartExploration;
        using GoalHandleStartExploration = rclcpp_action::ServerGoalHandle<StartExploration>;

        // ros::Subscriber planner_msgs_sub_;
        rclcpp::Subscription<control_planner_interface::msg::PlannerMsgs>::SharedPtr planner_msgs_sub_;

        // ros::Publisher init_finish_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr init_finish_pub_;
        // ros::Publisher finish_explore_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr finish_explore_pub_;

        // ros::Subscriber explore_finish_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_finish_sub_;

        // std::shared_ptr<interface::ControlPlannerInterface> interface_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

        rclcpp::Service<autonovus_msgs::srv::StartExploration>::SharedPtr start_exploration_srv_;
        rclcpp::Service<autonovus_msgs::srv::StopExploration>::SharedPtr stop_exploration_srv_;
        rclcpp_action::Server<StartExploration>::SharedPtr start_exploration_action_;
        

        rclcpp::TimerBase::SharedPtr exploration_timer_;

        void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
        bool is_odom_ready;

        RunModeType run_mode_;
        std::string frame_id_;
        bool init_motion_enable_;
        double init_x_;
        double init_y_;
        double init_z_;
        double init_need_time_;

        int iteration_num_;

        bool iteration_goal_is_scaned_;
        bool exploration_finished_;
        bool exploration_active_;
        bool need_to_next_iteration_;
        bool waiting_for_planner_response_;
        
       

        // ros::WallTime follow_start_time_;
        rclcpp::Time follow_start_time_;
        rclcpp::Time exploration_start_time_;
        double wait_time_;
        geometry_msgs::msg::Pose rob_pose, goal_pose;
        rclcpp::Client<intf_pkg::srv::Explore>::SharedPtr explore_sc;       // SC = Service Client
        rclcpp::Client<intf_pkg::srv::SetMapBounds>::SharedPtr set_map_bounds_sc_; 
        nav_msgs::msg::Path path;
        bool is_goal_given;
        bool is_goal_reached;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr home_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub;
        void pub_home();
        void pub_goal();
        double goal_tolerance_;
        std::string odometry_topic_name_;
        std::shared_ptr<GoalHandleStartExploration> current_goal_handle_;
        autonovus_msgs::msg::BoundingBox3D bbox_;


        // Explorer(rclcpp::Node &nh, rclcpp::Node &nh_private, std::shared_ptr<interface::ControlPlannerInterface> &interface);
        // Explorer(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private, const std::shared_ptr<interface::ControlPlannerInterface> interface);
        Explorer(const std::shared_ptr<rclcpp::Node> nh);

        bool loadParams();

        bool init();

        bool initMotion();

        // bool callForPlanner(const int &iteration_num, std::vector<control_planner_interface::Path> &path_segments);
        // bool callForPlanner(const int &iteration_num, std::vector<control_planner_interface::msg::Path> &path_segments);
        bool callForPlanner(int iteration_num);
    
        // void followThePath( const std::vector<control_planner_interface::Path> &path_segments);
        // void followThePath( const std::vector<control_planner_interface::msg::Path> &path_segments);
        void followThePath();

        bool isFollowFinish();

        void stopMove();

        bool isWaitTooLong();

        // void PlannerMsgsCallback(const control_planner_interface::PlannerMsgsConstPtr &msg);
        void PlannerMsgsCallback(const control_planner_interface::msg::PlannerMsgs::ConstSharedPtr &msg);

        void explorationFinishCallback(const std_msgs::msg::Bool::ConstSharedPtr &finish);

        void startExplorationCallback(
            const std::shared_ptr<autonovus_msgs::srv::StartExploration::Request> request,
            const std::shared_ptr<autonovus_msgs::srv::StartExploration::Response> response);

        void stopExplorationCallback(
            const std::shared_ptr<autonovus_msgs::srv::StopExploration::Request> request,
            const std::shared_ptr<autonovus_msgs::srv::StopExploration::Response> response);

        void explorationTick();

        

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const StartExploration::Goal> goal);
        
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleStartExploration> goal_handle);

        void handle_accepted(
            const std::shared_ptr<GoalHandleStartExploration> goal_handle);
        
        void execute(const std::shared_ptr<GoalHandleStartExploration> goal_handle);
    
        // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_pub_;
        // void boundary_pub();

    };
}


#endif //TOPO_PLANNER_WS_EXPLORER_H
