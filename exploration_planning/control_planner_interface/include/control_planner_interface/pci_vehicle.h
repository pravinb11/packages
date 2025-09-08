//
// Created by hjl on 2021/11/30.
//

#ifndef TOPO_PLANNER_WS_PCI_VEHICLE_H
#define TOPO_PLANNER_WS_PCI_VEHICLE_H

#include "control_planner_interface/pci_manager.h"

// #include <actionlib/client/simple_action_client.h>
// #include <control_planner_interface/VehicleExecuteAction.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include <control_planner_interface/action/vehicle_execute.hpp>


using VehExec = control_planner_interface::action::VehicleExecute;
using VehExecCGH = rclcpp_action::ClientGoalHandle<VehExec>;


namespace interface {

    class PCIVehicle : public PCIManager {
    public:

        // using VehExec = control_planner_interface::action::VehicleExecute;
        // using VehExecCGH = rclcpp_action::ClientGoalHandle<VehExec>;

        // typedef actionlib::SimpleActionClient<control_planner_interface::VehicleExecuteAction> VehicleExecuteActionClient;

        // PCIVehicle(const rclcpp::Node &nh, const rclcpp::Node &nh_private);
        PCIVehicle(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private);

        bool loadParams(const std::string ns) override;

        bool initialize() override;

        void setCurrentPose(const geometry_msgs::msg::Pose &pose) override;

        void setFrameId(const std::string &frame_id) override;

        bool goToWaypoint(const geometry_msgs::msg::Pose &pose) override;

        // bool executePath(const std::vector<control_planner_interface::Path> &path_segments,
        bool executePath(const std::vector<control_planner_interface::msg::Path> &path_segments,
                         std::vector<geometry_msgs::msg::Pose> &modified_path) override;

        bool isGoalReached() override;

        void cancelCurrentGoal() override;

        void stopMove() override;

        VehExecCGH::SharedPtr cgh;
        bool is_goal_reached;

        void goal_response_cb(const VehExecCGH::SharedPtr &goal_handle);
        void feedback_cb(VehExecCGH::SharedPtr, const std::shared_ptr<const VehExec::Feedback> feedback);
        void result_cb(const VehExecCGH::WrappedResult &result);

        void goal_response_2_cb(const VehExecCGH::SharedPtr &goal_handle);
        void feedback_2_cb(VehExecCGH::SharedPtr, const std::shared_ptr<const VehExec::Feedback> feedback);
        void result_2_cb(const VehExecCGH::WrappedResult &result);

    private:

        // ros::Subscriber status_sub_;
        // rclcpp::Subscription<>::SharedPtr status_sub_;
        // ros::Publisher robot_status_pub_;
        // rclcpp::Publisher<>::SharedPtr robot_status_pub_;

        // VehicleExecuteActionClient vehicle_execute_client_;
        rclcpp_action::Client<VehExec>::SharedPtr vehicle_execute_client_;
        // VehicleExecuteActionClient vehicle_stop_client_;
        rclcpp_action::Client<VehExec>::SharedPtr vehicle_stop_client_;

        std::string frame_id_;

        int n_seq_;
        rclcpp::Node::SharedPtr nh_;
    };
}


#endif //TOPO_PLANNER_WS_PCI_VEHICLE_H
