//
// Created by hjl on 2022/1/11.
//

#include "topo_planner/topo_planner.h"

namespace topo_planner
{
    TopoPlanner::TopoPlanner(const std::shared_ptr<rclcpp::Node> &nh, 
                                const std::shared_ptr<rclcpp::Node> &nh_private):
        nh_(nh), 
        nh_private_(nh_private)//, 
        // preprocess_inited_(false)
    {
        elements_ = std::make_shared<preprocess::Preprocess>(nh_, nh_private_);
        planner_ = std::make_shared<rapid_cover_planner::RapidCoverPlanner>(nh_, 
                                                                            nh_private_,
                                                                            elements_->frontier_map_,
                                                                            elements_->map_2d_manager_,
                                                                            elements_->viewpoint_manager_,
                                                                            elements_->road_map_);
        
        // rclcpp::spin_some(nh_);
        
        explore_finish_sub_ = nh_->create_subscription<std_msgs::msg::Bool>("exploration_data_finish", 1, std::bind(&TopoPlanner::explorationFinishCallback, this, std::placeholders::_1));

        explore_ss = nh_->create_service<intf_pkg::srv::Explore>("explore_srv", std::bind(&TopoPlanner::explore_cb, this, std::placeholders::_1, std::placeholders::_2));

        // auto service = this->create_service<intf_pkg::srv::SetMapBounds>(
        //     "set_map_bounds",
        //     [this](const std::shared_ptr<intf_pkg::srv::SetMapBounds::Request> request,
        //         std::shared_ptr<intf_pkg::srv::SetMapBounds::Response> response) {
        //         elements_->frontier_map_->setBoundingBox(request->min_x, request->max_x,
        //                             request->min_y, request->max_y,
        //                             request->min_z, request->max_z);
        //         response->success = true;
        //     });
        set_map_bounds_srv_ = nh_->create_service<intf_pkg::srv::SetMapBounds>(
                    "set_map_bounds",
                    std::bind(&TopoPlanner::setMapBoundsCb, this,
                            std::placeholders::_1, std::placeholders::_2));
    }

    void TopoPlanner::explorationFinishCallback(const std_msgs::msg::Bool::ConstSharedPtr &finish)
    {
        if(finish->data == true)
            rclcpp::shutdown();
    }

    void TopoPlanner::setMapBoundsCb(
    const std::shared_ptr<intf_pkg::srv::SetMapBounds::Request> request,
    std::shared_ptr<intf_pkg::srv::SetMapBounds::Response> response)
    {
        elements_->frontier_map_->setBoundingBox(
            request->min_x, request->max_x,
            request->min_y, request->max_y,
            request->min_z, request->max_z);

        response->success = true;

        RCLCPP_INFO(nh_->get_logger(), "Set bounding box: "
            "x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
            request->min_x, request->max_x,
            request->min_y, request->max_y,
            request->min_z, request->max_z);
    }
    
    void TopoPlanner::explore_cb(const std::shared_ptr<intf_pkg::srv::Explore::Request> request, std::shared_ptr<intf_pkg::srv::Explore::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(),
            "[TopoPlanner] Received request: iter_id=%ld",
            request->iter_id);
            
        int retry_count = 0;
        const int max_retries = 5;   // set to -1 for infinite retries

        while (rclcpp::ok()) 
        {
        elements_->frontier_map_->map_mutex_.lock();
        elements_->map_2d_manager_->map_2d_update_mutex_.lock();
        elements_->elements_update_mutex_.lock();
        
        
        
        auto start_time = std::chrono::high_resolution_clock::now();
        double start_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count()) / 1000000;

        std::cout << "========== Iteration " << request->iter_id << " ==========" << std::endl;
        
        // elements_->frontier_map_->boundary_pub(request->min_x, request->max_x, request->min_y, request->max_y, request->min_z, request->max_z);

        elements_->frontier_map_->frontierUpdate();
        elements_->updateElements();

        if (elements_->viewpoint_manager_->viewpoints_attached_frontiers_.empty()) {
            std::cout << "[TopoPlanner] No viewpoints yet, retrying..." << std::endl;
            elements_->frontier_map_->map_mutex_.unlock();
            elements_->map_2d_manager_->map_2d_update_mutex_.unlock();
            elements_->elements_update_mutex_.unlock();
            response->path.clear();

            response->path.clear();

            retry_count++;
            if (max_retries > 0 && retry_count >= max_retries) {
                std::cout << "[TopoPlanner] No viewpoints after "
                          << retry_count << " retries. Aborting." << std::endl;
                return;
            }

            rclcpp::sleep_for(std::chrono::seconds(2));
            continue;  // try again

            
        }

        
        

        planner_->Initialize(elements_->current_position_);
        // preprocess_inited_ = true;

        // if (!preprocess_inited_) {
        //     RCLCPP_WARN(rclcpp::get_logger("Planner"), "preprocess not finish , waitting..");
        // } else 
        
        // if(planner_->tour_points_.empty())
        //     RCLCPP_WARN(rclcpp::get_logger("Planner"), "tourpoints is empty, planning finish..");
        // else
        // {
            bool is_successed = true;
            planner_->planning(elements_->current_pose_, elements_->forward_directory_, is_successed);

            if(is_successed)
                if(planner_->tsp_path_.empty())
                {
                    std::cout << "|tsp_path_| = 0" << std::endl;
                    response->path.clear();
                }
                else
                {
                    if(planner_->path_segments_.size())
                    {
                        std::vector<utils::Point3D> path_tmp = planner_->path_segments_[0];

                        for(uint pose_id = 0; pose_id < path_tmp.size(); ++pose_id)
                        {
                            geometry_msgs::msg::Pose pose_tmp;
                            pose_tmp.position.x = path_tmp[pose_id].x();
                            pose_tmp.position.y = path_tmp[pose_id].y();
                            pose_tmp.position.z = path_tmp[pose_id].z();
                            
                            if(path_tmp.size() != 1)
                            {
                                double angle;

                                if((pose_id + 1) < path_tmp.size())
                                    angle = std::atan2(path_tmp[pose_id + 1].y() - path_tmp[pose_id].y(), path_tmp[pose_id + 1].x() - path_tmp[pose_id].x());
                                else
                                    angle = std::atan2(path_tmp[pose_id].y() - path_tmp[pose_id - 1].y(), path_tmp[pose_id].x() - path_tmp[pose_id - 1].x()); 
                                
                                tf2::Quaternion quaternion;
                                quaternion.setRPY(0, 0, angle);
                                pose_tmp.orientation = tf2::toMsg(quaternion);
                            }

                            response->path.push_back(pose_tmp);
                        }
                    }
                }
            else
                std::cout << "Failed!" << std::endl;
        // }

        auto finish_time = std::chrono::high_resolution_clock::now();
        double finish_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(finish_time.time_since_epoch()).count()) / 1000000;
        double iteration_time = finish_time_second - start_time_second;

        // visualization_tools::IterationTime time;
        // time.iterationTime = iteration_time;
        // time.current_time = finish_time_second;
        // iteration_time_pub_.publish(time);
        // iteration_time_pub_->publish(time);

        elements_->viewpoint_manager_->pubMarkers();
        elements_->road_map_->pubGraphMarkers();

        elements_->frontier_map_->map_mutex_.unlock();
        elements_->map_2d_manager_->map_2d_update_mutex_.unlock();
        elements_->elements_update_mutex_.unlock();

            break;  
    }
        // if(request->iter_id == 1){std::cout << std::endl << " ---------- Aborting ---------- " << std::endl;rclcpp::shutdown();}
    }
    // void TopoPlanner::explore_cb(
    // const std::shared_ptr<intf_pkg::srv::Explore::Request> request,
    // std::shared_ptr<intf_pkg::srv::Explore::Response> response)
    // {
    //     elements_->frontier_map_->map_mutex_.lock();
    //     elements_->map_2d_manager_->map_2d_update_mutex_.lock();
    //     elements_->elements_update_mutex_.lock();

    //     auto start_time = std::chrono::high_resolution_clock::now();
    //     double start_time_second =
    //         static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
    //                                 start_time.time_since_epoch())
    //                                 .count()) /
    //         1000000.0;

    //     std::cout << "========== Iteration " << request->iter_id << " ==========" << std::endl;

    //     std::cout << "#representative_points: "
    //       << elements_->viewpoint_manager_->representative_points_.size()
    //       << ", #frontiers_viewpoints: "
    //       << elements_->viewpoint_manager_->frontiers_viewpoints_.size()
    //       << ", #attached: "
    //       << elements_->viewpoint_manager_->viewpoints_attached_frontiers_.size()
    //       << std::endl;

    //     // 1. Frontier update
    //     elements_->frontier_map_->frontierUpdate();
    //     std::cout << "#frontiers: " 
    //       << elements_->frontier_map_->getFrontiers().size() 
    //       << std::endl;

    //     // 2. Update elements (this usually generates viewpoints)
    //     elements_->updateElements();
    //     std::cout << "#viewpoints: " 
    //       << elements_->viewpoint_manager_->new_viewpoints_.size() 
    //       << std::endl;

    //     // 3. Check attached frontiers
    //     size_t attached_total = 0;
    //     for (const auto &vp : elements_->viewpoint_manager_->new_viewpoints_) {
    //         auto attached = elements_->viewpoint_manager_->viewpoints_attached_frontiers_[vp];
    //         attached_total += attached.size();
    //         std::cout << "#viewpoints: " 
    //             << elements_->viewpoint_manager_->new_viewpoints_.size() 
    //             << std::endl;
    //     }
    //     if (elements_->viewpoint_manager_->viewpoints_attached_frontiers_.empty()) {
    //         std::cout << "[TopoPlanner] No viewpoints yet, retrying..." << std::endl;
    //         elements_->frontier_map_->map_mutex_.unlock();
    //         elements_->map_2d_manager_->map_2d_update_mutex_.unlock();
    //         elements_->elements_update_mutex_.unlock();
    //         response->path.clear();
    //         return;  // retry later
    //     }
    //     if (attached_total == 0) {
    //         std::cout << "[TopoPlanner] Viewpoints exist but none attached to frontiers!" << std::endl;
    //     }

    //     // 4. Proceed with planner
    //     planner_->Initialize(elements_->current_position_);

    //     bool is_successed = true;
    //     planner_->planning(elements_->current_pose_,
    //                     elements_->forward_directory_,
    //                     is_successed);

    //     if (is_successed && !planner_->tsp_path_.empty()) {
    //         std::cout << "[TopoPlanner] Planner succeeded with path length = "
    //                 << planner_->tsp_path_.size() << std::endl;

    //         std::vector<utils::Point3D> path_tmp = planner_->path_segments_[0];
    //         for (uint pose_id = 0; pose_id < path_tmp.size(); ++pose_id) {
    //             geometry_msgs::msg::Pose pose_tmp;
    //             pose_tmp.position.x = path_tmp[pose_id].x();
    //             pose_tmp.position.y = path_tmp[pose_id].y();
    //             pose_tmp.position.z = path_tmp[pose_id].z();

    //             if (path_tmp.size() != 1) {
    //                 double angle;
    //                 if ((pose_id + 1) < path_tmp.size())
    //                     angle = std::atan2(path_tmp[pose_id + 1].y() - path_tmp[pose_id].y(),
    //                                     path_tmp[pose_id + 1].x() - path_tmp[pose_id].x());
    //                 else
    //                     angle = std::atan2(path_tmp[pose_id].y() - path_tmp[pose_id - 1].y(),
    //                                     path_tmp[pose_id].x() - path_tmp[pose_id - 1].x());

    //                 tf2::Quaternion quaternion;
    //                 quaternion.setRPY(0, 0, angle);
    //                 pose_tmp.orientation = tf2::toMsg(quaternion);
    //             }
    //             response->path.push_back(pose_tmp);
    //         }
    //     } else {
    //         std::cout << "[TopoPlanner] Planner failed or tsp_path_ empty!" << std::endl;
    //         response->path.clear();
    //     }

    //     auto finish_time = std::chrono::high_resolution_clock::now();
    //     double finish_time_second =
    //         static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
    //                                 finish_time.time_since_epoch())
    //                                 .count()) /
    //         1000000.0;
    //     double iteration_time = finish_time_second - start_time_second;
    //     std::cout << "[TopoPlanner] Iteration time = " << iteration_time << " seconds" << std::endl;

    //     elements_->viewpoint_manager_->pubMarkers();
    //     elements_->road_map_->pubGraphMarkers();

    //     elements_->frontier_map_->map_mutex_.unlock();
    //     elements_->map_2d_manager_->map_2d_update_mutex_.unlock();
    //     elements_->elements_update_mutex_.unlock();
    // }

}