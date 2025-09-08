//
// Created by hjl on 2022/1/13.
//

#include "rapid_cover_planner/rapid_cover_planner.h"

namespace rapid_cover_planner
{
    RapidCoverPlanner::RapidCoverPlanner(const std::shared_ptr<rclcpp::Node> nh, 
                                            const std::shared_ptr<rclcpp::Node> nh_private, 
                                            const Ufomap::Ptr &frontier_map, 
                                            const Map2DManager::Ptr &map_2d_manager, 
                                            const ViewpointManager::Ptr &viewpoint_manager, 
                                            const TopoGraph::Ptr &road_graph) :
        nh_(nh), 
        nh_private_(nh_private)
    {
        frontier_map_ = frontier_map;
        map_2d_manager_ = map_2d_manager;
        viewpoint_manager_ = viewpoint_manager;
        road_graph_ = road_graph;

        setParamsFromRos();
        // std::cout << "RapidCoverPlanner()" << std::endl;
    }

    void RapidCoverPlanner::setParamsFromRos()
    {
		nh_->declare_parameter("max_tour_point_num", 0);
        nh_->get_parameter("max_tour_point_num", max_tour_point_num_);

		nh_->declare_parameter("viewpoint_ignore_thre", 0.0);
        nh_->get_parameter("viewpoint_ignore_thre", viewpoint_ignore_thre_);

		nh_->declare_parameter("local_range", 0.0);
        nh_->get_parameter("local_range", local_range_);

		nh_->declare_parameter("frontier_gain", 0.0);
        nh_->get_parameter("frontier_gain", frontier_gain_);

		nh_->declare_parameter("tourpoint_ignore_thre", 0.0);
        nh_->get_parameter("tourpoint_ignore_thre", tourpoint_ignore_thre_);

		nh_->declare_parameter("tourpoint_ignore_distance", 0.0);
        nh_->get_parameter("tourpoint_ignore_distance", tourpoint_ignore_distance_);

		nh_->declare_parameter("is_directory", false);
        nh_->get_parameter("is_directory", is_directory_);

		nh_->declare_parameter("alpha", 0.0);
        nh_->get_parameter("alpha", alpha_);

        // std::cout << "============================== RapidCoverPlanner Parameters : START" << std::endl;
        // std::cout << "max_tour_point_num_ = " << max_tour_point_num_ << std::endl;
        // std::cout << "viewpoint_ignore_thre_ = " << viewpoint_ignore_thre_ << std::endl;
        // std::cout << "local_range_ = " << local_range_ << std::endl;
        // std::cout << "frontier_gain_ = " << frontier_gain_ << std::endl;
        // std::cout << "tourpoint_ignore_thre_ = " << tourpoint_ignore_thre_ << std::endl;
        // std::cout << "tourpoint_ignore_distance_ = " << tourpoint_ignore_distance_ << std::endl;
        // std::cout << "is_directory_ = " << is_directory_ << std::endl;
        // std::cout << "alpha_ = " << alpha_ << std::endl;
        // std::cout << "============================== RapidCoverPlanner Parameters : END" << std::endl;
        
        is_local_planning_ = true;
        std::string pkg_path = ament_index_cpp::get_package_share_directory("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";

        each_tourpoints_initilize_txt_name_ = txt_path + "each_tourpoints_initilize_time.txt";
        each_solving_txt_name_            = txt_path + "each_solving_time.txt";
        
        std::ofstream fout;

        sum_initilize_time_ = 0;
        initilize_num_ = 0;
        fout.open(each_tourpoints_initilize_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "each detection elisped time \n"
             << "start time \t" << "end time \t" << "elisped time \t"
             << "detection_num \t" << "average time \t"
             << std::endl;
        fout.close();

        sum_solving_time_ = 0;
        solving_num_ = 0;
        fout.open(each_solving_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "each iteration elisped time \n"
             << "iteration start time \t" << " iteration end time \t" << " iteration elisped time \t"
             << "iteration_num \t" << "average time \t"
             << std::endl;
        fout.close();

        sum_two_opt_time = 0.0;
        two_opt_time_name_ = txt_path + "two_opt_time.txt";
        fout.open(two_opt_time_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "solve_number\t" << "\t" << "solve_time(s)\t" << "mean_time(s)\t" << "points_num\t" << std::endl;
        fout.close();
    }

    void RapidCoverPlanner::planGraphConstruct(const graph::PlanGraph &old_graph, graph::PlanGraph &new_graph)
    {
        new_graph.clearGraph();
        new_graph = old_graph;
    }

    void RapidCoverPlanner::Initialize(const Point3D &current_position)
    {
        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "start initializing ....");
        auto start_time = nh_->get_clock()->now();
        initilize_num_++;

        getSuitableTourPoints(current_position);

        //calculate frontier gain
        tour_points_gains_.clear();
        max_gain_ = 0.0;
        viewpointsFrontierGain(tour_points_, tour_points_gains_, max_gain_);

        // if (tour_points_.size() > 3) {
        //     utils::Point3DSet tour_points;
        //     std::map<double, Point3DQueue> distances_viewpoints;

        //     for (const auto &item: tour_points_) {
        //         auto distance = item.distanceXY(current_position);
        //         distances_viewpoints[distance].push_back(item);
        //     }

        //     for (const auto &item: distances_viewpoints) {
        //         if (item.first < tourpoint_ignore_distance_) {
        //             for (const auto &point:item.second) {
        //                 if (viewpoint_manager_->viewpoints_attached_frontiers_[point].size() *
        //                     frontier_map_->map_.getResolution() * frontier_map_->map_.getResolution()
        //                     > tourpoint_ignore_thre_) {
        //                     tour_points.insert(point);
        //                 }
        //             }
        //         } else {
        //             for (const auto &point:item.second) {
        //                 tour_points.insert(point);
        //             }
        //         }
        //     }
        //     tour_points_ = tour_points;
        // }
        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "plan graph update...");
        planGraphConstruct(road_graph_->graph_, plan_graph_);
        // RCLCPP_INFO(rclcpp::get_logger("Planner"), " the plan graph update finish . vertex num is %zu, edge num is %zu ", plan_graph_.getAllVertices().size(), plan_graph_.getAllEdges().size());

        //add the viewpoints to the plan graph
        plan_graph_.updateKdtree();
        
        for(auto &viewpoint: tour_points_)
            if(!plan_graph_.isPoint3DExisted(viewpoint))
            {
                int a_id = plan_graph_.addVertex(viewpoint);
                int b_id = plan_graph_.getNearestVertexId(plan_graph_.kd_tree_, viewpoint);
                
                if(b_id != -1)
                {
                    plan_graph_.addTwoWayEdge(a_id, b_id);
                    // RCLCPP_INFO(rclcpp::get_logger("Planner"), "added a tour point and its edges in plan graph.");
                }
            }

        auto end_time = nh_->get_clock()->now();
        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "fast ray casting gain computed finish,spent %f s", (end_time - start_time).seconds());
        sum_initilize_time_ += (end_time - start_time).seconds();

        std::ofstream fout;
        fout.open(each_tourpoints_initilize_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << start_time.seconds() << "\t" << end_time.seconds() << "\t" << (end_time - start_time).seconds()
             << "\t" << initilize_num_ << "\t" << sum_initilize_time_ / initilize_num_ << "s \t"
             << std::endl;
        fout.close();
    }

    void RapidCoverPlanner::getSuitableTourPoints(const Point3D &current_position)
    {
        //tour points detection
        tour_points_.clear();

        utils::Point3DSet local_viewpoints;
        utils::Point3DSet global_viewpoints;

        if (!viewpoint_manager_) {
            std::cout << "[RapidCoverPlanner] ERROR: viewpoint_manager_ is null!" << std::endl;
            return;
        }

        // Print container size first
        std::cout << "[RapidCoverPlanner] DEBUG: viewpoint_manager_->viewpoints_attached_frontiers_.size() = "
                << viewpoint_manager_->viewpoints_attached_frontiers_.size() << std::endl;

        // Print current position and viewpoint_manager current_position_
        std::cout << "[RapidCoverPlanner] DEBUG: current_position = (" << current_position.x() << ", "
                << current_position.y() << ", " << current_position.z() << ")" << std::endl;
        std::cout << "[RapidCoverPlanner] DEBUG: viewpoint_manager_->current_position_ = ("
                << viewpoint_manager_->current_position_.x() << ", "
                << viewpoint_manager_->current_position_.y() << ", "
                << viewpoint_manager_->current_position_.z() << ")" << std::endl;
            
        // Print map lengths used for local/global split
        double gm_len_x = 0.0, gm_len_y = 0.0;
        if (map_2d_manager_) {
            gm_len_x = map_2d_manager_->fgmg_gm.getLength().x();
            gm_len_y = map_2d_manager_->fgmg_gm.getLength().y();
        }
        std::cout << "[RapidCoverPlanner] DEBUG: gm_len_x=" << gm_len_x << ", gm_len_y=" << gm_len_y << std::endl;

        // If container empty, early return (but we printed its size)
        if (viewpoint_manager_->viewpoints_attached_frontiers_.empty()) {
            std::cout << "[RapidCoverPlanner] WARNING: No viewpoints attached yet." << std::endl;
            return;
        }



        for(const auto &item: viewpoint_manager_->viewpoints_attached_frontiers_)
            if(item.first.distanceXY(current_position) > 1.0) 
            //     && item.second.size() * frontier_map_->map_.getResolution() * frontier_map_->map_.getResolution() > viewpoint_ignore_thre_)
                // if(item.first.distanceXY(viewpoint_manager_->current_position_) < local_range_ / 2)
                {
                    double gm_len_x = map_2d_manager_->fgmg_gm.getLength().x();
                    double gm_len_y = map_2d_manager_->fgmg_gm.getLength().y();

                if((viewpoint_manager_->current_position_.x() - (gm_len_x / 2)) <= item.first.x() 
                    && (viewpoint_manager_->current_position_.x() + (gm_len_x / 2)) >= item.first.x()
                    && (viewpoint_manager_->current_position_.y() - (gm_len_y / 2)) <= item.first.y()  
                    && (viewpoint_manager_->current_position_.y() + (gm_len_y / 2)) >= item.first.y())
                    local_viewpoints.insert(item.first);
                else
                    global_viewpoints.insert(item.first);
                }

        // //
        for (const auto &item : viewpoint_manager_->viewpoints_attached_frontiers_)
        {
            const Point3D &vp = item.first;
            const auto &frontiers = item.second;

            double dist = vp.distanceXY(current_position);
            std::cout << "Viewpoint (" << vp.x() << "," << vp.y() << ") distance=" << dist 
                    << " frontiers=" << frontiers.size() << std::endl;

            if (dist <= 1.0)
            {
                std::cout << "  Skipped: too close to robot" << std::endl;
                continue;
            }
        }

        // //
        std::cout << "#ViewPoints: Local = " << local_viewpoints.size() << ", Global = " << global_viewpoints.size() << std::endl;
            
        if(local_viewpoints.size() > max_tour_point_num_)
        {
            std::map<double, Point3DQueue> distances_viewpoints;
            std::vector<double> distances;

            for(const auto &item: local_viewpoints)
            {
                auto distance = item.distanceXY(current_position);
                
                if(distance > 1.0)
                {
                    distances.push_back(distance);
                    distances_viewpoints[distance].push_back(item);
                }
            }

            std::sort(distances.begin(), distances.end());
            int i = 0;

            for(const auto &item: distances_viewpoints)
            {
                for(const auto &viewpoint: item.second)
                {
                    tour_points_.insert(viewpoint);
                    ++i;

                    if(i >= max_tour_point_num_)
                        break;
                }

                if(i >= max_tour_point_num_)
                    break;
            }
        }
        else 
        {
            if(!local_viewpoints.empty())
            {
                is_local_planning_ = true;
                tour_points_ = local_viewpoints;
            }
            else
            {
                is_local_planning_ = false;
                auto graph = plan_graph_;
                int current_point_id = addCurrentPositionToGraph(current_position, graph);
                // int max_num = max_tour_point_num_;

                // if(global_viewpoints.size() > max_num) 
                if(global_viewpoints.size() > max_tour_point_num_)
                {
                    std::map<double, Point3DQueue> distances_viewpoints;
                    std::vector<double> distances;

                    for(const auto &item: global_viewpoints)
                    {
                        int item_point_id;

                        if(graph.getPointId(item, item_point_id))
                        {
                            auto path = getPathInGraph(current_point_id, item_point_id, graph);
                            double path_length = 0.0;

                            if(!path.empty())
                            {
                                for(int i = 0; i < path.size() - 1; ++i)
                                    path_length += path[i].distance(path[i + 1]);
                                
                                if(path_length > 1.0)
                                {
                                    distances.push_back(path_length);
                                    distances_viewpoints[path_length].push_back(item);
                                }
                            }
                            else
                            {
                                auto distance = item.distanceXY(current_position);

                                if(distance > 1.0)
                                {
                                    distances.push_back(distance);
                                    distances_viewpoints[distance].push_back(item);
                                }
                            }
                        }
                        else
                        {
                            auto distance = item.distanceXY(current_position);

                            if(distance > 1.0)
                            {
                                distances.push_back(distance);
                                distances_viewpoints[distance].push_back(item);
                            }
                        }
                    }

                    std::sort(distances.begin(), distances.end());
                    int i = 0;
                    
                    for(const auto &item: distances_viewpoints) 
                    {
                        for(const auto &viewpoint: item.second) 
                        {
                            tour_points_.insert(viewpoint);
                            ++i;

                            // if (i >= max_num)
                            if (i >= max_tour_point_num_)
                                break;
                        }
                        
                        // if (i >= max_num)
                        if (i >= max_tour_point_num_)
                            break;
                    }
                } 
                else
                    tour_points_ = global_viewpoints;
            }
        }
        
        // std::cout << "#TourPoints = " << tour_points_.size() << std::endl;
    }

    void RapidCoverPlanner::viewpointsFrontierGain(utils::Point3DSet &viewpoints, 
                                                   utils::Point3DMap<double> &tour_points_gains, 
                                                   double &max_gain)
    {
        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "start tour points gain computing...");
        // uint vp_id = 0;

        for(const auto &viewpoint: viewpoints)
        {
            Point3D sensor_point(viewpoint.x(), viewpoint.y(), viewpoint.z());
            // FrontierQueue viewpoint_visual_frontiers;
            std::cout << "Viewpoint " << sensor_point.x() << "," << sensor_point.y() 
                  << " frontiers count = " 
                  << viewpoint_manager_->viewpoints_attached_frontiers_[sensor_point].size()
                  << std::endl;
            // for(const auto &frontier: frontier_map_->getFrontiers())
            //     if((frontier.distanceXY(sensor_point) < frontier_map_->max_range_)) 
                    // && fabs(frontier.z() - sensor_point.z()) / frontier.distanceXY(sensor_point) < tan(M_PI * 15 / 180) 
                    // && frontier_map_->map_.isCollisionFree(ufo::map::Point3(sensor_point.x(), sensor_point.y(), sensor_point.z()), ufo::map::Point3(frontier.x(), frontier.y(), frontier.z())))
                    // && frontier_map_->isInCollision(ufo::map::Point3(sensor_point.x(), sensor_point.y(), sensor_point.z()), ufo::map::Point3(frontier.x(), frontier.y(), frontier.z())))     // High Computation
                    // viewpoint_visual_frontiers.push_back(frontier);

            // tour_points_gains[viewpoint] = static_cast<double>(viewpoint_visual_frontiers.size()) * frontier_gain_;
            tour_points_gains[viewpoint] = viewpoint_manager_->viewpoints_attached_frontiers_[sensor_point].size() * frontier_gain_;

            // std::cout << "VP_" << vp_id++ << " (" << viewpoint.x() << ", " << viewpoint.y() << ", " << viewpoint.z() << ") Gain = " << tour_points_gains[viewpoint] << std::endl;
            

            if(max_gain < tour_points_gains[viewpoint])
                max_gain = tour_points_gains[viewpoint];
        }

        // std::cout << "Max Gain = " << max_gain << std::endl;
    }

    void RapidCoverPlanner::planning(const geometry_msgs::msg::Pose &current_pose, const utils::Point3D &current_directory, bool &is_successed) {
        planner_mutex_.lock();
        solving_num_++;

        auto start_time = std::chrono::high_resolution_clock::now();
        double start_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count()) / 1000000;

        if(two_opt_solve_planning(current_pose, current_directory))
        {
            if(viewpoint_manager_->viewpoints_attached_frontiers_.count(goal_point_) != 0)
                goal_point_frontiers_ = viewpoint_manager_->viewpoints_attached_frontiers_.at(goal_point_);
            else
                goal_point_frontiers_.clear();
            
            std::cout << "Succeeded" << std::endl;
            is_successed = true;
        }
        else
        {
            goal_point_frontiers_.clear();
            std::cout << "Failed!" << std::endl;
            is_successed = false;
        }

        auto finish_time = std::chrono::high_resolution_clock::now();
        double finish_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(finish_time.time_since_epoch()).count()) / 1000000;
        double iteration_time = finish_time_second - start_time_second;
        sum_solving_time_ += iteration_time;

        std::ofstream time_fout;
        time_fout.precision(16);
        time_fout.open(each_solving_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        time_fout << start_time_second << "\t" << finish_time_second << "\t";
        time_fout.close();

        std::ofstream count_fout;
        count_fout.precision(6);
        count_fout.open(each_solving_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        count_fout << iteration_time << "\t" << solving_num_ << "\t" << sum_solving_time_ / solving_num_ << "s \t" << tour_points_.size() << std::endl;
        count_fout.close();

        planner_mutex_.unlock();
    }

    bool RapidCoverPlanner::two_opt_solve_planning(const geometry_msgs::msg::Pose &current_pose, const utils::Point3D &current_directory)
    {
        Point3D current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);

        path_to_go_.clear();
        tsp_path_.clear();
        path_segments_.clear();

        if(tour_points_.empty())
        {
            // if (plan_graph_.getAllVertices().size() < 5) {
            //     RCLCPP_INFO(rclcpp::get_logger("Planner"), "plan_map is not construct or vertex num is too few, please wait a minute...");
            // } else
            //     RCLCPP_WARN(rclcpp::get_logger("Planner"), " the candidate points is empty, exploration finish. ");

            std::cout << "Exploration Completed :)" << std::endl;
            rclcpp::shutdown();

            return false;
        }
        else
        {
            std::vector<Point3D> need_to_erase;
            
            for(auto &item1: pre_paths_)
            {
                if(viewpoint_manager_->viewpoints_attached_frontiers_.count(item1.first) == 0)
                    need_to_erase.push_back(item1.first);
                else
                {
                    std::vector<Point3D> to_erase;

                    for(const auto &item2: item1.second)
                        if((viewpoint_manager_->viewpoints_attached_frontiers_.count(item2.first) == 0) || item2.second.empty())
                            to_erase.push_back(item2.first);
                    
                    for(const auto &point:to_erase)
                        item1.second.erase(point);
                }
            }

            for(const auto &point: need_to_erase)
                pre_paths_.erase(point);
            // ====================================================================================================
    
            int current_point_id = addCurrentPositionToGraph(current_position, plan_graph_);
            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "current position is x = %f, y= %f, z = %f, id is %d", current_position.x(), current_position.y(), current_position.z(), current_point_id);

            Point3DQueue tour_points_term;
            tour_points_term.push_back(current_position);

            for(const auto &point: tour_points_)
                tour_points_term.push_back(point);
            
            int tour_points_num = tour_points_term.size();
            // ====================================================================================================

            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "start get path maxtirx of tour points");

            Path path_matrix[tour_points_num][tour_points_num];
            Path empty_path;

            for(int i = 0; i < tour_points_num; ++i)
                for(int j = 0; j < tour_points_num; ++j)
                    path_matrix[i][j] = empty_path;
            
            for(int i = 1; i < tour_points_num; i++)
            {
                // RCLCPP_INFO(rclcpp::get_logger("Planner"), "local point get path in terrain map..");
                // std::cout << "A* (R -> TP_" << i << ") in GM" << std::endl;
                // Path path = getPathInGridMap2D(current_position, tour_points_term.at(i));
                // if (path.empty()) {
                //     int end_point_id;
                //     if (plan_graph_.getPointId(tour_points_term.at(i), end_point_id)) {
                //         // RCLCPP_INFO(rclcpp::get_logger("Planner"), "local point get path in road map..");
                //         std::cout << "A* (R -> TP_" << i << ") in RM" << std::endl;
                //         path = getPathInGraph(current_point_id, end_point_id, plan_graph_);
                //     }
                // }

                int end_point_id;
                Path path;
                Point3D tp = tour_points_term.at(i);
                std::cout << "A*: R(" << current_position.x() << "," << current_position.y() << "," << current_position.z() << ") -> TP_" << i << "(" << tp.x() << "," << tp.y() << "," << tp.z() << ")" << std::endl;

                if(plan_graph_.getPointId(tp, end_point_id))
                {
                    std::cout << "RoadMap...";
                    path = getPathInGraph(current_point_id, end_point_id, plan_graph_);
                }
                
                if(path.empty())
                {
                    std::cout << "GridMap...";
                    path = getPathInGridMap2D(current_position, tp);                    
                }

                // RCLCPP_INFO(rclcpp::get_logger("Planner"), "path into matrix..");
                path_matrix[0][i] = path;
                std::reverse(path.begin(), path.end());
                path_matrix[i][0] = path;
            }

            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "start get other tour points paths..");
            for(int i = 1; i < tour_points_num; ++i)
                for(int j = 1; j < tour_points_num; ++j)
                    // if (i == j) {
                    //     path_matrix[i][j] = empty_path;
                    // } else if (path_matrix[i][j].empty()) {
                        // Path path = getPathInGridMap2D(tour_points_term.at(i), tour_points_term.at(j),
                        //                                map_2d_manager_->inflate_map_);
                        // Path path = getPathInGridMap2D(tour_points_term.at(i), tour_points_term.at(j));
                        // if (path.size() >= 2) {
                        //     pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)] = path;
                        //     auto path_reverse = path;
                        //     std::reverse(path_reverse.begin(), path_reverse.end());
                        //     pre_paths_[tour_points_term.at(j)][tour_points_term.at(i)] = path_reverse;
                        // }
                        // if (path.empty()) {
                        //     if (pre_paths_.count(tour_points_term.at(i)) != 0 &&
                        //         pre_paths_[tour_points_term.at(i)].count(tour_points_term.at(j)) != 0) {
                        //         path = pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)];
                        //     } else {
                        //         int start_point_id;
                        //         int end_point_id;
                        //         if (plan_graph_.getPointId(tour_points_term.at(i), start_point_id) &&
                        //             plan_graph_.getPointId(tour_points_term.at(j), end_point_id)) {
                        //             path = getPathInGraph(start_point_id, end_point_id, plan_graph_);
                        //             if (path.size() >= 2) {
                        //                 pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)] = path;
                        //                 auto path_reverse = path;
                        //                 std::reverse(path_reverse.begin(), path_reverse.end());
                        //                 pre_paths_[tour_points_term.at(j)][tour_points_term.at(i)] = path_reverse;
                        //             }
                        //         }
                        //     }
                        // }
                    if((i != j) && path_matrix[i][j].empty())
                    {
                        Point3D tp_i = tour_points_term.at(i);
                        Point3D tp_j = tour_points_term.at(j);
                        Path path;

                        if(pre_paths_.count(tp_i) != 0 
                            && pre_paths_[tp_i].count(tp_j) != 0)
                                path = pre_paths_[tp_i][tp_j];
                        else
                        {
                            int start_point_id;
                            int end_point_id;

                            if(plan_graph_.getPointId(tp_i, start_point_id) 
                                && plan_graph_.getPointId(tp_j, end_point_id))
                            {
                                path = getPathInGraph(start_point_id, end_point_id, plan_graph_);

                                if(path.size() >= 2)
                                {
                                    pre_paths_[tp_i][tp_j] = path;
                                    auto path_reverse = path;
                                    std::reverse(path_reverse.begin(), path_reverse.end());
                                    pre_paths_[tp_j][tp_i] = path_reverse;
                                }
                            }

                            if(path.empty())
                            {
                                path = getPathInGridMap2D(tp_i, tp_j);

                                if(path.size() >= 2)
                                {
                                    pre_paths_[tp_i][tp_j] = path;
                                    auto path_reverse = path;
                                    std::reverse(path_reverse.begin(), path_reverse.end());
                                    pre_paths_[tp_j][tp_i] = path_reverse;
                                }
                            }
                        }
                        
                        path_matrix[i][j] = path;
                        std::reverse(path.begin(), path.end());
                        path_matrix[j][i] = path;
                    }
            // ====================================================================================================
            
            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "path matrix got, start get cost matrix...");

            std::vector<std::vector<double>> cost_matrix = std::vector<std::vector<double>>(tour_points_num, std::vector<double>(tour_points_num, 10000000.0));

            for(int x = 1; x < tour_points_num; ++x)
                for(int y = 1; y < tour_points_num; ++y)
                    if(x == y)
                        cost_matrix[x][y] = 0.0;
                    else if(path_matrix[x][y].size() < 2)
                        cost_matrix[x][y] = 10000000;
                    else
                    {
                        double path_length = 0.0;

                        for(int k = 0; k < path_matrix[x][y].size() - 1; k++)
                            path_length += path_matrix[x][y][k].distance(path_matrix[x][y][k + 1]);
                        
                        cost_matrix[x][y] = path_length;
                    }
            
            for (int y = 1; y < tour_points_num; y++) {
                if (path_matrix[0][y].size() < 2) {
                    cost_matrix[0][y] = 100000000;
                } else {
                    double path_length = 0.0;
                    for (int k = 0; k < path_matrix[0][y].size() - 1; k++) {
                        path_length = path_length + path_matrix[0][y][k].distance(path_matrix[0][y][k + 1]);
                    }
                    cost_matrix[0][y] = path_length;

                    if(is_directory_){
                        if(tour_points_term[y].distance(current_position)<frontier_map_->max_range_){
                            Point3D diff_vector(tour_points_term[y].x() - current_position.x(),
                                                tour_points_term[y].y() - current_position.y(), 0);
                            double theta = current_directory.angleTo(diff_vector);
                            cost_matrix[0][y] = cost_matrix[0][y] * ((1-alpha_) * (log(theta / M_PI+1)/log(2))+alpha_);
                        }
                    }
                }
            }
            for (int x = 0; x < tour_points_num; ++x) {
                cost_matrix[x][0] = 0;
            }

            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "cost matrix got, start planning..");
            std::vector<std::vector<int>> nodes_indexs;
            std::vector<int> ids;

            for(int i = 0; i < tour_points_num; ++i)
                ids.push_back(i);
            
            // ====================================================================================================
            //two-opt solver ..
            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "start 2-opt solver..");
            // auto start_time = ros::WallTime::now();
            auto start_time = nh_->get_clock()->now();

            std::vector<double> gains;
            gains.push_back(0.0);

            for(int i = 1; i < tour_points_num; ++i)
                gains.push_back(tour_points_gains_[tour_points_term[i]]);
         
            double lambda = 3.0;
            std::vector<int> init_route = ids;

            Two_Opt two_opt_solve(init_route, gains, cost_matrix, lambda);
            two_opt_solve.solve();

            auto max_unity_way = two_opt_solve.best_route_;

            auto finish_time = nh_->get_clock()->now();
            double iteration_time = (finish_time - start_time).seconds();
            sum_two_opt_time += iteration_time;

            std::ofstream fout;
            fout.open(two_opt_time_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
            fout << solving_num_ << "\t" << iteration_time << "\t"
                 << sum_two_opt_time / solving_num_ << "s \t" << max_unity_way.size() - 1 << "\t"
                 << std::endl;
            fout.close();

            // RCLCPP_INFO(rclcpp::get_logger("Planner"), 
            //         "2-opt solver plan is finished, spent %.10f s, finish path node num is %zu, total searched ways size is %d",
            //         iteration_time, max_unity_way.size(), two_opt_solve.searched_route_num_);
            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "the best route is:");
            
            for(auto &i:max_unity_way)
                std::cout << i << " ";
            
            std::cout << std::endl;
            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "the best unity is %.10f", two_opt_solve.best_unity_);

            if(max_unity_way.size() > 1)
            {
                Path tsp_path;
                
                for(int i = 0; i < max_unity_way.size() - 1; i++)
                {
                    tsp_path.insert(tsp_path.end(), path_matrix[max_unity_way[i]][max_unity_way[i + 1]].begin(), path_matrix[max_unity_way[i]][max_unity_way[i + 1]].end());
                    path_segments_.push_back(path_matrix[max_unity_way[i]][max_unity_way[i + 1]]);
                }

                tsp_path_ = tsp_path;
                path_to_go_ = path_matrix[0][max_unity_way[1]];
                goal_point_ = tour_points_term.at(max_unity_way[1]);
                // RCLCPP_INFO(rclcpp::get_logger("Planner"), "the goal point is x= %f, y=%f, z=%f", goal_point_.x(), goal_point_.y(), goal_point_.z());

                return true;
            }
            else
                return false;
        }
    }

    int RapidCoverPlanner::addCurrentPositionToGraph(const Point3D &current_position, graph::PlanGraph &graph)
    {
        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "add current position to the plan map");
        int current_point_id = graph.addVertex(current_position);

        if(current_point_id != 0)
        {
            double local_range = 4.0;       // 2.0 OK
            Point3DQueue neighbor_vertexs;
            std::vector<int> neighbor_vertex_ids = graph.getNeighborVertexsIDs(graph.kd_tree_, current_position, local_range, neighbor_vertexs);

            for(const auto &item: neighbor_vertexs)
            {
                // if (map_2d_manager_->inflate_map_.isCollisionFreeStraight(
                //         Point2D(current_position.x(), current_position.y()),
                //         Point2D(item.x(), item.y())))
                if(map_2d_manager_->isLineTraversable(Point2D(current_position.x(), current_position.y()), Point2D(item.x(), item.y())))
                {
                    int item_id;

                    if(plan_graph_.getPointId(item, item_id) && current_point_id != item_id)
                    {
                        graph.addTwoWayEdge(current_point_id, item_id);
                        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "added a edge of the current point, end point coord z is %f ", item.z());
                    }
                }
            }
        }

        return current_point_id;
    }

    Path RapidCoverPlanner::getPathInGraph(const int &start_point_id, 
                                            const int &end_point_id, 
                                            const graph::PlanGraph &graph)
    {
        Path path;
        bool is_get_empty = false;
        std::vector<int> waypoints_ids;
        graph.getShortestPath(start_point_id, end_point_id, waypoints_ids, path);

        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "the path size is %zu", path.size());
        if(path.size() < 2)
            is_get_empty = true;

        return path;
    }

    Path RapidCoverPlanner::getPathInGridMap2D(const Point3D &start_point, const Point3D &end_point)
    {
        Point2D start_2d(start_point.x(), start_point.y());
        Point2D end_2d(end_point.x(), end_point.y());
        Path path_3d;
        bool is_get_empty = false;
        // if (grid_map_2d.isInMapRange2D(start_2d) && grid_map_2d.getStatusInMap2D(start_2d) == Status2D::Free &&
        //     grid_map_2d.isInMapRange2D(end_2d) && grid_map_2d.getStatusInMap2D(end_2d) == Status2D::Free)
        if(map_2d_manager_->isPointTraversable(start_2d) && map_2d_manager_->isPointTraversable(end_2d))
        {
            // std::vector<Point2D> path_2d = grid_map_2d.getShortestPath(end_2d, start_2d);
            std::vector<Point2D> path_2d = map_2d_manager_->a_star(end_2d, start_2d);

            if(path_2d.size() < 2)
                is_get_empty = true;
            
            // RCLCPP_INFO(rclcpp::get_logger("Planner"), "start optimal to straight..");
            // auto optimal_path_2d = grid_map_2d.optimalToStraight(path_2d);
            // auto optimal_path_2d = map_2d_manager_->smoothen_path(path_2d);

            // for (const auto &point: optimal_path_2d) {
            for(const auto &point: path_2d)
                // path_3d.emplace_back(point.x(), point.y(), map_2d_manager_->current_pose_.position.z);
                path_3d.emplace_back(point.x(), point.y(), map_2d_manager_->getPointElevation(point) + frontier_map_->sensor_height_);
            
        }
        else
        {
            std::cout << "NT" << std::endl;
            path_3d.clear();
        }

        return path_3d;
    }
}