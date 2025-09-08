#include "explorer/explorer.h"

namespace explorer
{
    Explorer::Explorer(const std::shared_ptr<rclcpp::Node> nh): 
        nh_(nh), iteration_num_(0)
    {
        loadParams();
        
        explore_sc = nh_->create_client<intf_pkg::srv::Explore>("explore_srv");
        set_map_bounds_sc_ = nh_->create_client<intf_pkg::srv::SetMapBounds>("set_map_bounds");
        
        odom_sub = nh_->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 1, std::bind(&Explorer::odom_cb, this, std::placeholders::_1));
        planner_msgs_sub_ = nh_->create_subscription<control_planner_interface::msg::PlannerMsgs>("topo_planner_msgs", 1, std::bind(&Explorer::PlannerMsgsCallback, this, std::placeholders::_1));
        explore_finish_sub_ = nh_->create_subscription<std_msgs::msg::Bool>("exploration_data_finish", 1, std::bind(&Explorer::explorationFinishCallback, this, std::placeholders::_1));
        
        auto latched_qos = rclcpp::QoS(1).transient_local(); // size 1, latched
        init_finish_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("explorer_inited", latched_qos);
        // init_finish_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("explorer_inited", 1);
        finish_explore_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("explorer_finish", 1);
        home_pub = nh_->create_publisher<visualization_msgs::msg::Marker>("home", 1);
        goal_pub = nh_->create_publisher<visualization_msgs::msg::Marker>("goal", 1);
        path_pub = nh_->create_publisher<nav_msgs::msg::Path>("path", 1);
        // boundary_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("boundary", 1);

        start_exploration_srv_ = nh_->create_service<autonovus_msgs::srv::StartExploration>(
            "exploration/start/service", 
            std::bind(&Explorer::startExplorationCallback, this, 
                std::placeholders::_1, std::placeholders::_2)); 
        
        stop_exploration_srv_ = nh_->create_service<autonovus_msgs::srv::StopExploration>(
            "exploration/stop",
            std::bind(&Explorer::stopExplorationCallback, this,
                 std::placeholders::_1, std::placeholders::_2)); 

        start_exploration_action_ = rclcpp_action::create_server<autonovus_msgs::action::StartExploration>(
            nh_->get_node_base_interface(),
            nh_->get_node_clock_interface(),
            nh_->get_node_logging_interface(),
            nh_->get_node_waitables_interface(),
            "start_exploration",
            std::bind(&Explorer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Explorer::handle_cancel, this, std::placeholders::_1),
            std::bind(&Explorer::handle_accepted, this, std::placeholders::_1));

        // action_server_pose_ = std::make_unique<ActionServerToPose>(
        //     shared_from_this(), "exploration/start", std::bind(&PlannerServer::startExplorationCallback, this),
        //     nullptr, 
        //     std::chrono::milliseconds(500), true);
        

        // Periodic exploration loop (disabled until service starts it)
        exploration_timer_ = nh_->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&Explorer::explorationTick, this));
        exploration_timer_->cancel(); // start only after service call

        is_odom_ready = is_goal_given = is_goal_reached = exploration_active_ = false;

        // while(!is_odom_ready)
        // {
        //     rclcpp::sleep_for(1s);
        //     rclcpp::spin_some(nh_);
        // }

        // if(init_motion_enable_)
        //     if(!initMotion())
        //     {
        //         RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"), "Initial motion failed!");
        //         rclcpp::shutdown();
        //     }

        // while(!is_goal_reached)
        // {
        //     rclcpp::sleep_for(1s);
        //     rclcpp::spin_some(nh_);
        // }

        // ====================================================================================================
        double start_explore_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000000;
        std_msgs::msg::Float64 is_init_finish;
        is_init_finish.data = start_explore_time;
        init_finish_pub_->publish(is_init_finish);
        // ====================================================================================================
        // exploration_finished_ = false;
        // iteration_goal_is_scaned_ = false;
        // need_to_next_iteration_ = true;

        // while(rclcpp::ok())
        // {
        //     if(exploration_finished_)
        //     {
        //         std::cout << "Exploration Completed :)" << std::endl;
        //         rclcpp::shutdown();
        //     }

        //     if(!is_goal_given)
        //     {
        //         iteration_num_++;
        //         std::cout << "Iteration " << iteration_num_ << ": " << std::flush;

        //         if(callForPlanner(iteration_num_))
        //         {
        //             std::cout << "Goal = (" << goal_pose.position.x << ", " << goal_pose.position.y << ", " << goal_pose.position.z << ")" << std::endl;
        //             pub_goal();

        //             followThePath();
        //             is_goal_reached = false;
        //             is_goal_given = true;
        //         }
        //     }
            

        //     rclcpp::sleep_for(1s);
        //     rclcpp::spin_some(nh_);
        // }
    }

    bool Explorer::loadParams()
    {
        std::string run_mode_val;
		nh_->declare_parameter("run_mode", "");
        nh_->get_parameter("run_mode", run_mode_val);

        if(!run_mode_val.compare("kReal"))
            run_mode_ = RunModeType::kReal;
        else if(!run_mode_val.compare("kSim"))
            run_mode_ = RunModeType::kSim;
        else
        {
            run_mode_ = RunModeType::kSim;
            RCLCPP_WARN(rclcpp::get_logger("ExplorationManager"), "Default: run_mode_ = kSim.");
        }

		nh_->declare_parameter("frame_id", "");
        nh_->get_parameter("frame_id", frame_id_);

		nh_->declare_parameter("init_motion_enable", false);
        nh_->get_parameter("init_motion_enable", init_motion_enable_);

		nh_->declare_parameter("init_x", 0.0);
        nh_->get_parameter("init_x", init_x_);

		nh_->declare_parameter("init_y", 0.0);
        nh_->get_parameter("init_y", init_y_);

		nh_->declare_parameter("init_z", 0.0);
        nh_->get_parameter("init_z", init_z_);

		nh_->declare_parameter("init_need_time", 0.0);
        nh_->get_parameter("init_need_time", init_need_time_);

		nh_->declare_parameter("goal_tolerance", 0.0);
        nh_->get_parameter("goal_tolerance", goal_tolerance_);

		nh_->declare_parameter("odometry_topic_name", "odometry");
        nh_->get_parameter("odometry_topic_name", odometry_topic_name_);

        // std::cout << "============================== Explorer::Parameters : START" << std::endl;
        // std::cout << "run_mode_ = " << run_mode_ << std::endl;
        // std::cout << "frame_id_ = " << frame_id_ << std::endl;
        // std::cout << "init_motion_enable_ = " << init_motion_enable_ << std::endl;
        // std::cout << "init_x_ = " << init_x_ << std::endl;
        // std::cout << "init_y_ = " << init_y_ << std::endl;
        // std::cout << "init_z_ = " << init_z_ << std::endl;
        // std::cout << "init_need_time_ = " << init_need_time_ << std::endl;
        // std::cout << "goal_tolerance_ = " << goal_tolerance_ << std::endl;
        // std::cout << "odometry_topic_name_ = " << odometry_topic_name_ << std::endl;
        // std::cout << "============================== Explorer::Parameters : END" << std::endl;

        return true;
    }

    bool Explorer::init()
    {
        if (init_motion_enable_) {
            if (!initMotion()) {
                RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"), "explorer init motion failed.");
                return false;
            }
        }
        return true;
    }

    bool Explorer::initMotion()
    {
        geometry_msgs::msg::PoseStamped home_msg;
        home_msg.pose = rob_pose;
        pub_home();

        geometry_msgs::msg::PoseStamped init_goal_msg;
        init_goal_msg.pose.position.x = rob_pose.position.x + init_x_;
        init_goal_msg.pose.position.y = rob_pose.position.y + init_y_;
        init_goal_msg.pose.position.z = rob_pose.position.z + init_z_;
        goal_pose = init_goal_msg.pose;
        pub_goal();

        std::cout << "initMotion: (" << rob_pose.position.x << ", " << rob_pose.position.y << ", " << rob_pose.position.z << ") -> (" << goal_pose.position.x << ", " << goal_pose.position.y << ", " << goal_pose.position.z << ")" << std::endl;

        nav_msgs::msg::Path init_path;
        init_path.header.stamp = nh_->get_clock()->now();
        init_path.header.frame_id = frame_id_;
        init_path.poses.push_back(home_msg);
        init_path.poses.push_back(init_goal_msg);
        path_pub->publish(init_path);

        is_goal_given = true;
        rclcpp::sleep_for(1s);
        
        return true;
    }

    bool Explorer::callForPlanner(int iteration_num)
    {
        std::cout << "In call for planner" <<std::endl;
        auto explore_srv_req = std::make_shared<intf_pkg::srv::Explore::Request>();
        explore_srv_req->iter_id = iteration_num_;
 

        
        std::cout << "[PlannerClient] Sending request: " << std::endl;
        
        while(!explore_sc->wait_for_service(1000ms)) {
            if(!rclcpp::ok()) {
                std::cout << "Explore SC Aborted!" << std::endl;
                return false;
            }
            std::cout << "Explore SS Unavailable!" << std::endl;
        }
        
        // auto future = explore_sc->async_send_request(
        //     explore_srv_req,
        //     [this](rclcpp::Client<intf_pkg::srv::Explore>::SharedFuture response) {
        //         std::cout << "Explorer has sent service request "<<std::endl;
        //         auto res = response.get();
        //         std::vector<geometry_msgs::msg::Pose> path_tmp = res->path;
        //         uint path_size = path_tmp.size();
        //         path.poses.clear();

        //         if(path_size > 0) {
        //             for(uint pose_id = 0; pose_id < path_size; ++pose_id) {
        //                 geometry_msgs::msg::PoseStamped ps_msg;
        //                 ps_msg.pose = path_tmp[pose_id];
        //                 path.poses.push_back(ps_msg);
        //             }
        //             goal_pose = path_tmp.back();
        //             std::cout << "Planner response received, goal set" << std::endl;
        //         } else {
        //             std::cout << "|path| = 0, Planner response empty" << std::endl;
        //         }

        //         waiting_for_planner_response_ = false; 
        //     }
        // );
        auto future = explore_sc->async_send_request(
        explore_srv_req,
        [this](rclcpp::Client<intf_pkg::srv::Explore>::SharedFuture response) {
            RCLCPP_DEBUG(nh_->get_logger(), "[Explorer] Service request sent to topo planner");

            try {
                auto res = response.get();
                RCLCPP_DEBUG(nh_->get_logger(), "[Explorer] Service response received");

                std::vector<geometry_msgs::msg::Pose> path_tmp = res->path;
                uint path_size = path_tmp.size();

                RCLCPP_DEBUG(nh_->get_logger(), "[Explorer] Received path size = %u", path_size);

                path.poses.clear();

                if (path_size > 0) {
                    for (uint pose_id = 0; pose_id < path_size; ++pose_id) {
                        geometry_msgs::msg::PoseStamped ps_msg;
                        ps_msg.pose = path_tmp[pose_id];
                        path.poses.push_back(ps_msg);

                        RCLCPP_DEBUG(nh_->get_logger(),
                            "[Explorer] Added pose[%u]: (%.2f, %.2f, %.2f)",
                            pose_id,
                            ps_msg.pose.position.x,
                            ps_msg.pose.position.y,
                            ps_msg.pose.position.z
                        );
                    }

                    goal_pose = path_tmp.back();
                    RCLCPP_INFO(nh_->get_logger(),
                        "[Explorer] Planner response received, goal set to (%.2f, %.2f, %.2f)",
                        goal_pose.position.x,
                        goal_pose.position.y,
                        goal_pose.position.z
                    );
                } else {
                    RCLCPP_WARN(nh_->get_logger(), "[Explorer] Planner response empty, path size = 0");
                }

                waiting_for_planner_response_ = false;
                RCLCPP_DEBUG(nh_->get_logger(), "[Explorer] waiting_for_planner_response_ set to false");

            } catch (const std::exception &e) {
                RCLCPP_ERROR(nh_->get_logger(), "[Explorer] Exception while handling service response: %s", e.what());
            }
        }
    );


        waiting_for_planner_response_ = true; 
        return true; 
    }

    void Explorer::followThePath()
    {
        path.header.stamp = follow_start_time_ = nh_->get_clock()->now();
        path.header.frame_id = frame_id_;
        path_pub->publish(path);
    }

    bool Explorer::isFollowFinish()
    {
        double d = std::hypot(goal_pose.position.x - rob_pose.position.x,
                          goal_pose.position.y - rob_pose.position.y);

        if(d <= 2)
            return true;
        else
            return false;
    }

    void Explorer::stopMove()
    {
        RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"), "..... the robot is going to stop .....");
    }

    bool Explorer::isWaitTooLong() {
        // return (ros::WallTime::now() - follow_start_time_).toSec() > wait_time_;
        return (nh_->get_clock()->now() - follow_start_time_).seconds() > wait_time_;
    }

    void Explorer::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
    {
        rob_pose = odom->pose.pose;
        // std::cout << "Explorer::odom_cb(): rob_pose = (" << rob_pose.position.x << "," << rob_pose.position.y << ","  << rob_pose.position.z << ")" << std::endl;
        is_odom_ready = true;

        if(is_goal_given)
        {
            double dist_x = std::abs(goal_pose.position.x - rob_pose.position.x);
            double dist_y = std::abs(goal_pose.position.y - rob_pose.position.y);
            double dist_z = std::abs(goal_pose.position.z - rob_pose.position.z);

            // std::cout << "dist_xyz = (" << dist_x << ", " << dist_y << ", " << dist_z << ")" << std::endl;

            // std::cout << "goal (" << goal_pose.position.x <<", "<< goal_pose.position.y << ", " << goal_pose.position.z <<")" << std::endl; 
            // std::cout << "robot pose (" << rob_pose.position.x <<", "<< rob_pose.position.y << ", " << rob_pose.position.z <<")" << std::endl;
            if((dist_x <= goal_tolerance_) && (dist_y <= goal_tolerance_) && (dist_z <= goal_tolerance_))
            {
                std::cout << "odom_cb: @ Goal" << std::endl;
                is_goal_reached = true;
                is_goal_given = false;
            }
            // double dist_3d = std::sqrt(
            //     std::pow(goal_pose.position.x - rob_pose.position.x, 2) +
            //     std::pow(goal_pose.position.y - rob_pose.position.y, 2) +
            //     std::pow(goal_pose.position.z - rob_pose.position.z, 2)
            // );
            // std::cout << "dist_3d = (" << dist_3d << std::endl;
            // if (dist_3d <= goal_tolerance_) {
            //     std::cout << "odom_cb: @ Goal" << std::endl;
            //     is_goal_reached = true;
            //     is_goal_given = false;
            // }
        }
    }

    void Explorer::PlannerMsgsCallback(const control_planner_interface::msg::PlannerMsgs::ConstSharedPtr &msg)
    {        
        exploration_finished_ = msg->is_exploration_finished;

        if(iteration_num_ == msg->current_goal_id)
            iteration_goal_is_scaned_ = msg->is_current_goal_scanned;
        else
            iteration_goal_is_scaned_ = false;
    }

    void Explorer::explorationFinishCallback(const std_msgs::msg::Bool::ConstSharedPtr &finish)
    {
        if(finish->data == true)
            exploration_finished_ = true;
        
        
    }

    void Explorer::startExplorationCallback(
        const std::shared_ptr<autonovus_msgs::srv::StartExploration::Request> request,
        const std::shared_ptr<autonovus_msgs::srv::StartExploration::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"),
                    "Received service goal request");

        if(!is_odom_ready){
            RCLCPP_WARN(rclcpp::get_logger("ExplorationManager"),"Odom not ready yet");
            response->success = false;
            response->message = "Odomety not ready";
            return;
        }

        const auto & bbox = request->exploration_area;
        RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"),
                    "Received timeout=%.2f",request->timeout);
         
        RCLCPP_INFO(nh_->get_logger(),
            "Received Bounding box: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f], frame: %s",
            bbox.min_x, bbox.max_x, bbox.min_y, bbox.max_y, bbox.min_z, bbox.max_z, bbox.frame_id.c_str());
        

        auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
            nh_,
            "topo_planner_node"
        );

        auto future = parameters_client->set_parameters({
            rclcpp::Parameter("min_x", request->exploration_area.min_x),
            rclcpp::Parameter("max_x", request->exploration_area.max_x),
            rclcpp::Parameter("min_y", request->exploration_area.min_y),
            rclcpp::Parameter("max_y", request->exploration_area.max_y),
            rclcpp::Parameter("min_z", request->exploration_area.min_z),
            rclcpp::Parameter("max_z", request->exploration_area.max_z)
        });

        exploration_finished_ = false;
        exploration_active_ = true;
        iteration_goal_is_scaned_ = false;
        need_to_next_iteration_ = true;
        is_goal_given = false;
        is_goal_reached = false;
        iteration_num_ = 0;
        waiting_for_planner_response_ = false;
        if(init_motion_enable_ && !initMotion())
        {
            RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"), "Initial motion failed!");
            response->success = false;
            response->message = "Initial motion failed";
            return;
        }
        
        exploration_start_time_ = nh_->get_clock()->now();
        exploration_timer_->reset();
        // exploration_timer_->cancel();
        // exploration_timer_->reset();
        exploration_timer_->execute_callback();
        // exploration_timer_->reset();

        response->success = true;
        response->message = "Exploration started";
    }                                                

    void Explorer::explorationTick(){

        if(exploration_finished_)
        {
            std::cout << "Exploration Completed :)" << std::endl;

            if(current_goal_handle_)
            {
                auto result = std::make_shared<autonovus_msgs::action::StartExploration::Result>();
                result->success = true;
                result->message = "Exploration finished";
                current_goal_handle_->succeed(result);
                current_goal_handle_.reset();
            }

            return;
        }

        if (waiting_for_planner_response_) {
            std::cout << "Waiting for planner response, skip this tick" << std::endl;
            return;
        }

        if(!is_goal_given)
        {
            iteration_num_++;
            std::cout << "Iteration " << iteration_num_ << ": " << std::flush;

            if(callForPlanner(iteration_num_))
            {
                std::cout << "Goal = (" << goal_pose.position.x << ", " << goal_pose.position.y << ", " << goal_pose.position.z << ")" << std::endl;
                pub_goal();

                followThePath();
                is_goal_reached = false;
                is_goal_given = true;
            }
            else
            {
                std::cout << "Planner call failed, will retry next tick." << std::endl;
            }
            return;
        }
            
    }

    rclcpp_action::GoalResponse Explorer::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const StartExploration::Goal> goal)
    {
        RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"), "Received action goal request");
        (void)uuid; //unused

        if(!is_odom_ready){
            RCLCPP_WARN(rclcpp::get_logger("ExplorationManager"),"Odom not ready yet");
            return rclcpp_action::GoalResponse::REJECT;
        }

        bbox_ = goal->exploration_area;
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        // boundary_pub();
        
    }

    rclcpp_action::CancelResponse Explorer::handle_cancel(
        const std::shared_ptr<GoalHandleStartExploration> goal_handle)
    {
        RCLCPP_INFO(nh_->get_logger(),"Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Explorer::handle_accepted(const std::shared_ptr<GoalHandleStartExploration> goal_handle)
    {
        current_goal_handle_ = goal_handle;
        std::thread{std::bind(&Explorer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void Explorer::execute(const std::shared_ptr<GoalHandleStartExploration> goal_handle)
    {
        
        
        
        
        RCLCPP_INFO(nh_->get_logger(),
            "Received Bounding box: x[%.2f, %.2f], x[%.2f, %.2f], x[%.2f, %.2f], frame: %s",
            bbox_.min_x, bbox_.max_x, bbox_.min_y, bbox_.max_y, bbox_.min_z, bbox_.max_z, bbox_.frame_id.c_str());
        

        auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
            nh_,
            "topo_planner_node"
        );


        auto request = std::make_shared<intf_pkg::srv::SetMapBounds::Request>();
        request->min_x = bbox_.min_x;
        request->max_x = bbox_.max_x;
        request->min_y = bbox_.min_y;
        request->max_y = bbox_.max_y;
        request->min_z = bbox_.min_z;
        request->max_z = bbox_.max_z;
        // set_map_bounds_sc_->async_send_request(request);
        // auto future = set_map_bounds_sc_->async_send_request(
        // request,
        // [this](rclcpp::Client<intf_pkg::srv::SetMapBounds>::SharedFuture response) {
        //     auto res = response.get();
        //     std::this_thread::sleep_for(std::chrono::seconds(2));
        //     if (res->success) {
        //         RCLCPP_INFO(nh_->get_logger(), "SetMapBounds service succeeded");
                
        //     } else {
        //         RCLCPP_WARN(nh_->get_logger(), "SetMapBounds service failed");
        //     }
        // });
        auto future = set_map_bounds_sc_->async_send_request(request);

        // Wait until the service call completes (with timeout)
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(nh_->get_logger(), "SetMapBounds service succeeded");
            } else {
                RCLCPP_WARN(nh_->get_logger(), "SetMapBounds service failed");
            }
        } else {
            RCLCPP_ERROR(nh_->get_logger(), "SetMapBounds service call timed out!");
        }
        
        RCLCPP_INFO(nh_->get_logger(),
            "Received Bounding box: x[%.2f, %.2f], x[%.2f, %.2f], x[%.2f, %.2f], frame: %s",
            bbox_.min_x, bbox_.max_x, bbox_.min_y, bbox_.max_y, bbox_.min_z, bbox_.max_z, bbox_.frame_id.c_str());

        RCLCPP_INFO(nh_->get_logger(), "Executing goal");

        exploration_finished_ = false;
        exploration_active_ = true;
        iteration_goal_is_scaned_ = false;
        need_to_next_iteration_ = true;
        is_goal_given = false;
        is_goal_reached = false;
        iteration_num_ = 0;
        waiting_for_planner_response_ = false;

         if(init_motion_enable_ && !initMotion())
        {
            RCLCPP_INFO(rclcpp::get_logger("ExplorationManager"), "Initial motion failed!");
            goal_handle->abort(std::make_shared<autonovus_msgs::action::StartExploration::Result>());
            return;
        }

        exploration_start_time_ = nh_->get_clock()->now();
        exploration_timer_->reset();
        exploration_timer_->execute_callback();        
    }

    
    void Explorer::stopExplorationCallback(
        const std::shared_ptr<autonovus_msgs::srv::StopExploration::Request> request,
        const std::shared_ptr<autonovus_msgs::srv::StopExploration::Response> response)
    {
        RCLCPP_INFO(nh_->get_logger(), "Stop Exploration requested");
        
        if (exploration_timer_) {
            exploration_timer_->cancel();
        } 

        // if(current_goal_handle_)
        // {
        //     auto result = std::make_shared<StartExploration::Result>();
        //     result->success = false;
        //     result->message = "Exploration canceled via service";
        //     current_goal_handle_->canceled(result);
        //     current_goal_handle_.reset();
        // }
        if (current_goal_handle_) {
            if (current_goal_handle_->is_active()) {
                auto result = std::make_shared<StartExploration::Result>();
                result->success = false;
                result->message = "Exploration canceled via service";

                try {
                    current_goal_handle_->canceled(result);
                    RCLCPP_INFO(nh_->get_logger(), "Exploration goal canceled");
                } catch (const rclcpp::exceptions::RCLError &e) {
                    RCLCPP_ERROR(nh_->get_logger(), "Failed to cancel goal: %s", e.what());
                }
            } else {
                RCLCPP_WARN(nh_->get_logger(), "Stop request received but goal handle was not active");
            }
            current_goal_handle_.reset();
        }

        response->success = true;
        if (exploration_active_)
        {
            response->was_active = true;
            RCLCPP_INFO(nh_->get_logger(), "Exploration stopped successfully");
            exploration_active_ = false;
        }
        else   
        {
            response->was_active = false;
            RCLCPP_INFO(nh_->get_logger(), "Exploration was not running");
        }

        exploration_finished_ = true;
        is_goal_given = false;
        is_goal_reached = true;
    }

    void Explorer::pub_home()
    {
        visualization_msgs::msg::Marker home_marker;
        home_marker.header.frame_id = frame_id_;
        home_marker.header.stamp = nh_->get_clock()->now();
        home_marker.ns = "home_ns";
        home_marker.id = 0;
        home_marker.type = visualization_msgs::msg::Marker::SPHERE;
        home_marker.action = visualization_msgs::msg::Marker::ADD;
        home_marker.pose.position.x = rob_pose.position.x;
        home_marker.pose.position.y = rob_pose.position.y;
        home_marker.pose.position.z = rob_pose.position.z;
        home_marker.pose.orientation.w = 1.0;
        home_marker.pose.orientation.x = 0.0;
        home_marker.pose.orientation.y = 0.0;
        home_marker.pose.orientation.z = 0.0;
        home_marker.scale.x = 0.5;
        home_marker.scale.y = 0.5;
        home_marker.scale.z = 0.5;
        home_marker.color.r = 0.0;
        home_marker.color.g = 1.0;
        home_marker.color.b = 0.0;
        home_marker.color.a = 1.0;
        home_pub->publish(home_marker);
    }

    void Explorer::pub_goal()
    {
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header.frame_id = frame_id_;
        goal_marker.header.stamp = nh_->get_clock()->now();
        goal_marker.ns = "goal_ns";
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose.position.x = goal_pose.position.x;
        goal_marker.pose.position.y = goal_pose.position.y;
        goal_marker.pose.position.z = goal_pose.position.z;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.pose.orientation.x = 0.0;
        goal_marker.pose.orientation.y = 0.0;
        goal_marker.pose.orientation.z = 0.0;
        goal_marker.scale.x = 0.5;
        goal_marker.scale.y = 0.5;
        goal_marker.scale.z = 0.5;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        goal_marker.color.a = 1.0;
        goal_pub->publish(goal_marker);
    }

    // void Explorer::boundary_pub()
    // {
    //     visualization_msgs::msg::Marker boundary_marker;
    //     boundary_marker.header.frame_id = frame_id_;
    //     boundary_marker.header.stamp = nh_->get_clock()->now();
    //     boundary_marker.ns = "boundary_ns";
    //     boundary_marker.id = 0;
    //     boundary_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    //     boundary_marker.action = visualization_msgs::msg::Marker::ADD;

    //     geometry_msgs::msg::Point boundary_point_1;
    //     boundary_point_1.x = bbox_.min_x;
    //     boundary_point_1.y = bbox_.min_y;
    //     boundary_point_1.z = current_sensor_pose_.z();
    //     boundary_marker.points.push_back(boundary_point_1);
        
    //     geometry_msgs::msg::Point boundary_point_2;
    //     boundary_point_2.x = bbox_.max_x;
    //     boundary_point_2.y = bbox_.min_y;
    //     boundary_point_2.z = current_sensor_pose_.z();
    //     boundary_marker.points.push_back(boundary_point_2);
    //     boundary_marker.points.push_back(boundary_point_2);
        
    //     geometry_msgs::msg::Point boundary_point_3;
    //     boundary_point_3.x = bbox_.max_x;
    //     boundary_point_3.y = bbox_.max_y;
    //     boundary_point_3.z = current_sensor_pose_.z();
    //     boundary_marker.points.push_back(boundary_point_3);
    //     boundary_marker.points.push_back(boundary_point_3);
        
    //     geometry_msgs::msg::Point boundary_point_4;
    //     boundary_point_4.x = bbox_.min_x;
    //     boundary_point_4.y = bbox_.max_y;
    //     boundary_point_4.z = current_sensor_pose_.z();
    //     boundary_marker.points.push_back(boundary_point_4);
    //     boundary_marker.points.push_back(boundary_point_4);
    //     boundary_marker.points.push_back(boundary_point_1);

    //     boundary_marker.scale.x = 0.125;
    //     boundary_marker.scale.y = 0.125;
    //     boundary_marker.scale.z = 0.125;
    //     boundary_marker.color.r = 1.0;
    //     boundary_marker.color.g = 0.0;
    //     boundary_marker.color.b = 0.0;
    //     boundary_marker.color.a = 1.0;
    //     boundary_pub_->publish(boundary_marker);
    // }
}