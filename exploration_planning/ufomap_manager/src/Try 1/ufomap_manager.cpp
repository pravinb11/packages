//
// Created by hjl on 2020/6/15.
//
#include <ufomap_manager/ufomap_manager.h>
// #include <ufomap_msgs/UFOMap.h>
#include <ufomap_msgs/msg/ufo_map.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <future>

namespace ufomap_manager {

    UFOMapManager::UFOMapManager(std::shared_ptr<rclcpp::Node> nh):
        nh_(nh),
        tf_buffer_(nh_->get_clock()),
        tf_listener_(tf_buffer_),
        map_(0.1, 16, true){

        setParametersFromROS();
        // cloud_sub_ = nh.subscribe("point_cloud", 10, &UFOMapManager::insertCloudCallback, this);
        cloud_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS(), std::bind(&UFOMapManager::insertCloudCallback, this, std::placeholders::_1));

        if (0 < pub_rate_) {
            // map_pub_ = nh_private.advertise<ufomap_msgs::UFOMapStamped>(
                    // "map", 10, nh_private.param("map_latch", false));
            map_pub_ = nh_->create_publisher<ufomap_msgs::msg::UFOMapStamped>("map", 10);
            // cloud_pub_ = nh_private.advertise<sensor_msgs::msg::PointCloud2>(
            //         "map_cloud", 10, nh_private.param("map_cloud_latch", false));
            cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
            // pub_timer_ = nh_private.createTimer(rclcpp::Rate(pub_rate_), &UFOMapManager::timerCallback, this);
            pub_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UFOMapManager::timerCallback, this));
        }
    }


    void UFOMapManager::setParametersFromROS() {
        // std::string ns = ros::this_node::getName();
        std::string ns = nh_->get_name();
        frame_id_ = "world";
        // if (!ros::param::get(ns + "/frame_id", frame_id_)) {
        if (!nh_->get_parameter(ns + "/frame_id", frame_id_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        max_range_ = 12;
        // if (!ros::param::get(ns + "/max_range", max_range_)) {
        if (!nh_->get_parameter(ns + "/max_range", max_range_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No max_range specified. Looking for %s. Default is '12'.",
                     (ns + "/max_range").c_str());
        }

        insert_discrete_ = true;
        // if (!ros::param::get(ns + "/insert_discrete", insert_discrete_)) {
        if (!nh_->get_parameter(ns + "/insert_discrete", insert_discrete_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No insert_discrete specified. Looking for %s. Default is 'true'.",
                     (ns + "/insert_discrete").c_str());
        }

        insert_depth_ = 0;
        // if (!ros::param::get(ns + "/insert_depth", insert_depth_)) {
        if (!nh_->get_parameter(ns + "/insert_depth", insert_depth_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No insert_depth specified. Looking for %s. Default is '0'.",
                     (ns + "/insert_depth").c_str());
        }

        simple_ray_casting_ = false;
        // if (!ros::param::get(ns + "/simple_ray_casting", simple_ray_casting_)) {
        if (!nh_->get_parameter(ns + "/simple_ray_casting", simple_ray_casting_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No simple_ray_casting specified. Looking for %s. Default is 'false'.",
                     (ns + "/simple_ray_casting").c_str());
        }

        early_stopping_ = 0;
        // if (!ros::param::get(ns + "/early_stopping", early_stopping_)) {
        if (!nh_->get_parameter(ns + "/early_stopping", early_stopping_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No early_stopping specified. Looking for %s. Default is '0'.",
                     (ns + "/early_stopping").c_str());
        }

        clear_robot_ = false;
        // if (!ros::param::get(ns + "/clear_robot_enabled", clear_robot_)) {
        if (!nh_->get_parameter(ns + "/clear_robot_enabled", clear_robot_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No clear_robot_enabled_ specified. Looking for %s. Default is 'false'.",
                     (ns + "/clear_robot_enabled_").c_str());
        }

        robot_height_ = 0.2;
        // if (!ros::param::get(ns + "/robot_height", robot_height_)) {
        if (!nh_->get_parameter(ns + "/robot_height", robot_height_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No robot_height specified. Looking for %s. Default is '0.2m'.",
                     (ns + "/robot_height").c_str());
        }

        robot_radius_ = 0.5;
        // if (!ros::param::get(ns + "/robot_radius", robot_radius_)) {
        if (!nh_->get_parameter(ns + "/robot_radius", robot_radius_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No robot_radius specified. Looking for %s. Default is '0.5m'.",
                     (ns + "/sensor_height_").c_str());
        }

        pub_rate_ = 10;
        // if (!ros::param::get(ns + "/pub_rate", pub_rate_)) {
        if (!nh_->get_parameter(ns + "/pub_rate", pub_rate_)) {
            RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "No pub_rate specified. Looking for %s. Default is '10hz'.",
                     (ns + "/pub_rate").c_str());
        }

    }


    void UFOMapManager::insertCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        ufo::math::Pose6 transform;
        try {
            // transform = ufomap_ros::rosToUfo(
            //         tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp,
            //                                    rclcpp::Duration(1.0)).transform);
            // transform = ufomap_ros::rosToUfo(tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(1.0)).transform);
            transform = ufomap_ros::rosToUfo(tf_buffer_->lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(1.0)).transform);
        } catch (tf2::TransformException &ex) {
            // ROS_WARN_THROTTLE(1, "%s", ex.what());
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", ex.what());
            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *(nh_->get_clock()), 1000, "%s", ex.what());
            return;
        }

        auto start = std::chrono::steady_clock::now();
        // auto start_time = rclcpp::Time::now();
        // auto start_time = this->get_clock()->now();
        auto start_time = nh_->get_clock()->now();
        // Update map
        ufo::map::PointCloudColor cloud;
        ufomap_ros::rosToUfo(*msg, cloud);
        cloud.transform(transform, true);

        if (insert_discrete_) {
            map_.insertPointCloudDiscrete(
                    transform.translation(), cloud, max_range_, insert_depth_,
                    simple_ray_casting_, early_stopping_, false);
        } else {
            map_.insertPointCloud(
                    transform.translation(), cloud, max_range_, insert_depth_,
                    simple_ray_casting_, early_stopping_, false);
        }

        // double integration_time = (rclcpp::Time::now() - start_time).toSec();
        double integration_time = (nh_->now() - start_time).seconds();
        RCLCPP_INFO(rclcpp::get_logger("UfomapManager"), "insert cloud spent %f s", integration_time);

        if (0 == num_integrations_ || integration_time < min_integration_time_) {
            min_integration_time_ = integration_time;
        }
        if (integration_time > max_integration_time_) {
            max_integration_time_ = integration_time;
        }
        accumulated_integration_time_ += integration_time;
        ++num_integrations_;

        // Clear robot
        if (clear_robot_) {
            start = std::chrono::steady_clock::now();

            try {
                // transform = ufomap_ros::rosToUfo(
                //         tf_buffer_.lookupTransform(frame_id_, robot_frame_id_, msg->header.stamp,
                //                                    rclcpp::Duration(1.0)).transform);
                // transform = ufomap_ros::rosToUfo(tf_buffer_.lookupTransform(frame_id_, robot_frame_id_, msg->header.stamp, rclcpp::Duration::from_seconds(1.0)).transform);
                transform = ufomap_ros::rosToUfo(tf_buffer_->lookupTransform(frame_id_, robot_frame_id_, msg->header.stamp, rclcpp::Duration::from_seconds(1.0)).transform);
            } catch (tf2::TransformException &ex) {
                // ROS_WARN_THROTTLE(1, "%s", ex.what());
                // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", ex.what());
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *(nh_->get_clock()), 1000, "%s", ex.what());
                return;
            }

            ufo::map::Point3 r(robot_radius_, robot_radius_, robot_height_ / 2.0);
            ufo::geometry::AABB aabb(transform.translation() - r,
                                     transform.translation() + r);
            map_.setValueVolume(aabb, map_.getClampingThresMin(), clearing_depth_);

            double clear_time =
                    std::chrono::duration<float, std::chrono::seconds::period>(
                            std::chrono::steady_clock::now() - start)
                            .count();
            if (0 == num_clears_ || clear_time < min_clear_time_) {
                min_clear_time_ = clear_time;
            }
            if (clear_time > max_clear_time_) {
                max_clear_time_ = clear_time;
            }
            accumulated_clear_time_ += clear_time;
            ++num_clears_;
        }

    }

    // void UFOMapManager::timerCallback(const rclcpp::TimerEvent &event) {
    void UFOMapManager::timerCallback() {
        std_msgs::msg::Header header;
        // header.stamp = rclcpp::Time::now();
        // header.stamp = this->get_clock()->now();
        header.stamp = nh_->get_clock()->now();
        header.frame_id = frame_id_;

        // ufomap_msgs::UFOMapStamped msg;
        ufomap_msgs::msg::UFOMapStamped msg;
        ufomap_msgs::ufoToMsg(map_, msg.map, false);
        msg.header = header;
        // map_pub_.publish(msg);
        map_pub_->publish(msg);

        // if (0 < cloud_pub_.getNumSubscribers() || cloud_pub_.isLatched()) {
        if (0 < cloud_pub_->get_subscription_count()) {
            ufo::map::PointCloud cloud;
            for (auto it = map_.beginLeaves(true, false, false, false, 0),
                         it_end = map_.endLeaves();
                 it != it_end; ++it) {
                cloud.push_back(it.getCenter());
            }
            sensor_msgs::msg::PointCloud2 cloud_msg;
            ufomap_ros::ufoToRos(cloud, cloud_msg);
            cloud_msg.header = header;
            // cloud_pub_.publish(cloud_msg);
            cloud_pub_->publish(cloud_msg);
        }

    }
}