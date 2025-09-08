//
// Created by hjl on 2021/9/1.
//

#include <ufomap_manager/frontier_manager.h>
// #include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/msg/ufo_map_stamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ufo/map/code.h>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("read_ufomap_node");
    rclcpp::Node nh_private("~");

    ufo::map::OccupancyMap map_(0.3);

    // std::string ns = ros::this_node::getName();
    std::string ns = nh->get_name();
    std::string txt_name = "ufomap.txt";
    // if (!ros::param::get(ns + "/txt_name", txt_name)) {
    if (!nh->get_parameter(ns + "/txt_name", txt_name)) {
        RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), 
                "No txt_name specified. Looking for %s. Default is 'ufomap.txt'.",
                (ns + "/txt_name").c_str());
    }

    if(map_.read(txt_name)){

        RCLCPP_INFO(rclcpp::get_logger("UfomapManager"), "resolution is %f", map_.getResolution());
        RCLCPP_INFO(rclcpp::get_logger("UfomapManager"), "size is % f", map_.getNodeSize(0));
        RCLCPP_INFO(rclcpp::get_logger("UfomapManager"), "leaf node num is %zu", map_.getNumLeafNodes());

        // auto pub = nh_private.advertise<ufomap_msgs::UFOMapStamped>("ufomap", 1);
        auto pub = nh_private.create_publisher<ufomap_msgs::msg::UFOMapStamped>("ufomap", 1);
        // auto cloud_pub_ = nh_private.advertise<sensor_msgs::msg::PointCloud2>("ufomap_cloud", 1);
        auto cloud_pub_ = nh_private.create_publisher<sensor_msgs::msg::PointCloud2>("ufomap_cloud", 1);

        while (rclcpp::ok()){
            std_msgs::msg::Header header;
            // header.stamp = rclcpp::Time::now();
            header.stamp = nh->get_clock()->now();
            header.frame_id = "world";

            // ufomap_msgs::UFOMapStamped msg;
            ufomap_msgs::msg::UFOMapStamped msg;
            ufomap_msgs::ufoToMsg(map_, msg.map, false);
            msg.header = header;
            // pub.publish(msg);//publish map
            pub->publish(msg);//publish map

            // if (0 < cloud_pub_.getNumSubscribers() || cloud_pub_.isLatched()) {
            if (0 < cloud_pub_->get_subscription_count()) {
                ufo::map::PointCloud cloud;
                for (auto it = map_.beginLeaves(true, false, false, false, 0),
                             it_end = map_.endLeaves();
                     it != it_end; ++it) {
                    cloud.push_back(it.getCenter());
                }
                RCLCPP_INFO(rclcpp::get_logger("UfomapManager"), "the cloud size is %zu", cloud.size());

                sensor_msgs::msg::PointCloud2 cloud_msg;
                ufomap_ros::ufoToRos(cloud, cloud_msg);
                cloud_msg.header = header;
                // cloud_pub_.publish(cloud_msg);
                cloud_pub_->publish(cloud_msg);
            }


            // rclcpp::spin_some(node);
            rclcpp::spin_some(nh);
            // rclcpp::Duration(1.0).sleep();
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }else{
        RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), "read ufomap failed..");
    }


    return 0;
}
