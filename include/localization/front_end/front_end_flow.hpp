/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:17
 * @LastEditTime: 2022-09-07 20:28:36
 * @Description:
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include "front_end/front_end_node.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/cloud_info_subscriber.hpp"
#include "publisher/cloud_info_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
namespace localization
{
    class FrontEndFlow
    {
    public:
        FrontEndFlow(rclcpp::Node::SharedPtr node_ptr);
        void run();

    private:
        void initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        void initPublisher(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        void resetParameter();
        bool readData();
        void initGnss();
        bool updateOdometry();
        bool publishMsg();

        rclcpp::Node::SharedPtr node_ptr_;
        std::deque<CloudData> cloud_buff_;
        std::deque<GnssData> gnss_buff_;
        std::deque<localization::msg::CloudInfo> cloud_info_buff_;

        std::shared_ptr<CloudInfoSubscriber> cloud_info_subscriber_;

        std::shared_ptr<CloudInfoPublisher> cloud_info_publisher_;

        CloudData current_cloud_;
        GnssData current_gnss_;
        bool flag_init_gnss_;
        bool flag_frist_cloud_;
    };
}