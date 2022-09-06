/*
 * @Author: Ke Zhang
 * @Date: 2022-09-06 19:39:14
 * @LastEditTime: 2022-09-06 19:56:55
 * @Description: CloudInfo消息发布类
 */
#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_data/cloud_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "localization/msg/cloud_info.hpp"

namespace localization
{
    class CloudInfoPublisher
    {
    public:
        CloudInfoPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id, int queue_size, int policy_history);
        void publishMsg(CloudData cloud, GnssData gnss);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        std::string frame_id_;
        rclcpp::Publisher<localization::msg::CloudInfo>::SharedPtr pub_cloud_info_;
    };
} // namespace localization
