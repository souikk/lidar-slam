/*
 * @Author: Ke Zhang
 * @Date: 2022-09-07 14:30:11
 * @LastEditTime: 2022-09-07 17:48:59
 * @Description: 订阅点云及其它传感器的同步信息类
 */
#pragma once

#include <memory>
#include <deque>

#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "localization/msg/cloud_info.hpp"

namespace localization
{
    class CloudInfoSubscriber
    {
    public:
        CloudInfoSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, size_t queue_size, int policy_history);
        void accessData(std::deque<localization::msg::CloudInfo> &cloud_info_buff);
        void accessData(std::deque<CloudData> &cloud_buff, std::deque<GnssData> &gnss_buff);

    private:
        void cloudInfoCallback(localization::msg::CloudInfo::SharedPtr msg);

        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<localization::msg::CloudInfo>::SharedPtr sub_cloud_info_;
        std::deque<localization::msg::CloudInfo> cloud_info_data_buf_;
        std::mutex lock_key;
    };
} // namespace localization
