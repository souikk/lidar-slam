/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 16:53:01
 * @LastEditTime: 2022-09-06 19:50:57
 * @Description:
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sensor_data/gnss_data.hpp"
#include "preprocess_data/preprocess_data_node.hpp"

namespace localization
{
    class GnssPublisher
    {
    public:
        GnssPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id, int queue_size, int policy_history);

        void publishMsg(GnssData gnss_data);

    private:
        rclcpp::Node::SharedPtr node_ptr_;
        std::string frame_id_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gnss_;
    };
} // namespace localization
