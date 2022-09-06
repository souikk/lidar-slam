/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 17:42:37
 * @LastEditTime: 2022-09-06 14:21:09
 * @Description: 订阅imu数据类
 */
#pragma once

#include <memory>
#include <deque>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

#include "global_path.h"

#include "sensor_data/imu_data.hpp"

#include "preprocess_data/preprocess_data_node.hpp"

namespace localization
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, size_t queue_size, int policy_history);

        void accessData(std::deque<ImuData> &imu_buff);

    private:
        void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
        rclcpp::Node::SharedPtr node_ptr_;
        std::deque<ImuData> imu_data_buf_;
        std::mutex lock_key;
    };
}
