/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:58:44
 * @LastEditTime: 2022-09-06 20:21:49
 * @Description:数据预处理工作流程类的头文件
 */
#pragma once
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"

#include "preprocess_data/preprocess_data_node.hpp"
#include "preprocess_data/preprocess_data.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"

#include "publisher/cloud_publisher.hpp"
#include "publisher/gnss_publisher.hpp"
#include "publisher/cloud_info_publisher.hpp"
#include "sensor_data/imu_data.hpp"
#include "localization/msg/cloud_info.hpp"
#include "global_path.h"

namespace localization
{
    class PreprocessDataFlow
    {
    public:
        PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr);

        void run();

    private:
        void initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        void initPublisher(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        void resetParameters();
        bool read();
        bool vaildData();
        bool syncData();
        bool undistortedPointCloud();
        void publishData();

        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<ImuSubscriber> subImu_;
        std::shared_ptr<CloudSubscriber> subCloud_;
        std::shared_ptr<GnssSubscriber> subGnss_;

        std::shared_ptr<CloudPublisher> cloud_publisher_;
        std::shared_ptr<GnssPublisher> gnss_publisher_;
        std::shared_ptr<CloudInfoPublisher> cloud_info_publisher_;

        std::shared_ptr<PreprocessData> preprocess_data_;

        std::deque<GnssData> unsync_gnss_buff_;
        std::deque<ImuData> unsync_imu_buff_;
        std::deque<sensor_msgs::msg::PointCloud2> unsync_cloud_buff_;
        sensor_msgs::msg::PointCloud2 current_cloud_;
        GnssData sync_gnss_;
        ImuData sync_imu_;
        CloudData sync_cloud_;
        localization::msg::CloudInfo cloud_info_;
        double current_cloud_time_;

        int n_scan_;
        int horizon_scan_;
    };
} // namespace localization
