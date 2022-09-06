/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:55:26
 * @LastEditTime: 2022-09-06 13:25:04
 * @Description:预处理核心算法文件
 */
#pragma once
#include <omp.h>
#include <yaml-cpp/yaml.h>

#include <deque>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "glog/logging.h"

#include "sensor_data/imu_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "sensor_data/cloud_data.hpp"
#include "cloud_type/cloud_type.hpp"
#include "global_path.h"

namespace localization
{
    class PreprocessDataNode : public rclcpp::Node
    {
    public:
        PreprocessDataNode(const rclcpp::NodeOptions &options) : Node("PreprocessData", options)
        {
        }

    private:
    };
} // namespace localization