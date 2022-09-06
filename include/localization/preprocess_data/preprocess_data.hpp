/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:55:26
 * @LastEditTime: 2022-09-06 17:24:21
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

enum lidarType
{
    Pandar

};

namespace localization
{
    class PreprocessData
    {
    public:
        using PointXYZIRT = PandarPointXYZIRT;
        PreprocessData();

        bool preprocessCloud(std::deque<ImuData> &imu_data,
                             std::deque<GnssData> &gnss_data,
                             sensor_msgs::msg::PointCloud2 raw_cloud,
                             CloudData &deskewedCloud);

        Eigen::Affine3d imu2lidar_;
        Eigen::Affine3d gnss2lidar_;

    private:
        void readYaml(YAML::Node &config);
        void resetParameter();
        void initLidar();
        void initExtrinsicParameter();

        bool currentCloudInfo(sensor_msgs::msg::PointCloud2 currentCloudMsg);
        bool imuDeskewInfo(std::deque<ImuData> &imu_data);
        bool gnssDeskewInfo(std::deque<GnssData> &gnss_data);
        bool deskewPoint(CloudData::pointType &point, double point_time);
        bool deskewCloud(CloudData &deskewed_cloud);
        double pointDistance(CloudData::pointType point);

        pcl::PointCloud<PointXYZIRT>::Ptr laser_cloud_in_;
        double lidar_min_range_;
        double lidar_max_range_;
        int downsample_rate_;
        int n_scan_;
        int horizon_scan_;
        lidarType lidar_;
        int number_of_cores_;

        bool deskew_flag_;
        double time_scan_cur_;
        double time_scan_end_;

        std::vector<double> imu_time_;
        std::vector<Eigen::Quaterniond> imu_rot_;
        std::vector<Eigen::Affine3d> gnss_pose_;
        std::vector<double> gnss_time_;

        Eigen::Affine3f trans_tart_inverse;
        Eigen::Quaterniond imu_start_rot_inv_;
        Eigen::Vector3d imu_start_position_;
        Eigen::Quaterniond gnss_start_rot_;
        Eigen::Vector3d gnss_start_position_;
        bool frist_point_flag_;
        int imu_num_;
        int gnss_num_;
    };
} // namespace localization