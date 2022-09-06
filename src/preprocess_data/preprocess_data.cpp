/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:47
 * @LastEditTime: 2022-09-06 18:01:14
 * @Description:
 */
#include "preprocess_data/preprocess_data.hpp"

namespace localization
{
    PreprocessData::PreprocessData()
    {
        std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        readYaml(config);
        laser_cloud_in_.reset(new pcl::PointCloud<PointXYZIRT>());

        resetParameter();
        initLidar();
    }

    /**
     * @description: 读取yaml文件中的参数
     * @param {Node} &config
     * @return {*}
     */
    void PreprocessData::readYaml(YAML::Node &config)
    {
        lidar_ = lidarType(config["preprocess"]["lidar"].as<int>());
        downsample_rate_ = config["preprocess"]["downsampleRate"].as<int>();
        lidar_min_range_ = config["preprocess"]["lidarMinRange"].as<double>();
        lidar_max_range_ = config["preprocess"]["lidarMaxRange"].as<double>();
        n_scan_ = config["preprocess"]["N_SCAN"].as<int>();
        horizon_scan_ = config["preprocess"]["Horizon_SCAN"].as<int>();
        number_of_cores_ = config["preprocess"]["numberOfCores"].as<int>();
        deskew_flag_ = config["preprocess"]["dekewFlag"].as<bool>();
    }

    /**
     * @description: 重置参数
     * @return {*}
     */
    void PreprocessData::resetParameter()
    {
        laser_cloud_in_->clear();
        gnss_pose_.clear();
        frist_point_flag_ = true;
        imu_time_.clear();
        imu_rot_.clear();
        imu_num_ = 0;
        gnss_num_ = 0;
    }
    /**
     * @description: 初始化lidar型号
     * @return {*}
     */
    void PreprocessData::initLidar()
    {
        if (lidar_ == lidarType::Pandar)
        {
            using PointXYZIRT = PandarPointXYZIRT;
        }
        else
        {
            LOG(ERROR) << "unkown lidar type.";
            rclcpp::shutdown();
        }
    }

    /**
     * @description: 初始化外参
     * @return {*}
     */
    void PreprocessData::initExtrinsicParameter()
    {
        std::string config_file_path = WORKSPACE_PATH + "/config/extrinsic_parameter/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        std::vector<double> trans;
        std::vector<double> rot;
        trans = config["imu2lidar"]["extrinsicTrans"].as<std::vector<double>>();
        rot = config["imu2lidar"]["extrinsicRot"].as<std::vector<double>>();
        Eigen::Vector3d imu2lidar_translation = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(trans.data(), 3, 1);
        Eigen::Matrix3d imu2lidar_rot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(rot.data(), 3, 3);
        imu2lidar_ = Eigen::Affine3d(
            Eigen::Translation3d(imu2lidar_translation[0], imu2lidar_translation[1], imu2lidar_translation[2]) * imu2lidar_rot);

        trans.clear();
        rot.clear();

        trans = config["gnss2lidar"]["extrinsicTrans"].as<std::vector<double>>();
        rot = config["gnss2lidar"]["extrinsicRot"].as<std::vector<double>>();
        Eigen::Vector3d gnss2lidar_translation = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(trans.data(), 3, 1);
        Eigen::Matrix3d gnss2lidar_rot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(rot.data(), 3, 3);
        gnss2lidar_ = Eigen::Affine3d(
            Eigen::Translation3d(gnss2lidar_translation[0], gnss2lidar_translation[1], gnss2lidar_translation[2]) * gnss2lidar_rot);
    }

    /**
     * @description: 点云预处理
     * @return {bool}
     */
    bool PreprocessData::preprocessCloud(std::deque<ImuData> &imu_data,
                                         std::deque<GnssData> &gnss_data,
                                         sensor_msgs::msg::PointCloud2 current_cloud_msg,
                                         CloudData &deskewed_cloud)
    {
        if (!currentCloudInfo(current_cloud_msg))
        {
            return false;
        }
        if (!imuDeskewInfo(imu_data))
        {
            return false;
        }
        // if (!gnssDeskewInfo(gnss_data))
        // {
        //     return false;
        // }
        if (!deskewCloud(deskewed_cloud))
        {
            return false;
        }
        resetParameter();
        return true;
    }

    /**
     * @description: 获取当前帧的信息
     * @param {PointCloud2} current_cloud_msg
     * @return {bool}
     */
    bool PreprocessData::currentCloudInfo(sensor_msgs::msg::PointCloud2 current_cloud_msg)
    {

        pcl::moveFromROSMsg(current_cloud_msg, *laser_cloud_in_);
        time_scan_cur_ = rclcpp::Time(current_cloud_msg.header.stamp).seconds();
        time_scan_end_ = laser_cloud_in_->points.back().timestamp;
        if (laser_cloud_in_->is_dense == false)
        {
            LOG(ERROR) << "Point cloud is not in dense format, please remove NaN points first!";
            rclcpp::shutdown();
        }

        // check ring channel
        static int ring_flag = 0;
        if (ring_flag == 0)
        {
            ring_flag = -1;
            for (int i = 0; i < (int)current_cloud_msg.fields.size(); ++i)
            {
                if (current_cloud_msg.fields[i].name == "ring")
                {
                    ring_flag = 1;
                    break;
                }
            }
            if (ring_flag == -1)
            {
                LOG(ERROR) << "Point cloud ring channel not available, please configure your point cloud data!";
                rclcpp::shutdown();
            }
        }

        // check point time
        // if (deskew_flag_ == 0)
        // {
        //     deskew_flag_ = -1;
        //     for (auto &field : current_cloud_msg.fields)
        //     {
        //         if (field.name == "time" || field.name == "t" || field.name == "timestamp")
        //         {
        //             deskew_flag_ = 1;
        //             break;
        //         }
        //     }
        //     if (deskew_flag_ == -1)
        //         LOG(WARNING) << "Point cloud timestamp not available, deskew function disabled, system will drift significantly!";
        // }

        return true;
    }

    /**
     * @description: 提取imu信息用于去畸变
     * @param {deque<ImuData>} &imu_data
     * @return {bool}
     */
    bool PreprocessData::imuDeskewInfo(std::deque<ImuData> &imu_data)
    {
        //# 判断imu数据是否有效
        if (imu_data.size() < 2)
            return false;

        if (imu_data.front().time >= time_scan_cur_ || imu_data.back().time < time_scan_end_ + 0.01)
        {
            LOG(ERROR) << "imu data can not deskew";
            return false;
        }

        //# 缓存imu积分的旋转值
        double imu_start = imu_data.front().time;
        double last_imu_time = imu_start;
        Eigen::Quaterniond last_rot(Eigen::Matrix3d::Identity());
        imu_rot_.push_back(last_rot);
        imu_time_.push_back(imu_start);

        for (int i = 0; i < (int)imu_data.size() - 1; i++)
        {
            Eigen::Vector3d gyr1(imu_data[i].angular_velocity.x, imu_data[i].angular_velocity.y, imu_data[i].angular_velocity.z);
            gyr1 = imu2lidar_.rotation() * gyr1;
            Eigen::Vector3d gyr2(imu_data[i + 1].angular_velocity.x, imu_data[i + 1].angular_velocity.y, imu_data[i + 1].angular_velocity.z);
            gyr2 = imu2lidar_.rotation() * gyr2;
            Eigen::Vector3d angle = (imu_data[i + 1].time - imu_data[i].time) * (gyr1 + gyr2) / 2;
            //! 小角度计算旋转矩阵与旋转顺序无关
            Eigen::AngleAxisd roll(angle[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch(angle[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw(angle[2], Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond rot = yaw * pitch * roll;
            imu_rot_.push_back(last_rot * rot);
            imu_time_.push_back(imu_data[i + 1].time);

            last_rot = last_rot * rot;
            imu_num_++;
            if (imu_data[i + 1].time > time_scan_end_ + 0.05)
                break;
        }
        return true;
    }

    /**
     * @description: 提取gnss信息用于去畸变
     * @param {deque<GnssData>} &gnss_data
     * @return {bool}
     */
    bool PreprocessData::gnssDeskewInfo(std::deque<GnssData> &gnss_data)
    {
        //# 判断gnss数据是否有效
        if (gnss_data.size() < 2)
            return false;
        if (gnss_data.front().time >= time_scan_cur_ || gnss_data.back().time < time_scan_end_ + 0.05)
        {
            LOG(ERROR) << "gnss data can not deskew";
            return false;
        }

        gnss_data[0].initOriginPosition();
        Eigen::Quaterniond init_rot(gnss_data[0].orientation.w,
                                    gnss_data[0].orientation.x,
                                    gnss_data[0].orientation.y,
                                    gnss_data[0].orientation.z);
        Eigen::Affine3d init_pose;

        init_pose.rotate(init_rot);
        init_pose.translate(Eigen::Vector3d(gnss_data[0].local_E, gnss_data[0].local_N, gnss_data[0].local_U));

        //# 更新gnss数据的东北天坐标
        for (int i = 0; i < (int)gnss_data.size(); i++)
        {
            gnss_data[i].updateENU();
            Eigen::Vector3d tmp_translation(gnss_data[i].local_E, gnss_data[i].local_N, gnss_data[i].local_U);
            Eigen::Quaterniond tmp_rot(gnss_data[i].orientation.w,
                                       gnss_data[i].orientation.x,
                                       gnss_data[i].orientation.y,
                                       gnss_data[i].orientation.z);

            Eigen::Affine3d tmp_pose;

            tmp_pose.rotate(tmp_rot);
            tmp_pose.translate(tmp_translation);

            gnss_pose_.push_back(tmp_pose * gnss2lidar_.inverse());
            gnss_time_.push_back(gnss_data[i].time);
            gnss_num_++;
            if (gnss_data[i].time > time_scan_end_ + 0.05)
                break;
        }
        return true;
    }

    /**
     * @description: 点云去畸变
     * @param {CloudData} &deskewed_cloud
     * @return {*}
     */
    bool PreprocessData::deskewCloud(CloudData &deskewed_cloud)
    {
        deskewed_cloud.cloud_ptr_->clear();
        deskewed_cloud.time = time_scan_cur_;
        int point_cloud_size = laser_cloud_in_->points.size();
//# 遍历每个点云
#pragma omp parallel for num_threads(number_of_cores_)
        for (int i = 0; i < point_cloud_size; i++)
        {
            CloudData::pointType this_point;
            this_point.x = laser_cloud_in_->points[i].x;
            this_point.y = laser_cloud_in_->points[i].y;
            this_point.z = laser_cloud_in_->points[i].z;
            this_point.intensity = laser_cloud_in_->points[i].intensity;

            double range = pointDistance(this_point);
            if (range < lidar_min_range_ || range > lidar_max_range_)
                continue;

            int row_idn = laser_cloud_in_->points[i].ring;
            if (row_idn < 0 || row_idn >= n_scan_)
                continue;

            if (row_idn % downsample_rate_ != 0)
                continue;

            float horizon_angle = atan2(this_point.x, this_point.y) * 180 / M_PI;

            static float ang_res_x = 360.0 / float(horizon_scan_);
            int column_idn = -round((horizon_angle - 90.0) / ang_res_x) + horizon_scan_ / 2;
            if (column_idn >= horizon_scan_)
                column_idn -= horizon_scan_;

            if (column_idn < 0 || column_idn >= horizon_scan_)
                continue;
            this_point.col = column_idn;
            this_point.ring = row_idn;
            double point_time = laser_cloud_in_->points[i].timestamp;
            if (deskew_flag_ == true)
                deskewPoint(this_point, point_time);
            deskewed_cloud.cloud_ptr_->push_back(this_point);
        }
        return true;
    }

    /**
     * @description: 每个点云去畸变
     * @param {pointType} &point
     * @param {double} point_time
     * @return {*}
     */
    bool PreprocessData::deskewPoint(CloudData::pointType &point, double point_time)
    {
        //# 用imu数据计算当前点的旋转
        int j = 0;
        for (; j < imu_num_; j++)
        {
            if (point_time < imu_time_[j])
            {
                break;
            }
        }
        double ratio = (point_time - imu_time_[j - 1]) / (imu_time_[j] - imu_time_[j - 1]);
        Eigen::Quaterniond cur_point_rot_imu = imu_rot_[j - 1].slerp(ratio, imu_rot_[j]);

        //# 计算当前点到的第一个时刻的gnss的位移和姿态
        // j = 0;
        // for (; j < gnss_num_; j++)
        // {
        //     if (point_time < gnss_time_[j])
        //     {
        //         break;
        //     }
        // }
        // double front_scale = (gnss_time_[j] - point_time) / (gnss_time_[j] - gnss_time_[j - 1]);
        // double back_scale = (point_time - gnss_time_[j - 1]) / (gnss_time_[j] - gnss_time_[j - 1]);
        // double x = front_scale * gnss_pose_[j - 1].translation().x() + back_scale * gnss_pose_[j].translation().x();
        // double y = front_scale * gnss_pose_[j - 1].translation().y() + back_scale * gnss_pose_[j].translation().y();
        // double z = front_scale * gnss_pose_[j - 1].translation().z() + back_scale * gnss_pose_[j].translation().z();
        // Eigen::Vector3d cur_point_trans_gnss(x, y, z);

        // Eigen::Quaterniond cur_point_rot_gnss =
        //     Eigen::Quaterniond(gnss_pose_[j - 1].rotation()).slerp(back_scale, Eigen::Quaterniond(gnss_pose_[j].rotation()));
        //# 如果是第一个点，不需要去畸变，只记录它相对imu和gnss的位姿
        if (frist_point_flag_ == true)
        {
            frist_point_flag_ = false;
            imu_start_rot_inv_ = cur_point_rot_imu.inverse();
            // gnss_start_position_ = cur_point_trans_gnss;
            // gnss_start_rot_ = cur_point_rot_gnss;
        }

        // Eigen::Vector3d cur2ori_trans = gnss_start_rot_.inverse() * (cur_point_trans_gnss - gnss_start_position_);
        Eigen::Quaterniond cur2ori_rot = imu_start_rot_inv_ * cur_point_rot_imu;
        Eigen::Vector3d raw_point(point.x, point.y, point.z);
        Eigen::Vector3d deskewed_point = cur2ori_rot * raw_point;
        point.x = deskewed_point.x();
        point.y = deskewed_point.y();
        point.z = deskewed_point.z();
    }

    /**
     * @description: 计算点云向量模长
     * @param {pointType} point
     * @return {*}
     */
    double PreprocessData::pointDistance(CloudData::pointType point)
    {
        return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    }
}