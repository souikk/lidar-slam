/*
 * @Author: Ke Zhang
 * @Date: 2022-09-07 15:12:32
 * @LastEditTime: 2022-09-07 17:49:31
 * @Description:
 */
#include "subscriber/cloud_info_subscriber.hpp"

namespace localization
{
    CloudInfoSubscriber::CloudInfoSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, size_t queue_size, int policy_history)
    {
        node_ptr_ = node_ptr;

        rmw_qos_history_policy_t hp;
        if (policy_history == 0)
        {
            hp = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
        }
        else if (policy_history == 1)
        {
            hp = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        }
        else if (policy_history == 2)
        {
            hp = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        }
        rmw_qos_profile_t qos_profile{
            hp,                                  // history,保留近期记录(Keep last)： 缓存最多N条记录，可通过队列长度选项来配置。
                                                 //保留所有记录(Keep all)：缓存所有记录，但受限于底层中间件可配置的最大资源。
            queue_size,                          // Size of the message queue,队列深度(Size of the queue)：只能与Keep last配合使用。
            RMW_QOS_POLICY_RELIABILITY_RELIABLE, //可靠性(Reliability),尽力的(Best effort)：尝试传输数据但不保证成功传输(当网络不稳定时可能丢失数据)。
                                                 //可靠的(Reliable)：反复重传以保证数据成功传输。
            RMW_QOS_POLICY_DURABILITY_VOLATILE,  //持续性(Durability),局部瞬态(Transient local)：发布器为晚连接(late-joining)的订阅器保留数据。
                                                 //易变态(Volatile)：不保留任何数据。
                                                 //以上每个策略都有系统默认值。这个默认值就是底层中间件的默认值，由DDS供应商工具(如XML配置文件)定义。
                                                 // DDS本身提供了许多可配置的策略。这些策略与ROS1的特征相似，所以在ROS1中是可见的。之后可能会有更多的策略在ROS2中可见。
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false};

        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                qos_profile.history,
                qos_profile.depth),
            qos_profile);
        sub_cloud_info_ = node_ptr_->create_subscription<localization::msg::CloudInfo>(
            topic_name, qos, std::bind(&CloudInfoSubscriber::cloudInfoCallback, this, std::placeholders::_1));
    }

    void CloudInfoSubscriber::cloudInfoCallback(localization::msg::CloudInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(lock_key);
        cloud_info_data_buf_.push_back(*msg);
    }

    void CloudInfoSubscriber::accessData(std::deque<localization::msg::CloudInfo> &cloud_info_buff)
    {
        std::lock_guard<std::mutex> lock(lock_key);
        cloud_info_buff.insert(cloud_info_buff.end(), cloud_info_data_buf_.begin(), cloud_info_data_buf_.end());
        cloud_info_data_buf_.clear();
    }

    void CloudInfoSubscriber::accessData(std::deque<CloudData> &cloud_buff, std::deque<GnssData> &gnss_buff)
    {
        std::lock_guard<std::mutex> lock(lock_key);
        int buff_size = cloud_info_data_buf_.size();
        for (int i = 0; i < buff_size; i++)
        {
            localization::msg::CloudInfo cloud_info = cloud_info_data_buf_[i];
            CloudData cloud;
            GnssData gnss;
            cloud.cloud_ptr_.reset(new CloudData::cloudType());
            pcl::moveFromROSMsg(cloud_info.cloud_deskewed, *cloud.cloud_ptr_);
            cloud.time = rclcpp::Time(cloud_info.header.stamp).seconds();
            gnss.time = cloud.time;
            gnss.latitude = cloud_info.gnss_lat;
            gnss.longitude = cloud_info.gnss_lon;
            gnss.altitude = cloud_info.gnss_h;
            gnss.orientation.w = cloud_info.gnss_orientation_w;
            gnss.orientation.x = cloud_info.gnss_orientation_x;
            gnss.orientation.y = cloud_info.gnss_orientation_y;
            gnss.orientation.z = cloud_info.gnss_orientation_z;

            cloud_buff.push_back(cloud);
            gnss_buff.push_back(gnss);
        }

        cloud_info_data_buf_.clear();
    }
} // namespace localization
