/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:35
 * @LastEditTime: 2022-09-09 10:26:33
 * @Description:
 */
#include "front_end/front_end_flow.hpp"
namespace localization
{
    FrontEndFlow ::FrontEndFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        flag_init_gnss_ = false;
        flag_frist_cloud_ = true;
        current_cloud_.cloud_ptr_.reset(new CloudData::cloudType());

        std::string config_file_path = WORKSPACE_PATH + "/config/front_end/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        initSubscriber(node_ptr_, config["subscriber"]);
        initPublisher(node_ptr_, config["publisher"]);
        resetParameter();
    }

    /**
     * @description: 初始化订阅类
     * @param {SharedPtr} node_ptr
     * @param {Node} &config
     * @return {*}
     */
    void FrontEndFlow::initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config)
    {
        cloud_info_subscriber_ = std::make_shared<CloudInfoSubscriber>(
            node_ptr,
            config["cloudInfo"]["topic_name"].as<std::string>(),
            config["cloudInfo"]["queue_size"].as<size_t>(),
            config["policy_history"].as<int>());
    }

    /**
     * @description: 初始化发布类
     * @param {SharedPtr} node_ptr
     * @param {Node} &config
     * @return {*}
     */
    void FrontEndFlow::initPublisher(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config)
    {
        cloud_info_publisher_ = std::make_shared<CloudInfoPublisher>(
            node_ptr,
            config["cloudInfo"]["topic_name"].as<std::string>(),
            config["cloudInfo"]["frame_id"].as<std::string>(),
            config["cloudInfo"]["queue_size"].as<size_t>(),
            config["policy_history"].as<int>());
    }

    void FrontEndFlow::resetParameter()
    {
        current_cloud_.cloud_ptr_->clear();
    }

    /**
     * @description: 前端运行入口
     * @return {*}
     */
    void FrontEndFlow::run()
    {
        while (rclcpp::ok())
        {
            if (!readData())
            {
                usleep(1e5);
                continue;
            }
            if (flag_init_gnss_ == false)
            {
                initGnss();
            }
            if (updateOdometry())
            {
                publishMsg();
            }
            resetParameter();
        }
    }

    /**
     * @description: 读取数据
     * @return {*}
     */
    bool FrontEndFlow::readData()
    {
        cloud_info_subscriber_->accessData(cloud_buff_, gnss_buff_);

        if (cloud_info_buff_.empty())
        {
            return false;
        }

        current_cloud_ = cloud_buff_.front();
        current_gnss_ = gnss_buff_.front();
        cloud_buff_.pop_front();
        gnss_buff_.pop_front();
        return true;
    }

    /**
     * @description: 初始化gnss
     * @return {*}
     */
    void FrontEndFlow::initGnss()
    {
        current_gnss_.initOriginPosition();
        current_gnss_.updateENU();
        flag_init_gnss_ = true;

        return;
    }

    bool FrontEndFlow::updateOdometry()
    {
    }

    bool FrontEndFlow::publishMsg()
    {
    }
}
