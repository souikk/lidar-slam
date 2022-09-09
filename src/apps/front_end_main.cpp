/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 15:42:07
 * @LastEditTime: 2022-09-07 17:25:49
 * @Description:前端节点主程序
 */
#include <chrono>
#include <thread>

#include "glog/logging.h"

#include "front_end/front_end_node.hpp"
#include "front_end/front_end_flow.hpp"
#include "global_path.h"

using namespace std::chrono_literals;
using namespace localization;

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORKSPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    std::string config_file_path = WORKSPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config = YAML::LoadFile(config_file_path);
    int policy_history = config["subscriber"]["policy_history"].as<int>();

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    if (policy_history == 1)
    {
        options.use_intra_process_comms(true);
    }
    else
    {
        options.use_intra_process_comms(false);
    }
    rclcpp::Node::SharedPtr front_end_node = std::make_shared<FrontEndNode>(options);
    std::shared_ptr<FrontEndFlow> front_end_flow = std::make_shared<FrontEndFlow>(front_end_node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 0, true);

    executor.add_node(front_end_node);

    std::thread front_end_thread(&FrontEndFlow::run, front_end_flow);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}