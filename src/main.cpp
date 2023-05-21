//
// Created by philipp on 03.10.22.
//

#include <signal.h>
#include <memory>
#include <thread>

#include <glog/logging.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "system/system.hpp"

auto mySigintHandler(int sig) -> void {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    LOG(INFO) << "Calling ros::shutdown()";

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char** argv) {
    // Set logging flags.
    FLAGS_log_dir = "../log/";
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = 2;

    // Check correct number of CL arguments.
    if (argc < 2) {
        LOG(ERROR) << "Not enough input arguments. Need to provide the path to a .yaml file.";
        return EXIT_FAILURE;
    }

    // Read the config file.
    const std::string config_file_path{argv[1]};
    YAML::Node yaml_node;
    try {
        yaml_node = YAML::LoadFile(config_file_path);
    } catch (...) {
        LOG(ERROR) << "YAML configuration file at " << config_file_path << " not found.";
        return EXIT_FAILURE;
    }

    // Initialize ROS.
    const auto id_str = yaml_node["id"].as<std::string>();
    const auto node_name = yaml_node["name"].as<std::string>() + "_" + id_str;
    DLOG(INFO) << "Starting agent " << id_str << " with node name " << node_name;
    google::InitGoogleLogging(node_name.c_str());
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    signal(SIGTERM, mySigintHandler);

    // Construct and start system.
    auto system = std::make_unique<deco::System>(nh, yaml_node);
    system->start();

    return EXIT_SUCCESS;
}