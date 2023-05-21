//
// Created by philipp on 03.10.22.
//

#pragma once

#include <ros/callback_queue.h>
#include <ros/subscriber.h>

#include "global.hpp"
#include "utils/processing.hpp"

namespace YAML {
class Node;
}

namespace hyper {
class Camera;
}

namespace deco {
class SharedMap;
class LocalStorage;
class Communicator;
class LocalTracking;
class LocalMapping;
class PlaceRecognition;
class Alignment;
class Optimizer;
class Visualizer;
struct MatchingParameters;

class System {
  public:
    System(ros::NodeHandle& node_handle, const YAML::Node& system_node);
    System() = delete;
    ~System();

    auto start() -> void;

  private:
    auto initializeSensors_(ros::NodeHandle& node_handle, const YAML::Node& yaml_node, const Id& id) -> void;
    auto readMatchingParameters(const YAML::Node& yaml_node) -> MatchingParameters;

    // Storage.
    StorageMutexHandler mutex_handler_;
    std::unique_ptr<SharedMap> shared_map_;
    std::unique_ptr<LocalStorage> local_storage_;

    // Modules.
    std::unique_ptr<Communicator> communicator_;
    std::unique_ptr<LocalTracking> local_tracking_;
    std::unique_ptr<LocalMapping> local_mapping_;
    std::unique_ptr<PlaceRecognition> place_recognition_;
    std::unique_ptr<Alignment> alignment_;
    // std::unique_ptr<Optimizer> optimizer_;

    // Sensors
    std::vector<std::unique_ptr<hyper::Camera>> cameras_;

    // ROS.
    ros::CallbackQueue callback_queue_;
    std::vector<ros::Subscriber> image_subscribers_;
    ros::AsyncSpinner async_spinner_;

    // Visualization.
    std::unique_ptr<Visualizer> visualizer_;

    // Logging.
    const FilePath output_path_;
};
} // namespace deco
