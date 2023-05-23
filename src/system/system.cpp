//
// Created by philipp on 03.10.22.
//

#include <mutex>

#include <glog/logging.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <hyper/sensors/camera.hpp>
#include <hyper/variables/intrinsics.hpp>

#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/alignment.hpp"
#include "system/communicator.hpp"
#include "system/local_mapping.hpp"
#include "system/local_tracking.hpp"
#include "system/place_recognition.hpp"
#include "system/system.hpp"
#include "visualization/visualizer.hpp"

namespace deco {

System::System(ros::NodeHandle& node_handle, const YAML::Node& yaml_node)
    : mutex_handler_{StorageMutexHandler()},
      shared_map_{std::make_unique<SharedMap>(yaml_node["id"].as<Id>(), mutex_handler_)},
      local_storage_{std::make_unique<LocalStorage>(yaml_node["id"].as<Id>(), mutex_handler_)},
      communicator_{std::make_unique<Communicator>(yaml_node["id"].as<Id>())},
      local_tracking_{std::make_unique<LocalTracking>(yaml_node["id"].as<Id>(), mutex_handler_)},
      local_mapping_{std::make_unique<LocalMapping>(mutex_handler_)},
      place_recognition_{std::make_unique<PlaceRecognition>(yaml_node["id"].as<Id>(), mutex_handler_)},
      alignment_{std::make_unique<Alignment>(yaml_node["id"].as<Id>(), mutex_handler_)},
      async_spinner_{1, &callback_queue_},
      visualizer_{std::make_unique<Visualizer>(yaml_node["id"].as<Id>(), node_handle)},
      output_path_{std::filesystem::current_path() / ("agent_" + yaml_node["id"].as<std::string>())} {
    const auto& system_node = yaml_node["system"];
    LOG(INFO) << "Initializing sensors...";
    initializeSensors_(node_handle, system_node["sensors"], yaml_node["id"].as<Id>());
    const auto matching_parameters = readMatchingParameters(system_node["matching_parameters"]);

    // Initialize communicator.
    LOG(INFO) << "Initializing modules...";
    communicator_->readParameters(system_node["communicator"]);
    communicator_->setSharedMap(shared_map_.get());
    communicator_->setLocalStorage(local_storage_.get());
    communicator_->setPlaceRecognition(place_recognition_.get());
    communicator_->setAlignment(alignment_.get());

    // Initialize shared map.
    shared_map_->readParameters(system_node["optimizer"]); // For ADMM parameters.
    shared_map_->setLocalStorage(local_storage_.get());
    shared_map_->setCommunicator(communicator_.get());
    shared_map_->setAlignment(alignment_.get());

    // Initialize local storage.
    local_storage_->readParameters(system_node["optimizer"]);
    local_storage_->setSharedMap(shared_map_.get());
    local_storage_->setCommunicator(communicator_.get());
    local_storage_->setLocalTracking(local_tracking_.get());
    local_storage_->setAlignment(alignment_.get());
    local_storage_->setVisualizer(visualizer_.get());

    // Initialize local tracking.
    local_tracking_->readParameters(system_node["local_tracking"]);
    local_tracking_->setMatchingParameters(matching_parameters);
    local_tracking_->setCameras({cameras_[0].get(), cameras_[1].get()});
    local_tracking_->setLocalStorage(local_storage_.get());
    local_tracking_->setLocalMapping(local_mapping_.get());
    local_tracking_->setAlignment(alignment_.get());
    local_tracking_->setVisualizer(visualizer_.get());

    // Initialize local mapping.
    local_mapping_->readParameters(system_node["local_mapping"]);
    local_mapping_->setMatchingParameters(matching_parameters);
    local_mapping_->setSharedMap(shared_map_.get());
    local_mapping_->setLocalStorage(local_storage_.get());
    local_mapping_->setLocalTracking(local_tracking_.get());
    local_mapping_->setPlaceRecognition(place_recognition_.get());

    // Initialize place recognition.
    place_recognition_->readParameters(system_node["place_recognition"]);
    place_recognition_->createWordMap(communicator_->getAllAgentIds());
    place_recognition_->setMatchingParameters(matching_parameters);
    place_recognition_->setSharedMap(shared_map_.get());
    place_recognition_->setLocalStorage(local_storage_.get());
    place_recognition_->setCommunicator(communicator_.get());
    place_recognition_->setAlignment(alignment_.get());

    // Initialize alignment.
    alignment_->setCommunicator(communicator_.get());
    alignment_->setSharedMap(shared_map_.get());
    alignment_->setLocalStorage(local_storage_.get());
}

System::~System() = default;

auto System::initializeSensors_(ros::NodeHandle& node_handle, const YAML::Node& yaml_node, const Id& id) -> void {
    const FilePath sensor_config_file = yaml_node["sensor_config_file"].as<std::string>();

    // Read the sensor config file.
    YAML::Node sensor_node;
    try {
        sensor_node = YAML::LoadFile(sensor_config_file);
    } catch (...) {
        LOG(ERROR) << "YAML sensor configuration file at " << sensor_config_file << " not found.";
    }

    // Initialize sensors.
    cameras_.emplace_back(std::make_unique<hyper::Camera>(sensor_node["cam0"]));
    cameras_.emplace_back(std::make_unique<hyper::Camera>(sensor_node["cam1"]));

    // Initialize ROS subscribers.
    constexpr auto kQueueSize = 100;
    std::vector<std::string> topics;
    const auto topic_ns = "agent_" + std::to_string(id);
    topics.emplace_back("/" + topic_ns + sensor_node["cam0"]["topic"].as<std::string>());
    topics.emplace_back("/" + topic_ns + sensor_node["cam1"]["topic"].as<std::string>());
    for (auto i = 0u; i < cameras_.size(); ++i) {
        auto callback = [this, i](auto&& arg) -> void { local_tracking_->callback(arg, i); };
        auto subscribe_options = ros::SubscribeOptions::create<sensor_msgs::Image>(topics[i], kQueueSize, callback, ros::VoidPtr(), &callback_queue_);
        subscribe_options.allow_concurrent_callbacks = false;
        image_subscribers_.emplace_back(node_handle.subscribe(subscribe_options));
    }
}

auto System::readMatchingParameters(const YAML::Node& yaml_node) -> MatchingParameters {
    // Compute geometric thresholds
    const auto left_intrinsics = cameras_[0]->intrinsics();
    const auto right_intrinsics = cameras_[1]->intrinsics();
    const auto average_focal_length = 0.25 * (left_intrinsics.fx() + left_intrinsics.fy() + right_intrinsics.fx() + right_intrinsics.fy());
    const auto ransac_threshold_px = yaml_node["ransac_threshold"].as<Scalar>();
    const auto ransac_threshold = 1.0 - std::cos(std::atan(ransac_threshold_px / average_focal_length));
    const auto epipolar_threshold_px = yaml_node["epipolar_threshold"].as<Scalar>();
    const auto epipolar_threshold = std::atan(epipolar_threshold_px / average_focal_length);
    const auto projection_threshold_deg = yaml_node["projection_threshold"].as<Scalar>();

    return MatchingParameters{
        .matching_threshold = yaml_node["matching_threshold"].as<Scalar>(),
        .ransac_threshold = ransac_threshold,
        .epipolar_threshold = epipolar_threshold,
        .projection_threshold = projection_threshold_deg * Scalar{M_PI} / Scalar{180}};
}

auto System::start() -> void {
    LOG(INFO) << "Starting decoSLAM...";
    communicator_->start();
    local_storage_->start();
    local_mapping_->start();
    place_recognition_->start();

    async_spinner_.start();
    while (!ros::isShuttingDown()) {
        ros::spinOnce();
        sleep(1);
    }

    LOG(INFO) << "Stopping decoSLAM...";
    communicator_->stop();
    LOG(INFO) << "Communicator shut down.";
    local_storage_->stop();
    LOG(INFO) << "Local Storage shut down.";
    local_mapping_->stop();
    LOG(INFO) << "Local Mapping shut down.";
    place_recognition_->stop();
    LOG(INFO) << "Place Recognition shut down.";
    LOG(INFO) << "decoSLAM shut down successfully.";

    LOG(INFO) << "Writing results to " << output_path_;
    std::filesystem::create_directory(output_path_);
    shared_map_->writeLogs(output_path_);
    local_storage_->writePoses(output_path_);
    local_storage_->writeLogs(output_path_);
    communicator_->writeLogs(output_path_);
    LOG(INFO) << "Writing results done.";
}

} // namespace deco
