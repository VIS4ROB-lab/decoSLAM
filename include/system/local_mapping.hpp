//
// Created by philipp on 04.10.22.
//

#pragma once

#include "global.hpp"
#include "utils/processing.hpp"

namespace YAML {
class Node;
}

namespace deco {
class SharedMap;
class LocalStorage;
class LocalTracking;
class PlaceRecognition;

class LocalMapping {
  public:
    using Input = std::tuple<std::unique_ptr<Frame>, std::set<ObservationKey>, std::set<StateId>>;

    explicit LocalMapping(StorageMutexHandler& mutex_handler);
    ~LocalMapping();
    auto setMatchingParameters(const MatchingParameters& parameters) -> void;
    auto readParameters(const YAML::Node& node) -> void;
    auto setSharedMap(SharedMap* shared_map) -> void;
    auto setLocalStorage(LocalStorage* local_storage) -> void;
    auto setLocalTracking(LocalTracking* local_tracking) -> void;
    auto setPlaceRecognition(PlaceRecognition* place_recognition) -> void;

    auto start() -> void;
    auto stop() -> void;
    auto submit(Input&& data) -> void;

  private:
    auto process_(Input&& data) -> void;

    auto reprojectIntoLocalFrames_(const std::set<StateId>& local_frame_ids,
        const std::vector<Translation>& points,
        const cv::Mat& new_point_descriptors,
        const std::set<StateId>& new_ids) -> std::set<ObservationKey>;

    StorageMutexHandler& mutex_handler_;
    SharedMap* shared_map_;
    LocalStorage* local_storage_;
    LocalTracking* local_tracking_;
    PlaceRecognition* place_recognition_;

    Id map_point_count_;
    MatchingParameters parameters_;
    std::pair<Scalar, Scalar> triangulation_range_;
    size_t max_num_points_;

    ProcessingQueue<Input> keyframe_queue_;
};

} // namespace deco
