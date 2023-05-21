//
// Created by philipp on 08.07.22.
//

#include <glog/logging.h>

#include "global.hpp"
#include "map/map.hpp"

namespace deco {
Map::Map() = default;

Map::~Map() = default;

auto Map::addObservation(const ObservationKey& key) -> void {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    // DLOG(INFO) << key;

    // Store observation.
    auto [obs_itr, obs_inserted] = observations_.insert(key);
    DCHECK(obs_inserted) << key;

    // Add frame -> map point.
    auto frame_itr = observed_map_points_.find(key.frame_id);
    if (frame_itr == observed_map_points_.end()) {
        frame_itr = observed_map_points_.insert({key.frame_id, {}}).first;
    }
    auto& cam_map = frame_itr->second;
    auto cam_itr = cam_map.find(key.camera_index);
    if (cam_itr == cam_map.end()) {
        cam_itr = cam_map.insert({key.camera_index, {}}).first;
    }
    auto [_, point_inserted] = cam_itr->second.insert({key.map_point_id, key.keypoint_index});
    DCHECK(point_inserted) << "Observation " << key.map_point_id << " --> " << key.keypoint_index << " already exists.";

    // Add map point -> frame.
    auto map_point_itr = observing_frames_.find(key.map_point_id);
    if (map_point_itr == observing_frames_.end()) {
        observing_frames_.insert({key.map_point_id, {{key.frame_id, key.camera_index, key.keypoint_index}}});
    } else {
        map_point_itr->second.insert({key.frame_id, key.camera_index, key.keypoint_index});
    }
}

auto Map::hasObservation(const ObservationKey& key) const -> bool {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    return observations_.contains(key);
}

auto Map::hasObservationForCamera(const StateId& frame_id, const StateId& map_point_id, size_t cam_idx) const -> bool {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    const auto frame_itr = observed_map_points_.find(frame_id);
    if (frame_itr == observed_map_points_.end()) {
        return false;
    }
    const auto& cam_map = frame_itr->second;
    auto cam_itr = cam_map.find(cam_idx);
    if (cam_itr == cam_map.end()) {
        return false;
    } else {
        return cam_itr->second.contains(map_point_id);
    }
}

auto Map::removeObservation(const ObservationKey& key) -> void {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    // Remove observation.
    DCHECK(observations_.contains(key)) << key;
    observations_.erase(key);

    // Remove frame -> map point.
    DCHECK(observed_map_points_.contains(key.frame_id));
    auto& cam_map = observed_map_points_.find(key.frame_id)->second;
    DCHECK(cam_map.contains(key.camera_index));
    auto& map_point_map = cam_map.find(key.camera_index)->second;
    DCHECK(map_point_map.contains(key.map_point_id));
    DCHECK(map_point_map.find(key.map_point_id)->second == key.keypoint_index);
    map_point_map.erase(key.map_point_id);
    if (map_point_map.empty()) {
        cam_map.erase(key.camera_index);
    }
    if (cam_map.empty()) {
        observed_map_points_.erase(key.frame_id);
    }

    // Remove map point -> frame.
    DCHECK(observing_frames_.contains(key.map_point_id));
    auto& frame_camera_pairs = observing_frames_.find(key.map_point_id)->second;
    DCHECK(frame_camera_pairs.contains({key.frame_id, key.camera_index, key.keypoint_index}));
    frame_camera_pairs.erase({key.frame_id, key.camera_index, key.keypoint_index});
    if (frame_camera_pairs.empty()) {
        observing_frames_.erase(key.map_point_id);
    }
}

auto Map::getObservationsForFrame(const StateId& frame_id) const -> std::set<ObservationKey> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto frame_itr = observed_map_points_.find(frame_id);
    if (frame_itr == observed_map_points_.end()) {
        return {};
    } else {
        std::set<ObservationKey> observations;
        for (const auto& [cam_idx, map_point_map] : frame_itr->second) {
            for (const auto& [map_point_id, keypoint_index] : map_point_map) {
                observations.insert(ObservationKey{.frame_id = frame_itr->first,
                    .map_point_id = map_point_id,
                    .camera_index = cam_idx,
                    .keypoint_index = keypoint_index});
            }
        }
        return observations;
    }
}

auto Map::getObservationsForCamera(const StateId& frame_id, size_t cam_idx) const -> std::set<ObservationKey> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    const auto frame_itr = observed_map_points_.find(frame_id);
    if (frame_itr == observed_map_points_.end()) {
        return {};
    }
    const auto& cam_map = frame_itr->second;
    const auto cam_itr = cam_map.find(cam_idx);
    if (cam_itr == cam_map.end()) {
        return {};
    }
    const auto& map_point_map = cam_itr->second;
    auto observations = std::set<ObservationKey>();
    for (const auto& [map_point_id, keypoint_index] : map_point_map) {
        observations.insert(ObservationKey{.frame_id = frame_id,
            .map_point_id = map_point_id,
            .camera_index = cam_idx,
            .keypoint_index = keypoint_index});
    }
    return observations;
}

auto Map::getMapPointObservationForCamera(const StateId& frame_id, const StateId& map_point_id, size_t cam_idx) const -> ObservationKey {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    const auto frame_itr = observed_map_points_.find(frame_id);
    CHECK(frame_itr != observed_map_points_.end());
    const auto& cam_map = frame_itr->second;
    auto cam_itr = cam_map.find(cam_idx);
    CHECK(cam_itr != cam_map.end());
    CHECK(cam_itr->second.contains(map_point_id));
    const auto kp_idx = cam_itr->second.find(map_point_id)->second;
    return {.frame_id = frame_id, .map_point_id = map_point_id, .camera_index = cam_idx, .keypoint_index = kp_idx};
}

auto Map::getObservationsForMapPoint(const StateId& map_point_id) const -> std::set<ObservationKey> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto observations = std::set<ObservationKey>();
    auto itr = observing_frames_.find(map_point_id);
    if (itr == observing_frames_.end()) {
        return {};
    }

    for (const auto& [frame_id, cam_idx, keypoint_index] : itr->second) {
        observations.insert(ObservationKey{.frame_id = frame_id,
            .map_point_id = map_point_id,
            .camera_index = cam_idx,
            .keypoint_index = keypoint_index});
    }
    return observations;
}

auto Map::getObservations() const -> const std::set<ObservationKey> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    return observations_;
}

auto Map::hasObservationForMapPoint(const StateId& frame_id, const StateId& map_point_id) const -> bool {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    const auto frame_itr = observed_map_points_.find(frame_id);
    if (frame_itr == observed_map_points_.end()) {
        return false;
    }
    const auto& cam_map = frame_itr->second;
    bool has_observation = false;
    for (const auto& [_, map_point_map] : cam_map) {
        has_observation |= map_point_map.contains(map_point_id);
    }
    return has_observation;
}

auto Map::getMapPointIds() const -> std::set<StateId> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto map_point_ids = std::set<StateId>();
    for (const auto& [map_point_id, _] : observing_frames_) {
        map_point_ids.insert(map_point_id);
    }
    return map_point_ids;
}

auto Map::getFrameIds() const -> std::set<StateId> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto frame_ids = std::set<StateId>();
    for (const auto& [frame_id, _] : observed_map_points_) {
        frame_ids.insert(frame_id);
    }
    return frame_ids;
}

auto Map::hasMapPoint(const StateId& map_point_id) const -> bool {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    return observing_frames_.contains(map_point_id);
}

auto Map::getObservedMapPoints(const StateId& frame_id) const -> std::set<StateId> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    const auto frame_itr = observed_map_points_.find(frame_id);
    if (frame_itr == observed_map_points_.end()) {
        return {};
    } else {
        const auto& cam_map = frame_itr->second;
        auto observed_points = std::set<StateId>();
        for (const auto& [cam_idx, map_point_map] : cam_map) {
            for (const auto& [map_point_id, keypoint_index] : map_point_map) {
                observed_points.insert(map_point_id);
            }
        }
        return observed_points;
    }
}

auto Map::getObservingFrames(const StateId& map_point_id) const -> std::set<StateId> {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    const auto map_point_itr = observing_frames_.find(map_point_id);
    if (map_point_itr == observing_frames_.end()) {
        return {};
    } else {
        std::set<StateId> observing_frames;
        for (const auto& [frame_id, cam_idx, keypoint_idx] : map_point_itr->second) {
            observing_frames.insert(frame_id);
        }
        return observing_frames;
    }
}
} // namespace deco
