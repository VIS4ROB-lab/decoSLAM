//
// Created by philipp on 08.07.22.
//

#pragma once

#include <map>
#include <mutex>
#include <set>

#include "global.hpp"

namespace deco {

class Map {
  public:
    Map();
    ~Map();

    auto addObservation(const ObservationKey& key) -> void;
    auto hasObservation(const ObservationKey& key) const -> bool;
    auto hasObservationForCamera(const StateId& frame_id, const StateId& map_point_id, size_t cam_idx) const -> bool;
    auto removeObservation(const ObservationKey& key) -> void;
    auto getObservations() const -> const std::set<ObservationKey>;
    auto hasObservationForMapPoint(const StateId& frame_id, const StateId& map_point_id) const -> bool;
    [[nodiscard]] auto getObservationsForFrame(const StateId& frame_id) const -> std::set<ObservationKey>;
    [[nodiscard]] auto getObservationsForCamera(const StateId& frame_id, size_t cam_idx) const -> std::set<ObservationKey>;
    [[nodiscard]] auto getMapPointObservationForCamera(const StateId& frame_id, const StateId& map_point_id, size_t cam_idx) const -> ObservationKey;
    [[nodiscard]] auto getObservationsForMapPoint(const StateId& map_point_id) const -> std::set<ObservationKey>;
    [[nodiscard]] auto getMapPointIds() const -> std::set<StateId>;
    [[nodiscard]] auto getFrameIds() const -> std::set<StateId>;
    auto hasMapPoint(const StateId& map_point_id) const -> bool;

    [[nodiscard]] auto getObservedMapPoints(const StateId& frame_id) const -> std::set<StateId>;
    [[nodiscard]] auto getObservingFrames(const StateId& map_point_id) const -> std::set<StateId>;

  private:
    mutable std::mutex mutex_;
    std::set<ObservationKey> observations_;
    std::map<StateId, std::map<size_t, std::map<StateId, int>>> observed_map_points_;
    std::map<StateId, std::set<std::tuple<StateId, size_t, int>>> observing_frames_;
};
} // namespace deco
