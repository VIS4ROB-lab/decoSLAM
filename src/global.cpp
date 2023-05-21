//
// Created by philipp on 21.08.22.
//

#include "global.hpp"

namespace deco {
auto ObservationKey::operator<(const ObservationKey& other) const -> bool {
    using Tuple = std::tuple<StateId, StateId, size_t, int>;
    return Tuple{this->frame_id, this->map_point_id, this->camera_index, this->keypoint_index} < Tuple{other.frame_id, other.map_point_id, other.camera_index, other.keypoint_index};
}
auto ObservationKey::operator==(const ObservationKey& other) const -> bool {
    using Tuple = std::tuple<StateId, StateId, size_t, int>;
    return Tuple{this->frame_id, this->map_point_id, this->camera_index, this->keypoint_index} == Tuple{other.frame_id, other.map_point_id, other.camera_index, other.keypoint_index};
}

auto operator<<(std::ostream& os, const StateId& id) -> std::ostream& {
    os << "(" << id.first << ", " << id.second << ")";
    return os;
}

auto operator<<(std::ostream& os, const ObservationKey& key) -> std::ostream& {
    os << "[" << key.frame_id << ", " << key.map_point_id << ", " << key.camera_index << ", " << key.keypoint_index << "]";
    return os;
}



} // namespace deco
