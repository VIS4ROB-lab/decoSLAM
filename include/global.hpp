//
// Created by philipp on 20.06.22.
//

#pragma once

#include <boost/functional/hash.hpp>
#include <cstddef>
#include <filesystem>
#include <tuple>

#include <hyper/variables/groups/forward.hpp>

namespace deco {
// Float type.
using Scalar = double;

// Ids.
using Id = uint64_t;
using StateId = std::pair<Id, Id>;
using Timestamp = uint64_t;
struct ObservationKey {
    StateId frame_id;
    StateId map_point_id;
    size_t camera_index;
    int keypoint_index;
    auto operator<(const ObservationKey& other) const -> bool;
    auto operator==(const ObservationKey& other) const -> bool;
};
template <typename Type> using Hash = boost::hash<Type>;

// Storage Actions.
enum class StorageAction {
    ADD,
    REMOVE
};

// Geometry.
using Translation = hyper::Position<Scalar>;
using Position = hyper::Position<Scalar>;
using Bearing = hyper::Bearing<Scalar>;
using Pixel = hyper::Pixel<Scalar>;
using SU2 = hyper::SU2<Scalar>;
using SE3 = hyper::SE3<Scalar>;

// Parameter struct
struct MatchingParameters {
    Scalar matching_threshold;
    Scalar ransac_threshold;
    Scalar epipolar_threshold;
    Scalar projection_threshold;
};

// Stream operators for state ids and observation keys.
auto operator<<(std::ostream& os, const StateId& id) -> std::ostream&;
auto operator<<(std::ostream& os, const ObservationKey& key) -> std::ostream&;

// File path
using FilePath = std::filesystem::path;

// TODO: Move to config.
constexpr auto kMinCovisibilityWeight = 10;

} // namespace deco
