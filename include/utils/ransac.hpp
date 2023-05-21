//
// Created by philipp on 20.06.22.
//

#pragma once

#include <vector>

#include "global.hpp"
#include "map/frame.hpp"

namespace cv {
class DMatch;
}


namespace deco {
constexpr auto kNumRansacIterations = 150;
constexpr auto kRansacProbability = 0.95;

/// Performs RANSAC on a set of 3D to 2D matches.
/// \param map_point_positions Position of map points.
/// \param camera_bearings Per camera bearings.
/// \param camera_matches Per camera matches.
/// \param t_body_camera Per camera transformations of the sensor frame w.r.t. the body frame.
/// \param ransac_threshold Threshold used for RANSAC inliers.
/// \return Found body to world transformation.
auto ransac3d2d(const std::vector<Translation>& map_point_positions,
    const std::vector<std::vector<Bearing>>& camera_bearings,
    std::vector<std::vector<cv::DMatch>>& camera_matches,
    const std::vector<SE3>& se3_body_camera,
    const SE3& se3_initial,
    Scalar ransac_threshold) -> SE3;

/*/// Performs RANSAC on a set of 3D to 3D matches.
/// \param map_points_A First set of map points.
/// \param map_points_B Second set of map points.
/// \param matches Matches from first to second set.
/// \param ransac_threshold Threshold used for RANSAC inliers.
/// \return Found transformation between map point sets.
auto ransac3d3d(const std::vector<MapPoint*>& map_points_A,
    const std::vector<map::MapPoint*>& map_points_B,
    std::vector<cv::DMatch>& matches,
    Scalar ransac_threshold) -> SE3;

/// Performs RANSAC on a set of 2D to 2D matches.
/// \param bearings_A First set of bearings.
/// \param bearings_B Second set of bearings.
/// \param matches Matches from first to second set.
/// \param ransac_threshold Threshold used for RANSAC inliers.
/// \return Found transformation between bearing sets.
auto ransac2d2d(const std::vector<Bearing>& bearings_A,
    const std::vector<Bearing>& bearings_B,
    std::vector<cv::DMatch>& matches,
    Scalar ransac_threshold) -> SE3;*/
} // namespace deco
