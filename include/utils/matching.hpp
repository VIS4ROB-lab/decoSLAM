//
// Created by philipp on 20.06.22.
//

#pragma once

#include <vector>

#include "global.hpp"
#include "map/frame.hpp"

namespace cv {
class DMatch;
class Mat;
} // namespace cv

namespace deco {
constexpr auto kRatioTestThreshold = 0.6;
using Matches = std::vector<cv::DMatch>;

/// Brute force matching of two descriptor blocks.
/// \param descriptors_A First block of descriptors.
/// \param descriptors_B Second block of descriptors.
/// \param matching_threshold Threshold for radius match.
/// \param mask Binary mask allowing / disallowing some matches.
/// \return Vector of matched indices.
auto descriptorMatching(const cv::Mat& descriptors_A,
    const cv::Mat& descriptors_B,
    Scalar matching_threshold,
    const cv::Mat* mask = nullptr) -> Matches;

/// Create a mask to only allows beearings that are close to a map point projection for matching.
/// Should be used with 3D to 2D matching.
/// \param map_point_positions Map point positions to check, expressed in world frame.
/// \param bearings Bearings to check.
/// \param se3_camera_world Transformation of the camera w.r.t. the world frame.
/// \param projection_threshold Radius (in radians) for a projection to be considered for matching.
/// \return Matching mask.
auto projectionMask(const std::vector<Translation>& map_point_positions,
    const std::vector<Bearing>& bearings,
    const SE3& se3_camera_world,
    Scalar projection_threshold) -> cv::Mat;

/// Create a mask that only allows bearings pairs that satisfy the epipolar constraint for matching.
/// \param left_bearings Bearings from the left camera.
/// \param right_bearings Bearings from the right camera.
/// \param essential_matrix Essential matrix between the cameras.
/// \param threshold Threshold for epipolar constraint satisfaction.
/// \return Matching mask.
auto epipolarMask(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    const Eigen::Matrix3d& essential_matrix,
    Scalar threshold) -> cv::Mat;

/// Combines two sets of matches using the query index.
/// \param matches_A First set of matches.
/// \param matches_B Second set of matches.
/// \return Vector of tuples of the form <query index, matched index A, matched index B>.
auto extractCommonMatches(const Matches& matches_A, const Matches& matches_B) -> std::vector<std::tuple<int, int, int>>;

/// Filter 3D to 2D matches from a stereo camera pair using the epipolar constraint.
/// \param left_bearings Bearings of the left camera.
/// \param right_bearings Bearings of the right camera.
/// \param left_matches 3D to 2D matches from the left camera.
/// \param right_matches 3D to 2D matches from the right camera.
/// \param essential_matrix Essential matrix left-to-right.
/// \param epipolar_threshold Epipolar threshold.
auto filterEpipolar3D2D(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    Matches& left_matches,
    Matches& right_matches,
    const Eigen::Matrix3d& essential_matrix,
    Scalar epipolar_threshold) -> void;

auto filterEpipolar2D2D(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    Matches& matches,
    const Eigen::Matrix3d& essential_matrix,
    Scalar epipolar_threshold) -> void;

} // namespace deco
