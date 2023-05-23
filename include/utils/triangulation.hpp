//
// Created by philipp on 20.06.22.
//

#pragma once
#include <vector>

#include "global.hpp"

namespace cv {
class DMatch;
}

namespace deco {
/// Triangulate a set of left-right matches from a stereo camera pair.
/// \param left_bearings Bearings from the left camera.
/// \param right_bearings Bearings from the right camera.
/// \param matches Matches left-to-right.
/// \param se3_left_right Transformation of right camera w.r.t. left camera.
/// \param se3_world_left Transformation of the left camera w.r.t. world frame.
/// \return Vector of triangulated positions expressed in the world frame.
auto stereoTriangulate(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    std::vector<cv::DMatch>& matches,
    const SE3& se3_left_right) -> std::vector<std::pair<Position, cv::DMatch>>;

} // namespace deco
