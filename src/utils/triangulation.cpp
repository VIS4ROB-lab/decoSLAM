//
// Created by philipp on 20.06.22.
//

#include <hyper/variables/bearing.hpp>
#include <hyper/variables/groups/se3.hpp>
#include <opencv2/features2d.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/triangulation/methods.hpp>

#include "utils/triangulation.hpp"

namespace deco {
auto stereoTriangulate(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    std::vector<cv::DMatch>& matches,
    const SE3& se3_left_right) -> std::vector<std::pair<Position, cv::DMatch>> {
    const opengv::rotation_t rotation_matrix = se3_left_right.rotation().toRotationMatrix();

    // Triangulate match, store point if successful, remove match if not.
    std::vector<std::pair<Position, cv::DMatch>> triangulated_points;
    auto triangulate_remove = [&](const cv::DMatch& match) -> bool {
        const auto left_bearing = opengv::bearingVectors_t{left_bearings[match.queryIdx]};
        const auto right_bearing = opengv::bearingVectors_t{right_bearings[match.trainIdx]};
        opengv::relative_pose::CentralRelativeAdapter adapter(left_bearing, right_bearing, se3_left_right.translation(), rotation_matrix);
        const opengv::point_t point_in_CL = opengv::triangulation::triangulate2(adapter, 0);
        if (point_in_CL.z() > 0.0) {
            triangulated_points.emplace_back(point_in_CL, match);
            return false;
        } else {
            return true;
        }
    };
    matches.erase(std::remove_if(matches.begin(), matches.end(), triangulate_remove), matches.end());

    return triangulated_points;
}
} // namespace deco
