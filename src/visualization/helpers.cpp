//
// Created by philipp on 05.04.22.
//

#include <map>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "utils/matching.hpp"
#include "visualization/helpers.hpp"

namespace deco {
auto getColor(uint64_t id) -> std::tuple<uint8_t, uint8_t, uint8_t> {
    switch (id) {
        case 0: // red.
            return {255, 0, 0};
        case 1: // green.
            return {0, 255, 0};
        case 2: // blue.
            return {0, 0, 255};
        case 3: // orange.
            return {255, 102, 0};
        case 4: // yellow
            return {255, 255, 0};
        case 5: // violett.
            return {102, 0, 102};
        case 6: // mauve.
            return {153, 0, 255};
        case 7: // dark violett.
            return {0, 204, 153};
        default:
            LOG(FATAL) << "Not enough colors defined.";
    }
}

auto showTracks(
    const std::string& window_name,
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    const std::vector<cv::DMatch>& left_inliers,
    const std::vector<cv::DMatch>& right_inliers,
    const std::vector<cv::KeyPoint>& left_keypoints,
    const std::vector<cv::KeyPoint>& right_keypoints,
    const cv::Scalar& color) -> void {
    // Get inlier keypoints.
    std::vector<cv::KeyPoint> left_inlier_keypoints, right_inlier_keypoints;
    std::map<int, int> left_indices, right_indices;
    for (auto i = 0u; i < left_inliers.size(); ++i) {
        auto kp_idx = left_inliers[i].trainIdx;
        left_inlier_keypoints.emplace_back(left_keypoints[kp_idx]);
        left_indices.insert({kp_idx, i});
    }
    for (auto i = 0u; i < right_inliers.size(); ++i) {
        auto kp_idx = right_inliers[i].trainIdx;
        right_inlier_keypoints.emplace_back(right_keypoints[kp_idx]);
        right_indices.insert({kp_idx, i});
    }

    // Build 2D-2D matches from common inliers.
    auto common_inliers = extractCommonMatches(left_inliers, right_inliers);
    std::vector<cv::DMatch> common_inlier_matches;
    for (const auto& [map_point_idx, left_kp_idx, right_kp_idx] : common_inliers) {
        common_inlier_matches.emplace_back(cv::DMatch{left_indices[left_kp_idx], right_indices[right_kp_idx], 0.0});
    }

    // Draw matches and single points.
    cv::Mat out_img;
    cv::drawMatches(left_image, left_inlier_keypoints,
        right_image, right_inlier_keypoints,
        common_inlier_matches, out_img,
        color, color);
    cv::imshow(window_name, out_img);
    cv::waitKey(1);
}

} // namespace deco
