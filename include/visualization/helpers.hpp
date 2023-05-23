//
// Created by philipp on 05.04.22.
//

#pragma once

#include <tuple>
#include <vector>

#include <opencv2/core.hpp>

namespace deco {
/// Returns some hard-coded colors.
/// \param id Id of the color (current range is 0-4).
/// \return Tuple of the form <red, green, blue>.
auto getColor(uint64_t id) -> std::tuple<uint8_t, uint8_t, uint8_t>;

auto showTracks(
    const std::string& window_name,
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    const std::vector<cv::DMatch>& left_inliers,
    const std::vector<cv::DMatch>& right_inliers,
    const std::vector<cv::KeyPoint>& left_keypoints,
    const std::vector<cv::KeyPoint>& right_keypoints,
    const cv::Scalar& color) -> void;

} // namespace deco
