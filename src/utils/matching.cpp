//
// Created by philipp on 20.06.22.
//

#include <hyper/variables/bearing.hpp>
#include <opencv2/features2d.hpp>

#include "utils/matching.hpp"

namespace deco {

auto descriptorMatching(const cv::Mat& descriptors_A, const cv::Mat& descriptors_B, Scalar matching_threshold, const cv::Mat* mask) -> Matches {
    // Find all possible matches within a threshold.
    auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    auto matches_list = std::vector<Matches>();
    if (mask) {
        CHECK(mask->rows == descriptors_A.rows && mask->cols == descriptors_B.rows);
        matcher->radiusMatch(descriptors_A, descriptors_B, matches_list, static_cast<float>(matching_threshold), *mask, true);
    } else {
        matcher->radiusMatch(descriptors_A, descriptors_B, matches_list, static_cast<float>(matching_threshold));
    }

    // Erase unmatched keypoints and apply ratio test.
    auto remove_function = [](const auto& arg) -> bool {
        return (arg.empty() || (arg.size() != 1 && (arg[0].distance < kRatioTestThreshold * arg[1].distance)));
    };
    matches_list.erase(std::remove_if(matches_list.begin(), matches_list.end(), remove_function), matches_list.end());

    // Sort by matched index and score.
    auto sort_function = [](const auto& arg1, const auto& arg2) -> bool {
        return arg1[0].trainIdx == arg2[0].trainIdx ? arg1[0].distance < arg2[0].distance : arg1[0].trainIdx < arg2[0].trainIdx;
    };
    std::sort(matches_list.begin(), matches_list.end(), sort_function);

    // For multiple matches to the same keypoint, keep only the best.
    auto unique_function = [](const auto& arg1, const auto& arg2) { return arg1[0].trainIdx == arg2[0].trainIdx; };
    matches_list.erase(std::unique(matches_list.begin(), matches_list.end(), unique_function), matches_list.end());

    // Flatten vector.
    Matches matches;
    matches.reserve(matches_list.size());
    for (auto& match : matches_list) {
        matches.emplace_back(match[0]);
    }

    // Sort again by query index (indeces of first argument).
    std::sort(matches.begin(), matches.end(), [](const auto& arg1, const auto& arg2) { return arg1.queryIdx < arg2.queryIdx; });

    return matches;
}

auto projectionMask(const std::vector<Translation>& map_point_positions,
    const std::vector<Bearing>& bearings,
    const SE3& se3_camera_world,
    const Scalar projection_threshold) -> cv::Mat {
    const auto threshold = std::cos(projection_threshold);

    // Initially disallow all matches.
    cv::Mat mask = cv::Mat::zeros(static_cast<int>(map_point_positions.size()), static_cast<int>(bearings.size()), CV_8U);

    for (auto i = 0; i < mask.rows; ++i) {
        // Project map point.
        const Translation point_in_camera = se3_camera_world.vectorPlus(map_point_positions[i]);
        const Translation normalized_point = point_in_camera.normalized();

        // Allow matches to bearings which are within the threshold.
        for (auto j = 0; j < mask.cols; ++j) {
            if (bearings[j].dot(normalized_point) >= threshold) {
                mask.at<uchar>(i, j) = 1;
            }
        }
    }
    return mask;
}

auto epipolarMask(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    const Eigen::Matrix3d& essential_matrix,
    const Scalar threshold) -> cv::Mat {
    // Initially disallow all matches.
    cv::Mat mask = cv::Mat::zeros(static_cast<int>(left_bearings.size()), static_cast<int>(right_bearings.size()), CV_8U);

    // Only allow pairs that satisfy epipolar constraint.
    for (auto i = 0; i < mask.rows; ++i) {
        const Eigen::Vector3d& left_bearing = left_bearings[i];
        for (auto j = 0; j < mask.cols; ++j) {
            const Bearing& right_bearing = right_bearings[j];
            const Bearing transformed_bearing = (essential_matrix * right_bearing).normalized();
            if (std::abs(left_bearing.dot(transformed_bearing)) <= threshold) {
                mask.at<uchar>(i, j) = 1;
            }
        }
    }
    return mask;
}

auto extractCommonMatches(const std::vector<cv::DMatch>& matches_A,
    const std::vector<cv::DMatch>& matches_B) -> std::vector<std::tuple<int, int, int>> {
    auto sort_function = [](const auto& arg1, const auto& arg2) { return arg1.queryIdx < arg2.queryIdx; };
    CHECK(std::is_sorted(matches_A.begin(), matches_A.end(), sort_function));
    CHECK(std::is_sorted(matches_B.begin(), matches_B.end(), sort_function));

    auto left_itr = matches_A.begin();
    auto right_itr = matches_B.begin();
    std::vector<std::tuple<int, int, int>> common_matches;
    while (left_itr != matches_A.end() && right_itr != matches_B.end()) {
        auto left_index = left_itr->queryIdx;
        auto right_index = right_itr->queryIdx;

        // Common match.
        if (left_index == right_index) {
            common_matches.emplace_back(std::tuple<int, int, int>{left_index, left_itr->trainIdx, right_itr->trainIdx});
            ++left_itr;
            ++right_itr;
        } else if (left_index < right_index) {
            ++left_itr;
        } else {
            ++right_itr;
        }
    }
    return common_matches;
}

auto filterEpipolar3D2D(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    std::vector<cv::DMatch>& left_matches,
    std::vector<cv::DMatch>& right_matches,
    const Eigen::Matrix3d& essential_matrix,
    Scalar epipolar_threshold) -> void {
    auto common_matches = extractCommonMatches(left_matches, right_matches);
    for (const auto& indeces : common_matches) {
        const int left_kp_idx = std::get<1>(indeces);
        const int right_kp_idx = std::get<2>(indeces);
        const Bearing& left_bearing = left_bearings[left_kp_idx];
        const Bearing& right_bearing = right_bearings[right_kp_idx];
        const Bearing transformed_bearing = (essential_matrix * right_bearing).normalized();
        if (std::abs(left_bearing.dot(transformed_bearing)) > epipolar_threshold) {
            auto left_itr = std::find_if(left_matches.begin(), left_matches.end(), [&](const cv::DMatch& arg) -> bool { return arg.trainIdx == left_kp_idx; });
            auto right_itr = std::find_if(right_matches.begin(), right_matches.end(), [&](const cv::DMatch& arg) -> bool { return arg.trainIdx == right_kp_idx; });
            left_matches.erase(left_itr);
            right_matches.erase(right_itr);
        }
    }
}

auto filterEpipolar2D2D(const std::vector<Bearing>& left_bearings,
    const std::vector<Bearing>& right_bearings,
    Matches& matches,
    const Eigen::Matrix3d& essential_matrix,
    Scalar epipolar_threshold) -> void {
    for (auto itr = matches.begin(); itr != matches.end();) {
        auto& match = *itr;
        const Bearing& left_bearing = left_bearings[match.queryIdx];
        const Bearing& right_bearing = right_bearings[match.trainIdx];
        const Bearing transformed_bearing = (essential_matrix * right_bearing).normalized();
        if (std::abs(left_bearing.dot(transformed_bearing)) > epipolar_threshold) {
            itr = matches.erase(itr);
        } else {
            ++itr;
        }
    }
}

} // namespace deco
