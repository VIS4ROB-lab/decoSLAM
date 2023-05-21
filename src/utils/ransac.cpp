//
// Created by philipp on 20.06.22.
//

#include <hyper/variables/bearing.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "utils/common.hpp"
#include "utils/ransac.hpp"

namespace deco {
auto ransac3d2d(const std::vector<Translation>& map_point_positions,
    const std::vector<std::vector<Bearing>>& bearings,
    std::vector<std::vector<cv::DMatch>>& camera_matches,
    const std::vector<SE3>& se3_body_camera,
    const SE3& se3_initial,
    const Scalar ransac_threshold) -> SE3 {
    // Opengv types.
    using Adapter = opengv::absolute_pose::NoncentralAbsoluteAdapter;
    using SacProblem = opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem;
    using Ransac = opengv::sac::Ransac<SacProblem>;

    const auto num_cameras = camera_matches.size();

    // Build OpenGV containers.
    opengv::bearingVectors_t opengv_bearings;
    opengv::points_t opengv_points;
    std::vector<int> opengv_cam_correspondences;
    opengv::translations_t cam_offsets;
    opengv::rotations_t cam_rotations;
    std::vector<std::pair<int, cv::DMatch>> linear_indexes;
    for (auto i = 0u; i < num_cameras; ++i) {
        for (auto& match : camera_matches[i]) {
            linear_indexes.emplace_back(std::pair<int, cv::DMatch>{i, match});
            opengv_points.push_back(map_point_positions[match.queryIdx]);
            opengv_bearings.push_back(bearings[i][match.trainIdx]);
            opengv_cam_correspondences.push_back(static_cast<int>(i));
        }
        opengv::rotation_t rotation_matrix = se3_body_camera[i].rotation().toRotationMatrix();
        opengv::translation_t translation = se3_body_camera[i].translation();
        cam_offsets.push_back(translation);
        cam_rotations.push_back(rotation_matrix);
    }
    auto adapter = Adapter{opengv_bearings, opengv_cam_correspondences, opengv_points, cam_offsets, cam_rotations, se3_initial.translation(), se3_initial.rotation().toRotationMatrix()};
    auto problem = std::make_shared<SacProblem>(adapter, SacProblem::Algorithm::GP3P);

    // Ransac.
    Ransac ransac;
    ransac.threshold_ = ransac_threshold;
    ransac.max_iterations_ = kNumRansacIterations;
    ransac.probability_ = kRansacProbability;
    ransac.sac_model_ = std::move(problem);
    ransac.computeModel(0);

    // Nonlinear refinement.
    opengv::transformation_t optimized_pose;
    ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, optimized_pose);

    // Remove outliers per camera.
    filterByIndex(linear_indexes, ransac.inliers_);
    std::vector<std::vector<cv::DMatch>> camera_inliers;
    camera_inliers.resize(num_cameras);
    for (const auto& [cam_idx, match] : linear_indexes) {
        camera_inliers[cam_idx].push_back(match);
    }
    camera_matches = std::move(camera_inliers);

    // Return found transformation.
    return SE3{SU2{Eigen::Quaternion<Scalar>{optimized_pose.leftCols<3>()}}, optimized_pose.rightCols<1>()};
}
/*
auto ransac3d3d(const std::vector<map::MapPoint*>& map_points_A,
    const std::vector<map::MapPoint*>& map_points_B,
    std::vector<cv::DMatch>& matches,
    const Scalar ransac_threshold) -> SE3 {
    using Adapter = opengv::point_cloud::PointCloudAdapter;
    using SacProblem = opengv::sac_problems::point_cloud::PointCloudSacProblem;
    using Ransac = opengv::sac::Ransac<SacProblem>;

    // Build OpenGV containers.
    opengv::points_t points_A, points_B;
    points_A.reserve(matches.size());
    points_B.reserve(matches.size());
    for (auto& match : matches) {
        points_A.push_back(map_points_A[match.queryIdx]->getPosition());
        points_B.push_back(map_points_B[match.trainIdx]->getPosition());
    }
    auto adapter = Adapter{points_A, points_B};
    auto problem = std::make_shared<SacProblem>(adapter);

    Ransac ransac;
    ransac.sac_model_ = std::move(problem);
    ransac.threshold_ = ransac_threshold;
    ransac.max_iterations_ = kNumRansacIterations;
    ransac.probability_ = kRansacProbability;
    ransac.computeModel(0);

    // Nonlinear refinement.
    opengv::transformation_t optimized_pose;
    ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, optimized_pose);

    // Remove outlier matches.
    filterByIndex(matches, ransac.inliers_);

    // Return found transformation.
    return SE3{SU2{Eigen::Quaternion<Scalar>{optimized_pose.leftCols<3>()}}, optimized_pose.rightCols<1>()};
}

auto ransac2d2d(const std::vector<Bearing>& bearings_A,
    const std::vector<Bearing>& bearings_B,
    std::vector<cv::DMatch>& matches,
    Scalar ransac_threshold) -> SE3 {
    using Adapter = opengv::relative_pose::CentralRelativeAdapter;
    using SacProblem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
    using Ransac = opengv::sac::Ransac<SacProblem>;

    // Build OpenGV containers.
    opengv::bearingVectors_t opengv_bearings_A, opengv_bearings_B;
    for (const auto& match : matches) {
        CHECK(match.queryIdx < bearings_A.size());
        CHECK(match.trainIdx < bearings_B.size());
        opengv_bearings_A.push_back(bearings_A[match.queryIdx]);
        opengv_bearings_B.push_back(bearings_B[match.trainIdx]);
    }

    auto adapter = Adapter{opengv_bearings_A, opengv_bearings_B};
    auto problem = std::make_shared<SacProblem>(adapter, SacProblem::Algorithm::SEVENPT);

    Ransac ransac{kNumRansacIterations, ransac_threshold, kRansacProbability};
    ransac.sac_model_ = std::move(problem);
    ransac.computeModel(0);

    // Remove outlier matches.
    filterByIndex(matches, ransac.inliers_);

    // Return found transformation.
    return SE3{SU2{Eigen::Quaternion<Scalar>{ransac.model_coefficients_.leftCols<3>()}}, ransac.model_coefficients_.rightCols<1>()};
}
*/
} // namespace deco
