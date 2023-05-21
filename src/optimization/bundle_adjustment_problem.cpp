//
// Created by philipp on 11.10.21.
//

#include <ceres/loss_function.h>
#include <glog/logging.h>

#include "map/frame.hpp"
#include "map/map.hpp"
#include "optimization/bundle_adjustment_problem.hpp"
#include "optimization/pose_parametrization.hpp"

namespace {
auto getCeresProblemOptions() -> ceres::Problem::Options {
    ceres::Problem::Options options;
    options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    options.enable_fast_removal = true;
    return options;
}
auto getCeresSolverOptions() -> ceres::Solver::Options {
    ceres::Solver::Options solver_options;
    solver_options.num_threads = 8;
    solver_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
    solver_options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    solver_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
    solver_options.parameter_tolerance = 1e-6;
    return solver_options;
}
} // namespace

namespace deco {

BundleAdjustmentProblem::BundleAdjustmentProblem(const Scalar& pixel_noise, const Scalar& huber_loss_threshold, const Scalar& admm_gamma)
    : pixel_noise_{pixel_noise},
      huber_loss_threshold_{huber_loss_threshold},
      admm_gamma_{admm_gamma},
      problem_{getCeresProblemOptions()},
      solver_options_{getCeresSolverOptions()},
      reprojection_residuals_{},
      map_point_consensus_residuals_{} {}

auto BundleAdjustmentProblem::addFrame(SE3& frame_pose, std::vector<Frame::Calibration>& calibrations) -> void {
    // Pose.
    problem_.AddParameterBlock(frame_pose.data(), hyper::Traits<SE3>::kNumParameters, new PoseParametrization{});

    // Calibration.
    for (auto& calibration : calibrations) {
        // Extrinsics.
        double* extrinsic_parameters = calibration.se3_body_camera_.data();
        problem_.AddParameterBlock(extrinsic_parameters, hyper::Traits<SE3>::kNumParameters, new PoseParametrization{});
        problem_.SetParameterBlockConstant(extrinsic_parameters);

        // Intrinsics.
        double* intrinsic_parameters = calibration.intrinsics_.data();
        problem_.AddParameterBlock(intrinsic_parameters, 4);
        problem_.SetParameterBlockConstant(intrinsic_parameters);

        // Distortion.
        double* distortion_parameters = calibration.distortions_.data();
        problem_.AddParameterBlock(distortion_parameters, 4);
        problem_.SetParameterBlockConstant(distortion_parameters);
    }
}

auto BundleAdjustmentProblem::setFramePoseConstant(SE3& frame_pose) -> void {
    problem_.SetParameterBlockConstant(frame_pose.data());
}

auto BundleAdjustmentProblem::setFramePoseVariable(SE3& frame_pose) -> void {
    problem_.SetParameterBlockVariable(frame_pose.data());
}

auto BundleAdjustmentProblem::hasFrame(SE3& frame_pose) -> bool {
    return problem_.HasParameterBlock(frame_pose.data());
}

auto BundleAdjustmentProblem::removeFrame(SE3& frame_pose, std::vector<Frame::Calibration>& calibrations) -> void {
    DCHECK(this->hasFrame(frame_pose)) << "Trying to remove a non-existing frame.";
    auto frame_residuals = std::vector<ceres::ResidualBlockId>();
    problem_.GetResidualBlocksForParameterBlock(frame_pose.data(), &frame_residuals);
    DCHECK(frame_residuals.empty()) << "Remove all frame residuals before removing the frame itself.";
    problem_.RemoveParameterBlock(frame_pose.data());
    for (auto calibration : calibrations) {
        problem_.RemoveParameterBlock(calibration.se3_body_camera_.data());
        problem_.RemoveParameterBlock(calibration.intrinsics_.data());
        problem_.RemoveParameterBlock(calibration.distortions_.data());
    }
}

auto BundleAdjustmentProblem::addMapPoint(Position& map_point_position) -> void {
    problem_.AddParameterBlock(map_point_position.data(), hyper::Traits<Position>::kNumParameters);
}

auto BundleAdjustmentProblem::setMapPointPositionConstant(Position& map_point_position) -> void {
    problem_.SetParameterBlockConstant(map_point_position.data());
}

auto BundleAdjustmentProblem::setMapPointPositionVariable(Position& map_point_position) -> void {
    problem_.SetParameterBlockVariable(map_point_position.data());
}

auto BundleAdjustmentProblem::hasMapPoint(Position& map_point_position) -> bool {
    return problem_.HasParameterBlock(map_point_position.data());
}

auto BundleAdjustmentProblem::removeMapPoint(Position& map_point_position) -> void {
    DCHECK(this->hasMapPoint(map_point_position)) << "Tried to remove non-existing map point.";
    auto map_point_residuals = std::vector<ceres::ResidualBlockId>();
    problem_.GetResidualBlocksForParameterBlock(map_point_position.data(), &map_point_residuals);
    DCHECK(map_point_residuals.empty()) << "Remove residuals before removing the map point." << map_point_residuals.size();
    problem_.RemoveParameterBlock(map_point_position.data());
}

auto BundleAdjustmentProblem::createReprojectionResidual(const ObservationKey& key, Position& map_point_position, SE3& frame_pose,
    std::vector<Frame::Calibration>& calibrations, const Pixel& pixel) -> void {
    DCHECK(this->hasMapPoint(map_point_position));
    DCHECK(this->hasFrame(frame_pose));

    auto loss_function = new ceres::HuberLoss{huber_loss_threshold_};
    auto cost = std::make_unique<ReprojectionCost>(pixel, pixel_noise_);
    auto residual_id = problem_.AddResidualBlock(cost.get(),
        loss_function,
        map_point_position.data(),
        frame_pose.data(),
        calibrations[key.camera_index].se3_body_camera_.data(),
        calibrations[key.camera_index].intrinsics_.data(),
        calibrations[key.camera_index].distortions_.data());
    auto [itr, inserted] = reprojection_residuals_.insert({key, std::make_pair(residual_id, std::move(cost))});
    DCHECK(inserted) << "Tried to add multiple reprojection residuals for the same observation.";
}

auto BundleAdjustmentProblem::evaluateReprojectionResidual(const ObservationKey& key, Position& map_point_position, SE3& frame_pose,
    std::vector<Frame::Calibration>& calibrations) -> Pixel {
    auto itr = reprojection_residuals_.find(key);
    DCHECK(itr != reprojection_residuals_.end()) << "Cannot evaluate residual. Does not exist.";
    DCHECK(this->hasFrame(frame_pose));
    DCHECK(this->hasMapPoint(map_point_position));

    // Evaluate.
    Pixel residual;
    std::vector<double*> parameters{
        map_point_position.data(),
        frame_pose.data(),
        calibrations[key.camera_index].se3_body_camera_.data(),
        calibrations[key.camera_index].intrinsics_.data(),
        calibrations[key.camera_index].distortions_.data()};
    itr->second.second->Evaluate(parameters.data(), residual.data(), nullptr);
    return residual;
}

auto BundleAdjustmentProblem::removeReprojectionResidual(const ObservationKey& key) -> void {
    auto itr = reprojection_residuals_.find(key);
    CHECK(itr != reprojection_residuals_.end()) << "Tried to remove non-existing reprojection residual.";
    problem_.RemoveResidualBlock(itr->second.first);
    reprojection_residuals_.erase(itr);
}

auto BundleAdjustmentProblem::createFramePoseConsensusResidual(const StateId& frame_id, SE3& pose, const hyper::Tangent<SE3>& average_dual, const SE3& reference, const int cardinality) -> void {
    DCHECK(problem_.HasParameterBlock(pose.data()));
    auto cost = std::make_unique<FramePoseConsensusCost>(admm_gamma_, reference);
    cost->setDualCorrection(average_dual, cardinality);
    auto residual_id = problem_.AddResidualBlock(cost.get(), nullptr, pose.data());
    auto [itr, inserted] = frame_pose_consensus_residuals_.insert({frame_id, std::make_pair(residual_id, std::move(cost))});
    DCHECK(inserted) << "Tried to add multiple consensus costs for the same frame pose.";
}

auto BundleAdjustmentProblem::createMapPointConsensusResidual(const StateId& map_point_id, Position& position, const Position& average_dual, const int cardinality) -> void {
    DCHECK(problem_.HasParameterBlock(position.data()));
    auto cost = std::make_unique<MapPointConsensusCost>(admm_gamma_);
    cost->setDualCorrection(average_dual, cardinality);
    auto residual_id = problem_.AddResidualBlock(cost.get(), nullptr, position.data());
    auto [itr, inserted] = map_point_consensus_residuals_.insert({map_point_id, std::make_pair(residual_id, std::move(cost))});
    DCHECK(inserted) << "Tried to add multiple consensus costs for the same map point.";
}

auto BundleAdjustmentProblem::removeMapPointConsensusResidual(const StateId& map_point_id) -> void {
    auto itr = map_point_consensus_residuals_.find(map_point_id);
    DCHECK(itr != map_point_consensus_residuals_.end()) << "Tried removing non-existent consensus cost.";
    problem_.RemoveResidualBlock(itr->second.first);
    map_point_consensus_residuals_.erase(itr);
}

auto BundleAdjustmentProblem::solve(const int max_num_iterations) -> std::pair<bool, int> {
    ceres::Solver::Summary summary;
    solver_options_.max_num_iterations = max_num_iterations;
    ceres::Solve(solver_options_, &problem_, &summary);
    // DLOG(INFO) << summary.BriefReport();
    return {(summary.termination_type == ceres::TerminationType::CONVERGENCE), summary.iterations.size()};
}

} // namespace deco
