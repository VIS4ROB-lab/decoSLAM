//
// Created by philipp on 11.10.21.
//

#pragma once

#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include "global.hpp"
#include "map/frame.hpp"
#include "map/map_point.hpp"
#include "optimization/bearing_cost.hpp"
#include "optimization/euclidean_consensus_cost.hpp"
#include "optimization/pixel_cost.hpp"
#include "optimization/se3_consensus_cost.hpp"

namespace deco {

class BundleAdjustmentProblem {
  public:
    using ReprojectionCost = PixelCost;
    using MapPointConsensusCost = EuclideanConsensusCost<hyper::Traits<Position>::kNumParameters>;
    using FramePoseConsensusCost = SE3ConsensusCost;
    using ReprojectionResiduals = std::map<ObservationKey, std::pair<ceres::ResidualBlockId, std::unique_ptr<ceres::CostFunction>>>;
    using MapPointConsensusResiduals = std::map<StateId, std::pair<ceres::ResidualBlockId, std::unique_ptr<MapPointConsensusCost>>>;
    using FramePoseConsensusResiduals = std::map<StateId, std::pair<ceres::ResidualBlockId, std::unique_ptr<FramePoseConsensusCost>>>;

    BundleAdjustmentProblem(const Scalar& pixel_noise, const Scalar& huber_loss_threshold, const Scalar& admm_gamma);
    ~BundleAdjustmentProblem() = default;

    /// Add the pose and calibrations of a frame to the optimization problem.
    /// Note: This sets all intrinsics and extrinsics constant by default.
    /// \param frame_pose Reference to the pose of the frame.
    /// \param calibrations Reference to the calibrations of the frame.
    auto addFrame(SE3& frame_pose, std::vector<Frame::Calibration>& calibrations) -> void;

    /// Set the pose of a frame constant in the optimization.
    /// \param frame_pose Reference to frame pose to set constant.
    auto setFramePoseConstant(SE3& frame_pose) -> void;

    /// Set the pose of a frame variable in the optimization.
    /// \param frame_pose Reference to frame pose to set variable.
    auto setFramePoseVariable(SE3& frame_pose) -> void;

    /// Check if the state of a frame is present in the optimization.
    /// \param frame_pose Reference to frame pose to check.
    /// \return True if the state is in the optimization.
    auto hasFrame(SE3& frame_pose) -> bool;

    /// Remove the parameters of a frame from the optimization.
    /// \param frame_pose Reference to frame pose to be removed.
    /// \param calibrations Reference to the calibrations of the frame to be removed.
    auto removeFrame(SE3& frame_pose, std::vector<Frame::Calibration>& calibrations) -> void;

    /// Add the position of a map point to the optimization.
    /// \param map_point_position Reference to map point position to be added.
    auto addMapPoint(Position& map_point_position) -> void;

    /// Set the position of a map point constant in the optimization.
    /// \param map_point_position Reference to map point position to set constant.
    auto setMapPointPositionConstant(Position& map_point_position) -> void;

    /// Set the position of a map point variable in the optimization.
    /// \param map_point_position Reference to map point position to set variable.
    auto setMapPointPositionVariable(Position& map_point_position) -> void;

    /// Check if a given map point position is present in the optimization.
    /// \param map_point_position Map point state to check.
    /// \return True if the map point position is in the optimization.
    auto hasMapPoint(Position& map_point_position) -> bool;

    /// Remove a map point position from the optimization.
    /// \param map_point_position Reference to map point position to be removed.
    auto removeMapPoint(Position& map_point_position) -> void;

    /// Add a reprojection residual for a given map point and camera in a frame to the problem.
    /// \param key Observation key.
    /// \param map_point_position Reference to map point position.
    /// \param frame_pose Reference to frame pose.
    /// \param calibrations Reference to calibrations.
    /// \param pixel Reference to pixel measurement.
    auto createReprojectionResidual(const ObservationKey& key, Position& map_point_position, SE3& frame_pose,
        std::vector<Frame::Calibration>& calibrations, const Pixel& pixel) -> void;

    /// Evaluates the reprojection residual for a given map point observation and camera in a frame.
    /// \param key Observation key.
    /// \param map_point_position Reference to map point position.
    /// \param frame_pose Reference to frame pose.
    /// \param calibrations Reference to calibrations.
    /// \return Difference between estimated pixel and measured pixel.
    auto evaluateReprojectionResidual(const ObservationKey& key, Position& map_point_position, SE3& frame_pose,
        std::vector<Frame::Calibration>& calibrations) -> Pixel;

    /// Remove a reprojection residual for a given map point and camera in a frame from the problem.
    /// \param map_point_id Id of the map point.
    /// \param frame_id Id of the frame.
    /// \param cam_idx Index of the camera.
    auto removeReprojectionResidual(const ObservationKey& key) -> void;

    auto createFramePoseConsensusResidual(const StateId& frame_id, SE3& pose, const hyper::Tangent<SE3>& average_dual, const SE3& reference, int cardinality) -> void;

    /// Create a consensus residual for a map point.
    /// \param map_point Pointer to map point.
    auto createMapPointConsensusResidual(const StateId& map_point_id, Position& position, const Position& average_dual, int cardinality) -> void;

    /// Remove the consensus residual for a map point.
    /// \param map_point Pointer to map point.
    auto removeMapPointConsensusResidual(const StateId& map_point_id) -> void;

    /// Solve the underlying problem.
    /// \return Whether the problem converged.
    auto solve(int max_num_iterations) -> std::pair<bool, int>;

  private:
    Scalar pixel_noise_;
    Scalar huber_loss_threshold_;
    Scalar admm_gamma_;

    ceres::Problem problem_;                                   ///< Ceres problem object.
    ceres::Solver::Options solver_options_;                    ///< Ceres solver options.
    ReprojectionResiduals reprojection_residuals_;             ///< Map keeping track of reprojection residuals.
    MapPointConsensusResiduals map_point_consensus_residuals_; ///< Map keeping track of consensus residuals.
    FramePoseConsensusResiduals frame_pose_consensus_residuals_;
};
} // namespace deco
