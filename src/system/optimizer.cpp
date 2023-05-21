//
// Created by philipp on 08.11.22.
//
/*
#include <fstream>

#include <glog/logging.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "optimization/bundle_adjustment_problem.hpp"
#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/alignment.hpp"
#include "system/communicator.hpp"
#include "system/optimizer.hpp"
#include "utils/timer.hpp"

namespace deco {

Optimizer::Optimizer(StorageMutexHandler& mutex_handler)
    : mutex_handler_{mutex_handler},
      notified_{false} {}

Optimizer::~Optimizer() = default;

auto Optimizer::readParameters(const YAML::Node& node) -> void {
    pixel_noise_ = node["pixel_noise"].as<Scalar>();
    huber_loss_threshold_ = node["huber_loss_threshold"].as<Scalar>();
    outlier_threshold_ = node["outlier_threshold"].as<Scalar>();
    admm_gamma_ = node["admm_gamma"].as<Scalar>();
    max_num_iterations_ = node["max_num_iterations"].as<int>();
}

auto Optimizer::setSharedMap(SharedMap* shared_map) -> void {
    shared_map_ = shared_map;
}

auto Optimizer::setLocalStorage(LocalStorage* local_storage) -> void {
    local_storage_ = local_storage;
}

auto Optimizer::setCommunicator(Communicator* communicator) -> void {
    communicator_ = communicator;
}

auto Optimizer::setAlignment(Alignment* alignment) -> void {
    alignment_ = alignment;
}

auto Optimizer::start() -> void {
    optimization_thread_ = std::thread{&Optimizer::spin_, this};
}

auto Optimizer::stop() -> void {
    condition_.notify_one();
    optimization_thread_.join();
}

auto Optimizer::notify() -> void {
    auto lock = std::unique_lock<std::mutex>{optimization_thread_mutex_};
    notified_ = true;
    lock.unlock();
    condition_.notify_one();
}

auto Optimizer::setDriftReference(const StateId& drift_reference_id) -> void {
    auto lock = std::unique_lock<std::mutex>{optimization_thread_mutex_};
    drift_reference_id_ = drift_reference_id;
    lock.unlock();
}

auto Optimizer::writeLogs(const FilePath& output_path) -> void {
    const auto separator = ", ";
    const auto file_path = output_path / "map_size_log.csv";
    std::ofstream out_file;
    out_file.open(file_path);
    CHECK(out_file.is_open()) << "Could not open file at " << file_path;

    // Header.
    out_file << "timestamp" << separator
             << "num_keyframes" << separator
             << "num_map_points"
             << "\n";

    for (const auto& [timestamp, data] : map_size_log_) {
        const auto& [num_keyframes, num_map_points] = data;
        out_file << timestamp << separator
                 << num_keyframes << separator
                 << num_map_points << "\n";
    }
    out_file.close();
}

auto Optimizer::spin_() -> void {
    while (!ros::isShuttingDown()) {
        auto lock = std::unique_lock<std::mutex>(optimization_thread_mutex_);
        condition_.wait(lock, [this]() -> bool { return notified_ || ros::isShuttingDown(); });
        if (ros::isShuttingDown()) {
            break;
        }
        const auto drift_reference_id = drift_reference_id_;
        notified_ = false;
        lock.unlock();

        // Build problem.
        SE3 drift_reference_pose_before;
        auto observations = std::set<ObservationKey>();
        auto measurements = std::vector<Pixel>();
        Id world_frame_id_before;
        auto copy_timer = Timer("Copy Problem");
        copy_timer.tic();
        {
            auto storage_lock = mutex_handler_.lockLocalStorage();

            // Get world frame id and current reference frame pose.
            world_frame_id_before = local_storage_->getWorldFrameId();
            drift_reference_pose_before = local_storage_->getKeyframePose(drift_reference_id);

            // Copy all available frame poses and duals.
            for (const auto& id : local_storage_->getMap()->getFrameIds()) {
                if (local_storage_->hasKeyframePose(id)) {
                    keyframe_poses_.insert({id, local_storage_->getKeyframePose(id)});
                    keyframe_calibrations_.insert({id, local_storage_->getKeyframeCalibration(id)});
                }

                if (local_storage_->hasKeyframeDual(id)) {
                    keyframe_duals_.insert({id, local_storage_->getKeyframeDual(id)});
                }
            }

            // Copy all available map point positions and duals.
            for (const auto& id : local_storage_->getMap()->getMapPointIds()) {
                if (local_storage_->hasMapPointPosition(id)) {
                    map_point_positions_.insert({id, local_storage_->getMapPointPosition(id)});
                }
                if (local_storage_->hasMapPointDual(id)) {
                    map_point_duals_.insert({id, local_storage_->getMapPointDual(id)});
                }
            }

            // Copy measurements & observations that have a state associated to them.
            for (const auto& key : local_storage_->getMap()->getObservations()) {
                if (local_storage_->hasKeyframePose(key.frame_id) && local_storage_->hasMapPointPosition(key.map_point_id)) {
                    const auto& keypoint = (*local_storage_->getKeyframeImageData(key.frame_id))[key.camera_index].keypoints[key.keypoint_index];
                    measurements.emplace_back(keypoint.pt.x, keypoint.pt.y);
                    observations.insert(key);
                }
            }

            local_storage_->clearNewIds();

            // Log size of collected map.
            map_size_log_.insert({ros::Time::now().toNSec(), {keyframe_poses_.size(), map_point_positions_.size()}});
        }
        copy_timer.toc();
        // copy_timer.display();

        auto i = 0;
        auto global_problem = std::make_unique<BundleAdjustmentProblem>(pixel_noise_, huber_loss_threshold_, admm_gamma_);
        auto build_timer = Timer("Build Problem");
        build_timer.tic();
        for (const auto& key : observations) {
            auto& frame_pose = keyframe_poses_.find(key.frame_id)->second;
            auto& frame_calibration = keyframe_calibrations_.find(key.frame_id)->second;
            auto& map_point = map_point_positions_.find(key.map_point_id)->second;

            // Add frame if necessary.
            if (!global_problem->hasFrame(frame_pose)) {
                global_problem->addFrame(frame_pose, frame_calibration);

                // Detect if we own the world frame.
                if (key.frame_id == StateId{world_frame_id_before, 0}) {
                    global_problem->setFramePoseConstant(frame_pose);
                }

                if (keyframe_duals_.contains(key.frame_id)) {
                    const auto& dual = keyframe_duals_.find(key.frame_id)->second;
                    const auto& [average_dual, reference, cardinality] = dual;
                    global_problem->createFramePoseConsensusResidual(key.frame_id, frame_pose, average_dual, reference, cardinality);
                }
            }

            // Add map point if necessary.
            if (!global_problem->hasMapPoint(map_point)) {
                global_problem->addMapPoint(map_point);

                if (map_point_duals_.contains(key.map_point_id)) {
                    const auto& dual = map_point_duals_.find(key.map_point_id)->second;
                    const auto& [average_dual, cardinality] = dual;
                    global_problem->createMapPointConsensusResidual(key.map_point_id, map_point, average_dual, cardinality);
                }
            }

            // Add residual.
            global_problem->createReprojectionResidual(key, map_point, frame_pose, frame_calibration, measurements[i]);
            ++i;
        }
        build_timer.toc();
        // build_timer.display();

        // Call solver.
        auto solve_timer = Timer("Solve Problem");
        solve_timer.tic();
        auto is_converged = global_problem->solve(max_num_iterations_);

        // Identify outliers.
        std::set<ObservationKey> outliers;
        for (const auto& key : observations) {
            auto& frame_pose = keyframe_poses_.find(key.frame_id)->second;
            auto& frame_calibration = keyframe_calibrations_.find(key.frame_id)->second;
            auto& map_point = map_point_positions_.find(key.map_point_id)->second;
            auto error = global_problem->evaluateReprojectionResidual(key, map_point, frame_pose, frame_calibration);
            if (error.norm() > outlier_threshold_) {
                global_problem->removeReprojectionResidual(key);
                outliers.insert(key);
            }
        }

        if (!outliers.empty()) {
            is_converged = global_problem->solve(max_num_iterations_);
            DLOG(INFO) << "Identified " << outliers.size() << " outlier observations.";
        }

        solve_timer.toc();
        // solve_timer.display();

        auto update_timer = Timer("Update storage");
        update_timer.tic();

        auto keyframe_update_messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::KEYFRAME_UPDATES>>>();
        auto map_point_update_messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_UPDATES>>>();
        {
            auto storage_lock = mutex_handler_.lockLocalStorage();

            // Detect change of world frame.
            auto world_frame_id_after = local_storage_->getWorldFrameId();
            auto frame_transformation = SE3::Identity();
            if (world_frame_id_before != world_frame_id_after) {
                DLOG(INFO) << "Frame transformation " << world_frame_id_before << " --> " << world_frame_id_after << " in optimizer.";
                frame_transformation = alignment_->getTransformationToWorld(world_frame_id_before);
            }

            // Calculate drift.
            const auto& drift_reference_pose_after = keyframe_poses_.find(drift_reference_id)->second;
            const Translation position_drift = drift_reference_pose_after.translation() - drift_reference_pose_before.translation();
            const SU2 orientation_drift = drift_reference_pose_after.rotation().groupInverse().groupPlus(drift_reference_pose_after.rotation());

            // Propagate keyframe poses.
            for (auto& [id, pose] : keyframe_poses_) {
                // Apply transform
                pose = frame_transformation.groupPlus(pose);

                if (local_storage_->hasKeyframePose(id)) {
                    local_storage_->updateKeyframePose(id, pose);
                }

                if (keyframe_duals_.contains(id)) {
                    auto itr = keyframe_update_messages.find(id.first);
                    if (itr == keyframe_update_messages.end()) {
                        itr = keyframe_update_messages.insert({id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::KEYFRAME_UPDATES>>()}).first;
                    }
                    itr->second->keyframe_updates.emplace_back(id, pose);
                }
            }
            keyframe_poses_.clear();
            keyframe_calibrations_.clear();
            keyframe_duals_.clear();

            // Propagate map point positions.
            for (auto& [id, position] : map_point_positions_) {
                // Apply transform.
                position = frame_transformation.vectorPlus(position);

                if (local_storage_->hasMapPointPosition(id)) {
                    local_storage_->updateMapPointPosition(id, position);
                }

                if (map_point_duals_.contains(id)) {
                    auto itr = map_point_update_messages.find(id.first);
                    if (itr == map_point_update_messages.end()) {
                        itr = map_point_update_messages.insert({id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_UPDATES>>()}).first;
                    }
                    itr->second->map_point_updates.emplace_back(id, position);
                }
            }
            map_point_positions_.clear();
            map_point_duals_.clear();

            SE3 drift;
            drift.translation() = position_drift;
            drift.rotation() = orientation_drift;
            local_storage_->applyDriftCorrection(drift);
        }

        update_timer.toc();
        // update_timer.display();

        for (auto& [id, update] : keyframe_update_messages) {
            update->receiver_id = id;
            communicator_->send(std::move(update));
        }

        for (auto& [id, update] : map_point_update_messages) {
            update->receiver_id = id;
            communicator_->send(std::move(update));
        }

        {
            auto storage_lock = mutex_handler_.lockAllStorages();
            shared_map_->observationStorage(outliers, StorageAction::REMOVE);
        }

        local_storage_->requestKeyframeRemovals();
        local_storage_->triggerVisualization();

        lock.lock();
        notified_ = notified_ || !is_converged;
        lock.unlock();
    }
}

} // namespace deco
*/