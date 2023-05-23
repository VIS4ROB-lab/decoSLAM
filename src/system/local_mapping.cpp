//
// Created by philipp on 04.10.22.
//

#include <glog/logging.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "map/map_point.hpp"
#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/local_mapping.hpp"
#include "system/local_tracking.hpp"
#include "system/optimizer.hpp"
#include "system/place_recognition.hpp"
#include "utils/common.hpp"
#include "utils/matching.hpp"
#include "utils/ransac.hpp"
#include "utils/timer.hpp"
#include "utils/triangulation.hpp"

namespace deco {

namespace {
auto triangulateNewPoints(Frame* frame, const std::set<ObservationKey>& observations, const Scalar epipolar_threshold,
    const Scalar matching_threshold, const std::pair<Scalar, Scalar>& range, const size_t max_num_new_points) -> std::vector<std::pair<Position, cv::DMatch>> {
    const auto& left_image_data = frame->image_data[0];
    const auto& right_image_data = frame->image_data[1];

    // Mask all keypoints that are already associated.
    cv::Mat mask = cv::Mat::ones(static_cast<int>(left_image_data.bearings.size()), static_cast<int>(right_image_data.bearings.size()), CV_8U);
    for (const auto& key : observations) {
        if (key.camera_index == 0) {
            for (auto j = 0; j < mask.cols; ++j) {
                mask.at<uchar>(key.keypoint_index, j) = 0;
            }
        } else if (key.camera_index == 1) {
            for (auto j = 0; j < mask.rows; ++j) {
                mask.at<uchar>(j, key.keypoint_index) = 0;
            }
        }
    }

    // Match.
    auto matches = descriptorMatching(left_image_data.keypoint_descriptors, right_image_data.keypoint_descriptors, 2 * matching_threshold, &mask);

    // Filter by epipolar constraint.
    filterEpipolar2D2D(left_image_data.bearings, right_image_data.bearings, matches, frame->essential_matrix, epipolar_threshold);

    // Triangulate.
    const SE3 se3_left_right = frame->calibration[0].se3_body_camera_.groupInverse().groupPlus(frame->calibration[1].se3_body_camera_);
    auto triangulated_matches = stereoTriangulate(left_image_data.bearings, right_image_data.bearings, matches, se3_left_right);
    // DLOG(INFO) << "Triangulated " << triangulated_matches.size() << " points.";

    // Prune by matching score.
    if (triangulated_matches.size() > max_num_new_points) {
        int debug_num_pruned = 0;
        std::sort(triangulated_matches.begin(), triangulated_matches.end(), [](const auto& arg1, const auto& arg2) -> bool { return arg1.second.distance > arg2.second.distance; });
        auto itr = triangulated_matches.begin();
        while (itr->second.distance > matching_threshold && triangulated_matches.size() > max_num_new_points) {
            itr = triangulated_matches.erase(itr);
            ++debug_num_pruned;
        }
        // DLOG(INFO) << "Pruned " << debug_num_pruned << " by score";
    }

    // Prune far away points.
    if (triangulated_matches.size() > max_num_new_points) {
        int debug_num_pruned = 0;
        std::sort(triangulated_matches.begin(), triangulated_matches.end(), [](const auto& arg1, const auto& arg2) -> bool { return arg1.first.z() > arg2.first.z(); });
        const auto baseline = se3_left_right.translation().norm();
        auto itr = triangulated_matches.begin();
        while (itr->first.z() > range.second * baseline && triangulated_matches.size() > max_num_new_points) {
            itr = triangulated_matches.erase(itr);
            ++debug_num_pruned;
        }
        // DLOG(INFO) << "Pruned " << debug_num_pruned << " far away points.";
    }
    // Prune close points.
    if (triangulated_matches.size() > max_num_new_points) {
        int debug_num_pruned = 0;
        std::reverse(triangulated_matches.begin(), triangulated_matches.end());
        const auto baseline = se3_left_right.translation().norm();
        auto itr = triangulated_matches.begin();
        while (itr->first.z() < range.first * baseline && triangulated_matches.size() > max_num_new_points) {
            itr = triangulated_matches.erase(itr);
            ++debug_num_pruned;
        }
        // DLOG(INFO) << "Pruned " << debug_num_pruned << " close-by points.";
    }

    if (triangulated_matches.size() > max_num_new_points) {
        std::sort(triangulated_matches.begin(), triangulated_matches.end(), [](const auto& arg1, const auto& arg2) -> bool { return arg1.second.distance < arg2.second.distance; });
        triangulated_matches.erase(triangulated_matches.begin() + static_cast<long>(max_num_new_points), triangulated_matches.end());
    }

    const SE3 se3_world_left = frame->se3_world_body.groupPlus(frame->calibration[0].se3_body_camera_);
    for (auto& [point, match] : triangulated_matches) {
        point = se3_world_left.vectorPlus(point);
    }

    return triangulated_matches;
}
} // namespace

LocalMapping::LocalMapping(StorageMutexHandler& mutex_handler)
    : mutex_handler_{mutex_handler},
      map_point_count_{0},
      keyframe_queue_{"LocalMapping::KeyframeQueue", std::bind(&LocalMapping::process_, this, std::placeholders::_1), ros::isShuttingDown} {}

LocalMapping::~LocalMapping() = default;

auto LocalMapping::readParameters(const YAML::Node& node) -> void {
    triangulation_range_ = node["triangulation_range"].as<std::pair<Scalar, Scalar>>();
    max_num_points_ = node["max_num_points"].as<size_t>();
}

auto LocalMapping::setMatchingParameters(const MatchingParameters& parameters) -> void {
    parameters_ = parameters;
}

auto LocalMapping::setSharedMap(SharedMap* shared_map) -> void {
    shared_map_ = shared_map;
}

auto LocalMapping::setLocalStorage(LocalStorage* local_storage) -> void {
    local_storage_ = local_storage;
}

auto LocalMapping::setLocalTracking(LocalTracking* local_tracking) -> void {
    local_tracking_ = local_tracking;
}

auto LocalMapping::setPlaceRecognition(PlaceRecognition* place_recognition) -> void {
    place_recognition_ = place_recognition;
}

auto LocalMapping::start() -> void {
    keyframe_queue_.start();
}

auto LocalMapping::stop() -> void {
    keyframe_queue_.stop();
}

auto LocalMapping::submit(Input&& data) -> void {
    keyframe_queue_.submit(std::move(data));
}

auto LocalMapping::process_(Input&& data) -> void {
    auto& [keyframe, tracking_observations, neighbors] = data;

    auto timer = Timer("Local Mapping");
    timer.tic();
    DLOG(INFO) << "Local Mapping: Processing " << keyframe->id;

    // Triangulate new map points from new keyframe.
    const auto triangulated_matches = triangulateNewPoints(keyframe.get(), tracking_observations,
        parameters_.epipolar_threshold, parameters_.matching_threshold, triangulation_range_, max_num_points_);
    // DLOG(INFO) << "Triangulated " << triangulated_matches.size() << " new points.";
    auto triangulated_observations = std::set<ObservationKey>();
    auto new_map_points = std::vector<std::unique_ptr<MapPoint>>();
    auto new_descriptors = cv::Mat();
    auto new_positions = std::vector<Position>();
    auto new_map_point_ids = std::set<StateId>();
    new_map_points.reserve(triangulated_matches.size());
    new_positions.reserve(triangulated_matches.size());
    for (const auto& [position, match] : triangulated_matches) {
        // Map point.
        StateId map_point_id{keyframe->id.first, map_point_count_};
        auto map_point = std::make_unique<MapPoint>();
        map_point->id = map_point_id;
        map_point->position = position;
        map_point->reference_frame_id = keyframe->reference_frame_id;
        map_point->addDescriptor(keyframe->image_data[0].keypoint_descriptors.row(match.queryIdx));
        map_point->addDescriptor(keyframe->image_data[1].keypoint_descriptors.row(match.trainIdx));
        new_descriptors.push_back(map_point->getDescriptor());
        new_map_point_ids.insert(map_point_id);
        new_positions.push_back(position);
        new_map_points.emplace_back(std::move(map_point));

        // Observations.
        triangulated_observations.insert(ObservationKey{.frame_id = keyframe->id, .map_point_id = map_point_id, .camera_index = 0, .keypoint_index = match.queryIdx});
        triangulated_observations.insert(ObservationKey{.frame_id = keyframe->id, .map_point_id = map_point_id, .camera_index = 1, .keypoint_index = match.trainIdx});

        ++map_point_count_;
    }

    // DLOG(INFO) << "Triangulated ids " << debug_count_before << " to " << debug_count_after;

    const auto frame_id = keyframe->id;

    {
        // Synchronize storage of local mapper results.
        auto lock = mutex_handler_.lockAllStorages();
        shared_map_->storeKeyframe(std::move(keyframe));
        shared_map_->storeMapPoints(new_map_points);
        shared_map_->observationStorage(tracking_observations, StorageAction::ADD);
        shared_map_->observationStorage(triangulated_observations, StorageAction::ADD);
        local_storage_->registerKeyframeForDriftCorrection(frame_id);
        local_storage_->registerMapPointsForDriftCorrection(new_map_point_ids);
        local_storage_->addMapPointsToCullingQueue(new_map_point_ids);
        auto reprojected_observations = reprojectIntoLocalFrames_(neighbors, new_positions, new_descriptors, new_map_point_ids);
        shared_map_->observationStorage(reprojected_observations, StorageAction::ADD);
        new_map_points.clear();
    }

    // Reset the tracking reference.
    local_storage_->setTrackingReference(frame_id);

    // Submit to the PR module.
    place_recognition_->submit(frame_id);

    // Set optimization flag.
    local_storage_->notifyOptimization();
    timer.toc();
    // timer.display();
}

auto LocalMapping::reprojectIntoLocalFrames_(const std::set<StateId>& local_frame_ids,
    const std::vector<Translation>& points,
    const cv::Mat& new_point_descriptors,
    const std::set<StateId>& new_ids)
    -> std::set<ObservationKey> {
    std::set<ObservationKey> observations;
    // Project into neighboring keyframes.
    for (const auto& neighbor_id : local_frame_ids) {
        if (local_storage_->hasKeyframePose(neighbor_id)) {
            const auto pose = local_storage_->getKeyframePose(neighbor_id);
            const auto calibrations = local_storage_->getKeyframeCalibration(neighbor_id);
            const auto image_data = *(local_storage_->getKeyframeImageData(neighbor_id));
            const auto map = local_storage_->getMap();

            // Match image-wise.
            std::vector<std::vector<cv::DMatch>> camera_matches;
            for (auto cam_idx = 0u; cam_idx < image_data.size(); ++cam_idx) {
                const SE3 se3_camera_world = (pose.groupPlus(calibrations[cam_idx].se3_body_camera_)).groupInverse();
                auto mask = projectionMask(points, image_data[cam_idx].bearings, se3_camera_world, parameters_.projection_threshold);
                // Mask all keypoints that are already matched to a map point.
                for (const auto& key : map->getObservationsForCamera(neighbor_id, cam_idx)) {
                    for (auto j = 0; j < mask.rows; ++j) {
                        CHECK(key.keypoint_index < mask.cols);
                        mask.at<uchar>(j, key.keypoint_index) = 0;
                    }
                }
                camera_matches.push_back(descriptorMatching(new_point_descriptors, image_data[cam_idx].keypoint_descriptors, parameters_.matching_threshold, &mask));
            }

            // Filter out matches using epipolar constraint.
            auto se3_body_camera = std::vector<SE3>{calibrations[0].se3_body_camera_, calibrations[1].se3_body_camera_};
            const auto essential_matrix = computeEssentialMatrix(se3_body_camera);
            filterEpipolar3D2D(image_data[0].bearings, image_data[1].bearings, camera_matches[0], camera_matches[1], essential_matrix, parameters_.epipolar_threshold);

            if (camera_matches[0].size() + camera_matches[1].size() > 10) {
                // Apply RANSAC.
                ransac3d2d(points, {image_data[0].bearings, image_data[1].bearings}, camera_matches, se3_body_camera, pose, parameters_.ransac_threshold);

                // Generate new observations.
                for (auto i = 0u; i < image_data.size(); ++i) {
                    for (const auto& match : camera_matches[i]) {
                        const auto new_map_point_id = *std::next(new_ids.begin(), match.queryIdx);
                        observations.insert(ObservationKey{.frame_id = neighbor_id,
                            .map_point_id = new_map_point_id,
                            .camera_index = i,
                            .keypoint_index = match.trainIdx});
                    }
                }
            }
        } else {
            DLOG(INFO) << "Discarded neighbor " << neighbor_id;
        }
    }
    return observations;
}

} // namespace deco
