//
// Created by philipp on 03.10.22.
//

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include "storage/local_storage.hpp"
#include "system/alignment.hpp"
#include "system/local_mapping.hpp"
#include "system/local_tracking.hpp"
#include "utils/common.hpp"
#include "utils/detection.hpp"
#include "utils/matching.hpp"
#include "utils/ransac.hpp"
#include "utils/timer.hpp"
#include "visualization/helpers.hpp"
#include "visualization/visualizer.hpp"

namespace deco {

namespace {
auto computeAverageDepth(const std::vector<Position>& map_point_positions, const SE3& se3_world_camera) -> Scalar {
    Scalar average_depth = 0;
    for (const auto& position : map_point_positions) {
        const Position camera_position = se3_world_camera.groupInverse().vectorPlus(position);
        average_depth += camera_position.z();
    }
    average_depth /= static_cast<Scalar>(map_point_positions.size());
    return average_depth;
}
} // namespace

LocalTracking::LocalTracking(const Id& id, StorageMutexHandler& mutex_handler)
    : id_{id},
      se3_world_body_current_{SE3::Identity()},
      world_frame_id_{id},
      detector_{cv::BRISK::create()},
      map_{nullptr},
      keyframe_count_{0},
      is_map_reset_{false},
      mutex_handler_{mutex_handler} {}

auto LocalTracking::readParameters(const YAML::Node& node) -> void {
    detector_->setThreshold(node["agast_threshold"].as<int>());
    detector_->setOctaves(0);
    min_num_points_tracked_ = node["min_num_points_tracked"].as<size_t>();
    baseline_depth_ratio_ = node["baseline_depth_ratio"].as<Scalar>();
}

auto LocalTracking::setMatchingParameters(const MatchingParameters& parameters) -> void {
    parameters_ = parameters;
}

auto LocalTracking::setCameras(const std::vector<hyper::Camera*>& cameras) -> void {
    DCHECK(cameras.size() == 2) << "Only stereo is currently supported.";
    cameras_ = cameras;
    se3_left_right_ = cameras_[0]->transformation().groupInverse().groupPlus(cameras_[1]->transformation());
    essential_matrix_left_right_ = computeEssentialMatrix({cameras_[0]->transformation(), cameras_[1]->transformation()});
}

auto LocalTracking::setLocalStorage(LocalStorage* local_storage) -> void {
    local_storage_ = local_storage;
}

auto LocalTracking::setLocalMapping(LocalMapping* local_mapping) -> void {
    local_mapping_ = local_mapping;
}

auto LocalTracking::setAlignment(Alignment* alignment) -> void {
    alignment_ = alignment;
}

auto LocalTracking::setVisualizer(Visualizer* visualizer) -> void {
    visualizer_ = visualizer;
}

auto LocalTracking::mutex() -> std::mutex& {
    return map_mutex_;
}

auto LocalTracking::extendTrackingMap(const StateId& frame_id, const std::vector<std::tuple<StateId, Translation, cv::Mat>>& new_map_points) -> void {
    auto itr = map_->local_slam_graph.find(frame_id);
    if (itr == map_->local_slam_graph.end()) {
        itr = map_->local_slam_graph.insert({frame_id, {}}).first;
    }
    for (const auto& [id, position, descriptor] : new_map_points) {
        itr->second.insert(id);
        map_->map_points.insert({id, {position, descriptor}});
    }
    DLOG(INFO) << "Extend tracking map.";
}

auto LocalTracking::setTrackingMap(std::unique_ptr<TrackingMap>&& map) -> void {
    map_ = std::move(map);
    {
        auto lock = mutex_handler_.lockAlignment();
        const auto current_world_frame_id = alignment_->getRootId();
        if (map_->world_frame_id != current_world_frame_id) {
            const auto map_transform = alignment_->getTransformationToWorld(map_->world_frame_id);
            for (auto& [id, data] : map_->map_points) {
                auto& [position, _] = data;
                position = map_transform.vectorPlus(position);
            }
            map_->reference_keyframe_pose = map_transform.groupPlus(map_->reference_keyframe_pose);
            map_->world_frame_id = current_world_frame_id;
        }

        if (world_frame_id_ != current_world_frame_id) {
            const auto transform = alignment_->getTransformationToWorld(world_frame_id_);
            se3_world_body_current_ = transform.groupPlus(se3_world_body_current_);
            world_frame_id_ = current_world_frame_id;
        }
    }

    is_map_reset_ = true;
    // DLOG(INFO) << "Reset tracking map with reference " << map_->reference_keyframe_id;
}

auto LocalTracking::applyDriftCorrection(const SE3& drift) -> void {
    se3_world_body_current_ = drift.groupPlus(se3_world_body_current_);
    for (auto& [id, data] : map_->map_points) {
        auto& [position, _] = data;
        position = drift.vectorPlus(position);
    }
    map_->reference_keyframe_pose = drift.groupPlus(map_->reference_keyframe_pose);
}

auto LocalTracking::callback(boost::shared_ptr<const void> message, size_t cam_idx) -> void {
    // Convert message.
    auto image_msg = boost::static_pointer_cast<const sensor_msgs::Image>(std::move(message));
    auto cv_image_ptr = cv_bridge::toCvShare(image_msg, "mono8");

    // Check stamp
    const Timestamp ts = image_msg->header.stamp.toNSec();
    auto itr = image_queue_.find(ts);
    if (itr == image_queue_.end()) { // First frame
        itr = image_queue_.insert({ts, {}}).first;
    }
    auto& image_map = itr->second;
    image_map.insert({cam_idx, std::move(cv_image_ptr)});

    if (image_map.size() == 2) {
        Images images;
        for (const auto& [_, img_ptr] : image_map) {
            images.emplace_back(&img_ptr->image);
        }
        this->submit(images, ts);
        image_queue_.erase(itr);
    }
}

auto LocalTracking::submit(const Images& images, const Timestamp& timestamp) -> void {
    DCHECK(images.size() == 2) << "Local tracking needs stereo frames";

    // Extract features.
    auto timer = Timer("Extraction");
    timer.tic();
    auto image_data = extractImageData(images, cameras_, detector_);
    timer.toc();
    // timer.display();

    // Initialize map if empty.
    if (map_ == nullptr) {
        initialize_(timestamp, image_data, images);
    } else {
        localize_(timestamp, image_data, images);
    }
}

auto LocalTracking::initialize_(const Timestamp& timestamp, std::vector<Frame::ImageData>& image_data, const std::vector<const cv::Mat*>& images) -> void {
    auto keyframe = generateNewKeyframe_(timestamp, image_data, images);
    local_mapping_->submit({std::move(keyframe), std::set<ObservationKey>(), std::set<StateId>()});
}

auto LocalTracking::localize_(const Timestamp& timestamp, std::vector<Frame::ImageData>& image_data, const std::vector<const cv::Mat*>& images) -> void {
    auto lock = std::unique_lock{map_mutex_};
    DCHECK(map_->world_frame_id == world_frame_id_);
    // DLOG(INFO) << "Tracking: Localizing frame using " << map_->map_points.size() << " map points.";
    const auto num_images = image_data.size();
    std::vector<Translation> map_point_positions;
    cv::Mat map_point_descriptors;
    for (const auto& [id, data] : map_->map_points) {
        const auto& [position, descriptor] = data;
        map_point_positions.push_back(position);
        map_point_descriptors.push_back(descriptor);
    }

    // Match map point descriptors to keypoint descriptors in all cameras.
    std::vector<Matches> camera_matches;
    for (auto i = 0u; i < num_images; ++i) {
        const auto se3_camera_world = (se3_world_body_current_.groupPlus(cameras_[i]->transformation().groupInverse())).groupInverse();
        // const cv::Mat mask = projectionMask(map_point_positions, image_data[i].bearings, se3_camera_world, parameters_.projection_threshold);
        camera_matches.push_back(descriptorMatching(map_point_descriptors, image_data[i].keypoint_descriptors, parameters_.matching_threshold));
    }
    // Filter out matches using epipolar constraint.
    filterEpipolar3D2D(image_data[0].bearings, image_data[1].bearings,
        camera_matches[0], camera_matches[1], essential_matrix_left_right_, parameters_.epipolar_threshold);

    SE3 se3_world_body;
    bool tracked = true;
    if (camera_matches[0].size() + camera_matches[1].size() >= 8) {
        // Localize frame using 3d to 2d RANSAC + nonlinear refinement.
        se3_world_body = ransac3d2d(map_point_positions, {image_data[0].bearings, image_data[1].bearings},
            camera_matches, {cameras_[0]->transformation(), cameras_[1]->transformation()}, se3_world_body_current_, parameters_.ransac_threshold);
    } else {
        se3_world_body = se3_world_body_current_;
        tracked = false;
        DLOG(INFO) << "Tracking failed.";
    }

    // Update state.
    se3_world_body_current_ = se3_world_body;

    // Extract tracked map point ids and positions.
    auto tracked_map_point_ids = std::set<StateId>{};
    auto tracked_positions = std::vector<Position>{};
    for (auto& matches : camera_matches) {
        for (const auto& match : matches) {
            auto itr = std::next(map_->map_points.begin(), match.queryIdx);
            auto [_, inserted] = tracked_map_point_ids.insert(itr->first);
            if (inserted) {
                tracked_positions.push_back(map_point_positions[match.queryIdx]);
            }
        }
    }
    // DLOG(INFO) << "Tracked " << tracked_map_point_ids.size() << " ids.";

    // Compute ratio between average depth of tracked points and baseline to reference.
    const auto average_depth = computeAverageDepth(tracked_positions, se3_world_body);
    const Scalar baseline = map_->reference_keyframe_pose.groupInverse().groupPlus(se3_world_body).translation().norm();
    const auto ratio = baseline / average_depth;

    // Compute reference keyframe id.
    auto max_covisibility_weight = 0;
    auto new_reference_id = map_->reference_keyframe_id;
    auto local_frames = std::set<StateId>();
    for (const auto& [keyframe_id, keyframe_map_point_ids] : map_->local_slam_graph) {
        auto keyframe_tracked_set = std::vector<StateId>{};
        std::set_intersection(tracked_map_point_ids.begin(), tracked_map_point_ids.end(),
            keyframe_map_point_ids.begin(), keyframe_map_point_ids.end(),
            std::back_inserter(keyframe_tracked_set));
        const auto covisibility_weight = static_cast<int>(keyframe_tracked_set.size());
        if (covisibility_weight > max_covisibility_weight) {
            max_covisibility_weight = covisibility_weight;
            new_reference_id = keyframe_id;
        }
        local_frames.insert(keyframe_id);
    }

    // Visualization.
    /*showTracks("Stereo Matches Agent " + std::to_string(id_), *images[0], *images[1], camera_matches[0], camera_matches[1],
        image_data[0].keypoints, image_data[1].keypoints, color_);*/
    // TODO: Use the reference state here.
    visualizer_->showCameraPose(se3_world_body_current_, se3_world_body_current_);

    // Spawn new keyframe according to ratio and number of tracked ids.
    const bool geometric_condition = (ratio > baseline_depth_ratio_ || tracked_map_point_ids.size() < min_num_points_tracked_);
    const bool rate_condition = is_map_reset_ && tracked;
    if (geometric_condition && rate_condition) {
        auto keyframe = generateNewKeyframe_(timestamp, image_data, images);
        std::set<ObservationKey> observations;
        for (auto i = 0u; i < image_data.size(); ++i) {
            for (const auto& match : camera_matches[i]) {
                // DCHECK(match.queryIdx < map_->map_point_ids.size()) << match.queryIdx << ", " << map_->map_point_ids.size();
                auto itr = std::next(map_->map_points.begin(), match.queryIdx);
                observations.insert(ObservationKey{.frame_id = keyframe->id,
                    .map_point_id = itr->first,
                    .camera_index = i,
                    .keypoint_index = match.trainIdx});
            }
        }
        is_map_reset_ = false;
        local_mapping_->submit({std::move(keyframe), std::move(observations), local_frames});
    } else {
        // Trigger map update if the reference changed.
        if (new_reference_id != map_->reference_keyframe_id) {
            DLOG(INFO) << "Tracking: Change reference to " << new_reference_id;
            is_map_reset_ = false;
            lock.unlock(); // Return the lock early here.
            local_storage_->setTrackingReference(new_reference_id);
        }
    }
}

auto LocalTracking::generateNewKeyframe_(const Timestamp& timestamp, std::vector<Frame::ImageData>& image_data, const std::vector<const cv::Mat*>& images) -> std::unique_ptr<Frame> {
    // Construct key.
    auto id = StateId{id_, keyframe_count_};

    // Construct keyframe.
    auto keyframe = std::make_unique<Frame>();
    keyframe->id = id;
    keyframe->timestamp = timestamp;
    for (auto i = 0u; i < image_data.size(); ++i) {
        keyframe->images.emplace_back(images[i]->clone());
        keyframe->image_data.emplace_back(image_data[i]);
        keyframe->calibration.emplace_back(Frame::Calibration{
            .se3_body_camera_ = cameras_[i]->transformation(),
            .intrinsics_ = cameras_[i]->intrinsics(),
            .distortions_ = dynamic_cast<hyper::RadialTangentialDistortion<Scalar, 2>&>(cameras_[i]->distortion())});
    }
    keyframe->essential_matrix = essential_matrix_left_right_;
    keyframe->se3_world_body = se3_world_body_current_;
    keyframe->reference_frame_id = world_frame_id_;

    // Increase count.
    ++keyframe_count_;

    // DLOG(INFO) << "Tracking: Created keyframe " << id;
    return keyframe;
}

} // namespace deco
