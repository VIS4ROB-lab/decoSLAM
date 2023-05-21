//
// Created by philipp on 05.10.22.
//

#include <fstream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "messages/shared_map_messages.hpp"
#include "optimization/bundle_adjustment_problem.hpp"
#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/alignment.hpp"
#include "system/communicator.hpp"
#include "system/local_tracking.hpp"
#include "system/optimizer.hpp"
#include "visualization/helpers.hpp"
#include "visualization/visualizer.hpp"

#include "utils/timer.hpp"

namespace deco {

LocalStorage::LocalStorage(const Id& id, StorageMutexHandler& mutex_handler)
    : id_{id},
      world_frame_id_{id},
      drift_{SE3::Identity()},
      tracking_reference_{id, INT_MAX},
      tracking_update_notified_{false},
      optimization_notified_{false},
      mutex_handler_{mutex_handler} {}

LocalStorage::~LocalStorage() = default;

auto LocalStorage::readParameters(const YAML::Node& node) -> void {
    do_map_sharing_ = node["do_map_sharing"].as<bool>();
    pixel_noise_ = node["pixel_noise"].as<Scalar>();
    huber_loss_threshold_ = node["huber_loss_threshold"].as<Scalar>();
    outlier_threshold_ = node["outlier_threshold"].as<Scalar>();
    admm_gamma_ = node["admm_gamma"].as<Scalar>();
    max_num_iterations_ = node["max_num_iterations"].as<int>();
    optimizer_rate_us_ = node["rate"].as<int>()*1e6;
}

auto LocalStorage::setCommunicator(Communicator* communicator) -> void {
    communicator_ = communicator;
}

auto LocalStorage::setSharedMap(SharedMap* shared_map) -> void {
    shared_map_ = shared_map;
}

auto LocalStorage::setLocalTracking(LocalTracking* local_tracking) -> void {
    local_tracking_ = local_tracking;
}

/*auto LocalStorage::setOptimizer(Optimizer* optimizer) -> void {
    optimizer_ = optimizer;
}*/

auto LocalStorage::setAlignment(Alignment* alignment) -> void {
    alignment_ = alignment;
}

auto LocalStorage::setVisualizer(Visualizer* visualizer) -> void {
    visualizer_ = visualizer;
}

auto LocalStorage::start() -> void {
    tracking_update_thread_ = std::thread{&LocalStorage::trackingUpdate_, this};
    optimization_thread_ = std::thread{&LocalStorage::optimize_, this};
}

auto LocalStorage::stop() -> void {
    tracking_update_condition_.notify_one();
    tracking_update_thread_.join();

    optimization_condition_.notify_one();
    optimization_thread_.join();
}

auto LocalStorage::writePoses(const FilePath& output_path) -> void {
    const auto separator = ", ";
    const auto filepath = output_path / "poses.csv";
    std::ofstream file_stream;
    file_stream << std::scientific << std::setprecision(18);
    file_stream.open(filepath);

    // Header.
    file_stream << "id" << separator
                << "tx" << separator
                << "ty" << separator
                << "tz" << separator
                << "qx" << separator
                << "qy" << separator
                << "qz" << separator
                << "qw"
                << "\n";

    for (const auto& [id, entry] : keyframes_) {
        if (entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED) {
            const auto& pose = entry.pose;
            file_stream << id << separator
                        << pose.translation().x() << separator
                        << pose.translation().y() << separator
                        << pose.translation().z() << separator
                        << pose.rotation().x() << separator
                        << pose.rotation().y() << separator
                        << pose.rotation().z() << separator
                        << pose.rotation().w() << "\n";
        }
    }
    file_stream.close();
}

auto LocalStorage::writeLogs(const FilePath& output_path) -> void {
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

auto LocalStorage::submitMessage(std::unique_ptr<MessageBase>&& message) -> void {
    DCHECK(message->type == MessageType::LOCAL_STORAGE);
    auto& local_storage_message = dynamic_cast<LocalStorageMessageBase&>(*message);

    switch (local_storage_message.subtype) {
        case LocalStorageMessageType::OBSERVATION_STORAGE:
            observationStorage_(local_storage_message);
            break;
        case LocalStorageMessageType::EDGE_UPDATES:
            updateEdges_(local_storage_message);
            break;
        case LocalStorageMessageType::NEW_KEYFRAMES:
            addNewKeyframes_(local_storage_message);
            break;
        case LocalStorageMessageType::NEW_KEYFRAME_DUALS:
            addNewKeyframeDuals_(local_storage_message);
            break;
        case LocalStorageMessageType::KEYFRAME_DUAL_UPDATES:
            updateKeyframeDuals_(local_storage_message);
            break;
        case LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS:
            removeKeyframeDuals_(local_storage_message);
            break;
        case LocalStorageMessageType::KEYFRAME_REMOVALS:
            removeKeyframes_(local_storage_message);
            break;
        case LocalStorageMessageType::NEW_MAP_POINTS:
            addNewMapPoints_(local_storage_message);
            break;
        case LocalStorageMessageType::NEW_MAP_POINT_DUALS:
            addNewMapPointDuals_(local_storage_message);
            break;
        case LocalStorageMessageType::MAP_POINT_DUAL_UPDATES:
            updateMapPointDuals_(local_storage_message);
            break;
        case LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS:
            removeMapPointDuals_(local_storage_message);
            break;
        case LocalStorageMessageType::MAP_POINT_REMOVALS:
            removeMapPoints_(local_storage_message);
            break;
        default:
            LOG(FATAL) << "Unknown message type.";
    }
}

auto LocalStorage::setTrackingReference(const StateId& frame_id) -> void {
    auto lock = std::unique_lock<std::mutex>{tracking_update_mutex_};
    const auto debug_changed = tracking_reference_ != frame_id;
    tracking_reference_ = frame_id;
    tracking_update_notified_ = true;
    lock.unlock();
    tracking_update_condition_.notify_one();
    DLOG_IF(INFO, debug_changed) << id_ << ": Set new tracking reference " << frame_id;
}

auto LocalStorage::notifyTracker() -> void {
    auto lock = std::unique_lock<std::mutex>{tracking_update_mutex_};
    tracking_update_notified_ = true;
    lock.unlock();
    tracking_update_condition_.notify_one();
}

auto LocalStorage::notifyOptimization() -> void {
    auto lock = std::unique_lock{optimization_mutex_};
    optimization_notified_ = true;
    lock.unlock();
    optimization_condition_.notify_one();
}

auto LocalStorage::addMapPointsToCullingQueue(const std::set<StateId>& map_point_ids) -> void {
    for (const auto& id : map_point_ids) {
        map_point_culling_queue_.push_front(id);
    }
}

auto LocalStorage::registerKeyframeForDriftCorrection(const StateId& frame_id) -> void {
    auto lock = mutex_handler_.lockLocalStorage();
    DCHECK(keyframes_.contains(frame_id));
    auto& entry = keyframes_.find(frame_id)->second;
    DCHECK(entry.storage_status == StorageStatus::ACTIVE);
    new_keyframe_ids_.insert(frame_id);
}

auto LocalStorage::registerMapPointsForDriftCorrection(const std::set<StateId>& map_point_ids) -> void {
    auto lock = mutex_handler_.lockLocalStorage();
    for (const auto& id : map_point_ids) {
        DCHECK(map_points_.contains(id));
        auto& entry = map_points_.find(id)->second;
        DCHECK(entry.storage_status == StorageStatus::ACTIVE);
        new_map_point_ids_.insert(id);
    }
}

auto LocalStorage::applyDriftCorrection(const SE3& drift) -> void {
    {
        auto lock = mutex_handler_.lockLocalStorage();

        for (auto& id : new_keyframe_ids_) {
            DCHECK(keyframes_.contains(id));
            auto& entry = keyframes_.find(id)->second;
            DCHECK(entry.storage_status != StorageStatus::OPTIMIZED);
            entry.pose = drift.groupPlus(entry.pose);
        }
        new_keyframe_ids_.clear();

        for (auto& id : new_map_point_ids_) {
            DCHECK(map_points_.contains(id));
            auto& entry = map_points_.find(id)->second;
            DCHECK(entry.storage_status != StorageStatus::OPTIMIZED);
            entry.position = drift.vectorPlus(entry.position);
        }
        new_map_point_ids_.clear();
        drift_ = drift;
    }

    {
        auto lock = std::lock_guard{local_tracking_->mutex()};
        local_tracking_->applyDriftCorrection(drift);
    }

    // drift_ = SE3::Identity();

    // DLOG(INFO) << "Corrected " << new_keyframe_ids_.size() << " keyframe poses and " << new_map_point_ids_.size() << " map point positions.";

    // DLOG(INFO) << drift.translation().transpose();
    notifyTracker();
}

auto LocalStorage::requestKeyframeRemovals() -> void {
    auto lock = mutex_handler_.lockAllStorages();

    // Check map point culling queue for removable points.
    auto map_point_deletion_requests = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS>>>();
    auto non_removable_underconstrained_points = std::set<StateId>();
    auto debug_num_checked = 0;
    while (map_point_culling_queue_.size() > 1500) {
        const auto& id = map_point_culling_queue_.back();

        DCHECK(map_points_.contains(id));
        auto& entry = map_points_.find(id)->second;
        auto observing_frames = map_.getObservingFrames(id);
        const auto is_underconstrained = observing_frames.size() < 2;
        if (is_underconstrained) {
            const auto is_currently_removable = entry.storage_status == StorageStatus::OPTIMIZED && !entry.is_local && !entry.has_dual;
            if (is_currently_removable) {
                auto itr = map_point_deletion_requests.find(id.first);
                if (itr == map_point_deletion_requests.end()) {
                    itr = map_point_deletion_requests.insert({id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS>>()}).first;
                }
                itr->second->map_point_ids.insert(id);
                entry.storage_status = StorageStatus::REMOVAL_REQUESTED;
            } else {
                auto [itr, inserted] = non_removable_underconstrained_points.insert(id);
                if (inserted) {
                    map_point_culling_queue_.push_front(id);
                }
            }
        }
        map_point_culling_queue_.pop_back();
    }
    // DLOG_IF(INFO, debug_num_checked > 0) << "Checked " << debug_num_checked << " map point ids.";

    // Gather requests for all keyframes with a dual outside the neighborhood.
    auto keyframe_deletion_requests = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::KEYFRAME_DELETION_REQUESTS>>>();
    for (auto& [frame_id, entry] : keyframes_) {
        const auto is_not_local = !entry.is_local;
        const auto is_removable = entry.storage_status == StorageStatus::OPTIMIZED && entry.has_dual;
        if (is_not_local && is_removable) {
            auto itr = keyframe_deletion_requests.find(frame_id.first);
            if (itr == keyframe_deletion_requests.end()) {
                itr = keyframe_deletion_requests.insert({frame_id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::KEYFRAME_DELETION_REQUESTS>>()}).first;
            }
            itr->second->keyframe_ids.insert(frame_id);
            entry.storage_status = StorageStatus::REMOVAL_REQUESTED;
            DLOG(INFO) << id_ << ": Requested removal of " << frame_id;
        }
    }

    for (auto& [id, request] : keyframe_deletion_requests) {
        const auto num_requested = request->keyframe_ids.size();
        // DLOG(INFO) << "------------------------------------------";
        // DLOG(INFO) << id_ << ": Requesting " << num_requested << " keyframe removals from " << id;
        for (const auto& frame_id : request->keyframe_ids) {
            // DLOG(INFO) << frame_id;
        }
        request->receiver_id = id;
        communicator_->send(std::move(request));
        // DLOG(INFO) << id_ << ": Requesting " << num_requested << " keyframe removals from " << id << " done.";
        // DLOG(INFO) << "------------------------------------------";
    }

    for (auto& [id, request] : map_point_deletion_requests) {
        const auto num_requested = request->map_point_ids.size();
        // DLOG(INFO) << "------------------------------------------";
        // DLOG(INFO) << id_ << ": Requesting " << num_requested << " UC map point removals from " << id;
        request->receiver_id = id;
        communicator_->send(std::move(request));
        // DLOG(INFO) << id_ << ": Requesting " << num_requested << " UC map point removals from " << id << " done.";
        // DLOG(INFO) << "------------------------------------------";
    }
}

auto LocalStorage::applyTransformation(const Id& new_world_frame_id, const SE3& transformation) -> void {
    mutex_handler_.lockLocalStorage();
    DCHECK(world_frame_id_ != new_world_frame_id);
    for (auto& [id, entry] : keyframes_) {
        if (entry.storage_status != StorageStatus::EMPTY || entry.storage_status != StorageStatus::REMOVED) {
            const auto reference_after = transformation.groupPlus(entry.reference_pose);
            const auto pose_after = transformation.groupPlus(entry.pose);
            if (entry.has_dual) {
                const auto tangent_before = (entry.reference_pose.groupInverse().groupPlus(entry.pose)).toTangent();
                const auto tangent_after = (reference_after.groupInverse().groupPlus(pose_after)).toTangent();
                entry.dual = entry.dual + admm_gamma_ * entry.cardinality * (tangent_before - tangent_after);
            }
            entry.reference_pose = reference_after;
            entry.pose = pose_after;
        }
    }

    for (auto& [id, entry] : map_points_) {
        if (entry.storage_status != StorageStatus::EMPTY || entry.storage_status != StorageStatus::REMOVED) {
            if (entry.has_dual) {
                entry.dual = transformation.rotation().vectorPlus(entry.dual) - admm_gamma_ * entry.cardinality * transformation.translation();
            }
            entry.position = transformation.vectorPlus(entry.position);
        }
    }

    // DLOG(INFO) << "Change world frame id in local storage from " << world_frame_id_ << " to " << new_world_frame_id;
    world_frame_id_ = new_world_frame_id;
}

auto LocalStorage::addConstantKeyframeData(const StateId& frame_id, const std::vector<Frame::ImageData>& image_data, const std::vector<Frame::Calibration>& calibration) -> void {
    auto& entry = fetchOrCreateKeyframeEntry_(frame_id);
    DCHECK(!entry.has_constant_data) << "Repeatedly assigning constant data to KF " << frame_id;
    entry.image_data = image_data;
    entry.calibration = calibration;
    entry.has_constant_data = true;
}

auto LocalStorage::getKeyframeImageData(const StateId& frame_id) -> std::vector<Frame::ImageData>* {
    DCHECK(keyframes_.contains(frame_id));
    return &(keyframes_.find(frame_id)->second.image_data);
}

auto LocalStorage::getKeyframeCalibration(const StateId& frame_id) -> std::vector<Frame::Calibration> {
    DCHECK(keyframes_.contains(frame_id));
    return keyframes_.find(frame_id)->second.calibration;
}

auto LocalStorage::addKeyframePose(const StateId& frame_id, const SE3& pose) -> void {
    auto& entry = fetchOrCreateKeyframeEntry_(frame_id);
    entry.pose = pose;
    DCHECK(entry.storage_status == StorageStatus::EMPTY || entry.storage_status == StorageStatus::REMOVED);
    entry.storage_status = StorageStatus::ACTIVE;
    DCHECK(!covisibility_graph_.hasNode(frame_id));
    covisibility_graph_.addNode(frame_id);
}

auto LocalStorage::hasKeyframePose(const StateId& frame_id) -> bool {
    if (keyframes_.contains(frame_id)) {
        const auto& entry = keyframes_.find(frame_id)->second;
        return entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED;
    } else {
        return false;
    }
}

auto LocalStorage::getKeyframePose(const StateId& frame_id) -> SE3 {
    DCHECK(hasKeyframePose(frame_id)) << frame_id;
    return keyframes_.find(frame_id)->second.pose;
}

auto LocalStorage::addConstantMapPointData(const StateId& map_point_id, const cv::Mat& descriptor) -> void {
    auto& entry = fetchOrCreateMapPointEntry_(map_point_id);
    DCHECK(!entry.has_constant_data) << "Repeatedly assigning constant data to MP " << map_point_id;
    entry.descriptor = descriptor;
    entry.has_constant_data = true;
}

auto LocalStorage::getMapPointDescriptor(const StateId& map_point_id) -> cv::Mat {
    DCHECK(map_points_.contains(map_point_id));
    return map_points_.find(map_point_id)->second.descriptor;
}

auto LocalStorage::addMapPointPosition(const StateId& map_point_id, const Position& position) -> void {
    auto& entry = fetchOrCreateMapPointEntry_(map_point_id);
    entry.position = position;
    DCHECK(entry.storage_status == StorageStatus::EMPTY || entry.storage_status == StorageStatus::REMOVED);
    entry.storage_status = StorageStatus::ACTIVE;
}

auto LocalStorage::hasMapPointPosition(const StateId& map_point_id) -> bool {
    if (map_points_.contains(map_point_id)) {
        const auto& entry = map_points_.find(map_point_id)->second;
        return entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED;
    } else {
        return false;
    }
}

auto LocalStorage::getMapPointPosition(const StateId& map_point_id) -> Position {
    DCHECK(hasMapPointPosition(map_point_id));
    return map_points_.find(map_point_id)->second.position;
}

auto LocalStorage::triggerVisualization() -> void {
    auto lock = mutex_handler_.lockLocalStorage();
    constexpr auto shared_color = Visualizer::Color{255, 255, 255};
    const auto agent_color = getColor(id_);

    std::map<StateId, std::pair<Translation, Visualizer::Color>> keyframe_positions;
    std::vector<std::pair<StateId, StateId>> edges;
    // DLOG(INFO) << "------------------------------";
    for (const auto& [id, entry] : keyframes_) {
        if (entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED) {
            auto color = entry.has_dual ? shared_color : agent_color;
            // DLOG(INFO) << "Node " << id;
            keyframe_positions.insert({id, {entry.pose.translation(), color}});
            for (const auto& [neighbor_id, _] : covisibility_graph_.getNeighbors(id, kMinCovisibilityWeight)) {
                const bool should_have_entry = do_map_sharing_ || neighbor_id.first == id_;
                if (covisibility_graph_.hasNode(neighbor_id) && should_have_entry) {
                    DCHECK(keyframes_.contains(neighbor_id));
                    const auto& neighbor_entry = keyframes_.find(neighbor_id)->second;
                    if (neighbor_entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED) {
                        edges.emplace_back(id, neighbor_id);
                        auto neighbor_color = neighbor_entry.has_dual ? shared_color : agent_color;
                        if (!keyframe_positions.contains(neighbor_id)) {
                            keyframe_positions.insert({neighbor_id, {neighbor_entry.pose.translation(), neighbor_color}});
                        }
                        // DLOG(INFO) << "Neighbor " << neighbor_id;
                    }
                }
            }
        }
    }
    // DLOG(INFO) << "------------------------------";

    for (const auto& [id_1, id_2] : edges) {
        DCHECK(keyframe_positions.contains(id_1));
        DCHECK(keyframe_positions.contains(id_2));
    }

    /*std::vector<Translation> map_point_positions;
    std::vector<Visualizer::Color> map_point_colors;
    auto map_point_ids = std::vector<StateId>();
    for (const auto& [id, entry] : map_points_) {
        if (entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED) {
            auto color = entry.has_dual ? shared_color : agent_color;
            map_point_positions.push_back(entry.position);
            map_point_colors.push_back(color);
            map_point_ids.emplace_back(id);
        }
    }*/

    visualizer_->visualizeGraph(keyframe_positions, edges);
    // visualizer_->visualizeMapPoints(map_point_positions, map_point_colors, map_point_ids);
}

auto LocalStorage::getMap() -> Map* {
    return &map_;
}

auto LocalStorage::fetchOrCreateKeyframeEntry_(const StateId& frame_id) -> KeyframeEntry& {
    auto itr = keyframes_.find(frame_id);
    if (itr == keyframes_.end()) {
        itr = keyframes_.insert({frame_id, {}}).first;
    }
    return itr->second;
}

auto LocalStorage::fetchOrCreateMapPointEntry_(const StateId& map_point_id) -> MapPointEntry& {
    auto itr = map_points_.find(map_point_id);
    if (itr == map_points_.end()) {
        itr = map_points_.insert({map_point_id, {}}).first;
    }
    return itr->second;
}

auto LocalStorage::observationStorage_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::OBSERVATION_STORAGE>&>(message);

    // Synchronize with shared map due to map point requests.
    auto lock = mutex_handler_.lockLocalStorage();

    auto map_point_requests = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_REQUESTS>>>();
    auto map_point_removal_requests = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS>>>();
    auto debug_num_requested = 0;
    auto debug_num_removals_requested = 0;
    // DLOG(INFO) << "Storing " << query.keys.size() << " observations." << static_cast<int>(query.keys.begin()->second);
    for (const auto& [key, storage_action] : query.keys) {
        const auto& keyframe_id = key.frame_id;
        const auto& map_point_id = key.map_point_id;

        switch (storage_action) {
            case StorageAction::ADD: {
                DCHECK(covisibility_graph_.hasNode(keyframe_id));
                if (!map_.hasObservationForCamera(key.frame_id, key.map_point_id, key.camera_index)) {
                    map_.addObservation(key);
                }
                auto& entry = fetchOrCreateMapPointEntry_(map_point_id);
                if (entry.storage_status == StorageStatus::EMPTY || entry.storage_status == StorageStatus::REMOVED) {
                    entry.storage_status = StorageStatus::REQUESTED;
                    auto itr = map_point_requests.find(map_point_id.first);
                    if (itr == map_point_requests.end()) {
                        itr = map_point_requests.insert({map_point_id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_REQUESTS>>()}).first;
                    }
                    auto& request = itr->second;

                    request->map_point_ids.insert({map_point_id, !entry.has_constant_data});
                    ++debug_num_requested;
                }
            } break;
            case StorageAction::REMOVE: {
                if (map_.hasObservation(key)) {
                    map_.removeObservation(key);
                    if (!map_.hasMapPoint(map_point_id)) {
                        DCHECK(map_points_.contains(map_point_id));
                        auto& entry = map_points_.find(map_point_id)->second;
                        DCHECK(entry.storage_status != StorageStatus::EMPTY || entry.storage_status != StorageStatus::REMOVED);
                        if (entry.storage_status != StorageStatus::REMOVAL_REQUESTED) {
                            // DLOG_IF(INFO, entry.storage_status == StorageStatus::REQUESTED) << "Removing requested MP " << map_point_id;
                            entry.storage_status = StorageStatus::REMOVAL_REQUESTED;
                            auto itr = map_point_removal_requests.find(map_point_id.first);
                            if (itr == map_point_removal_requests.end()) {
                                itr = map_point_removal_requests.insert({map_point_id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS>>()}).first;
                            }
                            auto& request = itr->second;
                            request->map_point_ids.insert(map_point_id);
                            ++debug_num_removals_requested;
                        }
                    }
                    // DLOG_IF(INFO, !map_.hasMapPoint(key.map_point_id)) << "Removed all observations of" << key.map_point_id;
                } else {
                    // DLOG(INFO) << "Try to remove inexistent observation " << key;
                }
            } break;
        }
    }

    // DLOG_IF(INFO, debug_num_requested > 0) << id_ << ": Requesting " << debug_num_requested << " map points.";
    for (auto& [id, request] : map_point_requests) {
        request->receiver_id = id;
        communicator_->send(std::move(request));
    }
    // DLOG_IF(INFO, debug_num_requested > 0) << id_ << ": Request of " << debug_num_requested << " map points done.";

    // DLOG_IF(INFO, debug_num_removals_requested > 0) << id_ << ": Requesting removal of " << debug_num_removals_requested << " map points.";
    for (auto& [id, request] : map_point_removal_requests) {
        request->receiver_id = id;
        communicator_->send(std::move(request));
    }
    // DLOG_IF(INFO, debug_num_removals_requested > 0) << id_ << ": Requesting of removal of " << debug_num_removals_requested << " map points done.";
}

auto LocalStorage::updateEdges_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::EDGE_UPDATES>&>(message);

    auto lock = mutex_handler_.lockLocalStorage();
    for (const auto& [edge, weight_increment] : query.edge_updates) {
        if (weight_increment != 0) {
            DCHECK(covisibility_graph_.hasNode(edge.first));
            if (!covisibility_graph_.hasEdge(edge.first, edge.second)) {
                DCHECK(weight_increment > 0) << "Negative weight on zero edge.";
                covisibility_graph_.addEdge(edge.first, edge.second, weight_increment);
                // DLOG(INFO) << id_ << ": Add edge " << edge.first << " --> " << edge.second << " with weight " << weight_increment;
            } else {
                covisibility_graph_.incrementEdge(edge.first, edge.second, weight_increment);
                // DLOG(INFO) << id_ << ": Change weight of edge " << edge.first << " --> " << edge.second << " to " << new_weight;
            }
        } else {
            // DLOG(INFO) << "Zero weight increment.";
        }
    }
}

auto LocalStorage::addNewKeyframes_(LocalStorageMessageBase& message) -> void {
    auto& response = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::NEW_KEYFRAMES>&>(message);
    auto lock = mutex_handler_.lockAllStorages();
    DCHECK(response.world_frame_id == world_frame_id_) << "World frame ids of keyframes don't match!" << response.world_frame_id << " vs. " << world_frame_id_;

    // DLOG(INFO) << response.receiver_id << ": Obtained " << response.keyframes.size() << " keyframes from " << response.sender_id;
    for (const auto& [id, pose, image_data, calibration, initial_dual, reference, cardinality] : response.keyframes) {
        DCHECK(keyframes_.contains(id));
        auto& entry = keyframes_.find(id)->second;
        DCHECK(entry.storage_status == StorageStatus::REQUESTED);
        DCHECK(covisibility_graph_.hasNode(id));
        entry.storage_status = StorageStatus::ACTIVE;
        if (!entry.has_constant_data) {
            DCHECK(!image_data.empty());
            DCHECK(!calibration.empty());
            entry.image_data = image_data;
            entry.calibration = calibration;
            entry.has_constant_data = true;
        }
        entry.pose = pose;
        entry.has_dual = true;
        entry.dual = initial_dual * cardinality;
        // DLOG(INFO) << "Initial dual " << id << " " << initial_dual;
        entry.reference_pose = reference;
        entry.cardinality = cardinality;
        // DLOG(INFO) << id;
    }

    // Use internal message to store observations.
    auto observation_message = LocalStorageMessage<LocalStorageMessageType::OBSERVATION_STORAGE>();
    observation_message.receiver_id = response.receiver_id;
    for (const auto& key : response.observations) {
        observation_message.keys.insert({key, StorageAction::ADD});
    }
    observationStorage_(observation_message);
    // DLOG(INFO) << response.receiver_id << ": Stored observations for " << response.keyframes.size() << " keyframes from " << response.sender_id;

    // Use internal message to store additional edges.
    auto edge_update_message = LocalStorageMessage<LocalStorageMessageType::EDGE_UPDATES>();
    edge_update_message.receiver_id = response.receiver_id;
    edge_update_message.edge_updates.insert(response.edge_updates.begin(), response.edge_updates.end());
    updateEdges_(edge_update_message);
    // DLOG(INFO) << response.receiver_id << ": Updated edges for " << response.keyframes.size() << " keyframes from " << response.sender_id;

    // DLOG(INFO) << response.receiver_id << ": Storing of " << response.keyframes.size() << " keyframes from " << response.sender_id << " done.";
}

auto LocalStorage::addNewKeyframeDuals_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::NEW_KEYFRAME_DUALS>&>(message);

    auto lock = mutex_handler_.lockLocalStorage();
    auto response = std::make_unique<SharedMapMessage<SharedMapMessageType::KEYFRAME_SYNCHRONIZATIONS>>();
    response->receiver_id = query.sender_id;
    for (const auto& [id, reference] : query.duals) {
        DCHECK(keyframes_.contains(id));
        auto& entry = keyframes_.find(id)->second;
        DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
        DCHECK(!entry.has_dual);
        const hyper::Tangent<SE3> initial_dual = Scalar{-1.0} * admm_gamma_ * (reference.groupInverse().groupPlus(entry.pose)).toTangent();
        // DLOG(INFO) << "Initial dual " << id << " " << initial_dual;
        entry.has_dual = true;
        entry.dual = initial_dual;
        entry.reference_pose = reference;
        entry.cardinality = 1;
        response->keyframe_poses_.insert({id, entry.pose});
    }
    // DLOG(INFO) << id_ << ": Added " << query.duals.size() << " keyframe duals.";
    communicator_->send(std::move(response));
}

auto LocalStorage::updateKeyframeDuals_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_UPDATES>&>(message);
    {
        auto lock = mutex_handler_.lockLocalStorage();
        for (const auto& [id, dual_update, reference, cardinality_update] : query.dual_updates) {
            DCHECK(keyframes_.contains(id));
            auto& entry = keyframes_.find(id)->second;
            DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
            DCHECK(entry.has_dual);
            entry.dual = dual_update;
            entry.reference_pose = reference;
            entry.cardinality = cardinality_update;
        }
    }
    notifyOptimization();
}

auto LocalStorage::removeKeyframeDuals_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS>&>(message);

    auto lock = mutex_handler_.lockLocalStorage();
    for (const auto& frame_id : query.keyframe_ids) {
        DCHECK(keyframes_.contains(frame_id));
        auto& entry = keyframes_.find(frame_id)->second;
        DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
        DCHECK(entry.has_dual);
        entry.has_dual = false;
    }

    // DLOG(INFO) << id_ << ": Removed " << query.keyframe_ids.size() << " keyframe duals.";
}

auto LocalStorage::removeKeyframes_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_REMOVALS>&>(message);

    auto lock = mutex_handler_.lockAllStorages();
    // DLOG(INFO) << query.receiver_id << ": Received confirmation of removal of " << query.keyframe_ids.size() << " keyframes.";
    auto map_point_deletion_requests = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS>>>();
    auto debug_num_removed = 0;
    auto deleted_ids = std::set<StateId>();
    for (const auto& [frame_id, is_removed] : query.keyframe_ids) {
        DCHECK(keyframes_.contains(frame_id));
        auto& entry = keyframes_.find(frame_id)->second;
        DCHECK(entry.storage_status == StorageStatus::REMOVAL_REQUESTED);
        if (is_removed) {
            DCHECK(entry.has_dual);
            entry.has_dual = false;
            entry.storage_status = StorageStatus::REMOVED;
            // DCHECK(!entry.is_local) << id_ << ": " << frame_id;
            covisibility_graph_.removeNode(frame_id);
            // DLOG(INFO) << id_ << ": Removed " << frame_id;
            auto observed_map_points = map_.getObservedMapPoints(frame_id);
            auto observations = map_.getObservationsForFrame(frame_id);
            auto remove_message = LocalStorageMessage<LocalStorageMessageType::OBSERVATION_STORAGE>();
            remove_message.sender_id = id_;
            remove_message.receiver_id = id_;
            for (const auto& key : observations) {
                remove_message.keys.insert({key, StorageAction::REMOVE});
            }
            observationStorage_(remove_message);
            deleted_ids.insert(frame_id);
        } else {
            DCHECK(!entry.has_dual);
            entry.storage_status = StorageStatus::ACTIVE;
        }

        // DLOG(INFO) << frame_id;
    }
    visualizer_->deleteNodes(deleted_ids);
    // DLOG(INFO) << id_ << ": Removed " << deleted_ids.size() << " keyframes.";

    DLOG_IF(INFO, debug_num_removed > 0) << id_ << ": Request removal of " << debug_num_removed << " map points.";
    for (auto& [id, deletion_message] : map_point_deletion_requests) {
        deletion_message->receiver_id = id;
        communicator_->send(std::move(deletion_message));
    }
    DLOG_IF(INFO, debug_num_removed > 0) << id_ << ": Request removal of " << debug_num_removed << " map points done.";
}

auto LocalStorage::addNewMapPoints_(LocalStorageMessageBase& message) -> void {
    auto& response = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::NEW_MAP_POINTS>&>(message);
    auto lock = mutex_handler_.lockAllStorages();
    if (response.world_frame_id != world_frame_id_) {
        /*for (auto& [id, position, descriptor, initial_dual, cardinality] : response.map_points) {
            const auto transform = alignment_->getTransformationToWorld(response.world_frame_id);
            position = transform.vectorPlus(position);
            initial_dual = Scalar{-1.0} * admm_gamma_ * position;
        }*/
        // DLOG(INFO) << "World frame ids of map points don't match!" << response.world_frame_id << " vs. " << world_frame_id_;
    }

    for (const auto& id : response.refused_ids) {
        DCHECK(map_points_.contains(id));
        auto& entry = map_points_.find(id)->second;
        DCHECK(entry.storage_status == StorageStatus::REQUESTED || entry.storage_status == StorageStatus::REMOVAL_REQUESTED) << static_cast<int>(entry.storage_status);
        DLOG_IF(INFO, map_.hasMapPoint(id)) << "New map point refused that has not been removed " << id;
        entry.storage_status = StorageStatus::REMOVED;
    }

    auto new_ids = std::set<StateId>();
    // DLOG(INFO) << response.receiver_id << ": Obtained " << response.map_points.size() << " map points from " << response.sender_id;
    for (const auto& [id, position, descriptor, initial_dual, cardinality] : response.map_points) {
        DCHECK(map_points_.contains(id));
        auto& entry = map_points_.find(id)->second;
        DCHECK(entry.storage_status == StorageStatus::REQUESTED || entry.storage_status == StorageStatus::REMOVAL_REQUESTED)
            << static_cast<int>(entry.storage_status);

        if (!entry.has_constant_data) {
            DCHECK(!descriptor.empty());
            entry.descriptor = descriptor;
            entry.has_constant_data = true;
        }

        if (entry.storage_status == StorageStatus::REMOVAL_REQUESTED) {
            // DLOG(INFO) << "Not activating removal-requested MP " << id;
        } else {
            entry.storage_status = StorageStatus::ACTIVE;
            entry.position = position;
            entry.has_dual = true;
            entry.dual = initial_dual * cardinality;
            entry.cardinality = cardinality;
            new_ids.insert(id);
        }
    }
    addMapPointsToCullingQueue(new_ids);
    // DLOG(INFO) << response.receiver_id << ": Storing of " << response.map_points.size() << " map points from " << response.sender_id << " done.";
}

auto LocalStorage::addNewMapPointDuals_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::NEW_MAP_POINT_DUALS>&>(message);

    auto lock = mutex_handler_.lockLocalStorage();
    auto response = std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_SYNCHRONIZATIONS>>();
    response->receiver_id = query.sender_id;
    for (const auto& id : query.duals) {
        DCHECK(map_points_.contains(id));
        auto& entry = map_points_.find(id)->second;
        DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
        DCHECK(!entry.has_dual);
        const auto initial_dual = Scalar{-1.0} * admm_gamma_ * entry.position;
        entry.has_dual = true;
        entry.dual = initial_dual;
        entry.cardinality = 1;
        response->map_point_positions.insert({id, entry.position});
    }
    communicator_->send(std::move(response));

    //  DLOG(INFO) << id_ << ": Added " << query.duals.size() << " map point duals.";
}

auto LocalStorage::updateMapPointDuals_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_UPDATES>&>(message);
    {
        auto lock = mutex_handler_.lockLocalStorage();
        for (const auto& [id, dual_update, cardinality_update] : query.dual_updates) {
            DCHECK(map_points_.contains(id));
            auto& entry = map_points_.find(id)->second;
            DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
            DCHECK(entry.has_dual) << id;
            entry.dual = dual_update;
            entry.cardinality = cardinality_update;
        }
    }
    notifyOptimization();
}

auto LocalStorage::removeMapPointDuals_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS>&>(message);
    auto lock = mutex_handler_.lockLocalStorage();
    // DLOG(INFO) << id_ << ": Removing " << query.map_point_ids.size() << " map point duals.";
    for (const auto& map_point_id : query.map_point_ids) {
        DCHECK(map_points_.contains(map_point_id));
        auto& entry = map_points_.find(map_point_id)->second;
        DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
        DCHECK(entry.has_dual) << map_point_id << " with status " << static_cast<int>(entry.storage_status);
        entry.has_dual = false;
    }
}

auto LocalStorage::removeMapPoints_(LocalStorageMessageBase& message) -> void {
    auto& query = dynamic_cast<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_REMOVALS>&>(message);
    auto lock = mutex_handler_.lockLocalStorage();
    // DLOG(INFO) << query.receiver_id << ": Received confirmation of removal of " << query.map_point_ids.size() << " map points.";
    auto deleted_ids = std::set<StateId>();
    for (const auto& [map_point_id, is_removed] : query.map_point_ids) {
        DCHECK(map_points_.contains(map_point_id));
        auto& entry = map_points_.find(map_point_id)->second;
        DCHECK(entry.storage_status == StorageStatus::REMOVAL_REQUESTED || entry.storage_status == StorageStatus::REQUESTED)
            << map_point_id << " has status " << static_cast<int>(entry.storage_status);

        if (entry.storage_status == StorageStatus::REQUESTED) {
            // DLOG(INFO) << "Ignoring removal of " << map_point_id << " due to re-request.";
            continue;
        }

        if (is_removed) {
            entry.storage_status = StorageStatus::REMOVED;
            entry.has_dual = false;

            // Remove possible re-observations that may have happened since the request was sent.
            auto observations = map_.getObservationsForMapPoint(map_point_id);
            // DLOG_IF(INFO, !observations.empty()) << "Removing " << observations.size() << " re-observations of " << map_point_id;
            for (const auto& key : observations) {
                map_.removeObservation(key);
            }
            deleted_ids.insert(map_point_id);
        } else {
            entry.storage_status = StorageStatus::ACTIVE;
        }
    }
    visualizer_->deleteMapPoints(deleted_ids);
    // DLOG(INFO) << id_ << ": Removed " << deleted_ids.size() << " map points.";
}

auto LocalStorage::loadLocalNeighborhood_(const StateId& tracking_reference_id) -> void {
    auto lock = mutex_handler_.lockAllStorages();
    // Clear old neighborhood.
    for (const auto& local_id : local_keyframe_ids_) {
        auto& entry = keyframes_.find(local_id)->second;
        entry.is_local = false;
    }
    local_keyframe_ids_.clear();

    for (const auto& local_id : local_map_point_ids_) {
        auto& entry = map_points_.find(local_id)->second;
        entry.is_local = false;
    }
    local_map_point_ids_.clear();

    // Identify local frames and insert new nodes if neighbors don't exist yet.
    DCHECK(covisibility_graph_.hasNode(tracking_reference_id)) << id_ << ": " << tracking_reference_id;
    local_keyframe_ids_.insert(tracking_reference_id);
    for (const auto& [frame_id, weight] : covisibility_graph_.getNeighbors(tracking_reference_id, kMinCovisibilityWeight)) {
        if (!covisibility_graph_.hasNode(frame_id)) {
            covisibility_graph_.addNode(frame_id);
        }
        local_keyframe_ids_.insert(frame_id);
        /*for (const auto& [neighbor_id, _] : covisibility_graph_.getNeighbors(frame_id, kMinCovisibilityWeight)) {
            if (!covisibility_graph_.hasNode(neighbor_id)) {
                covisibility_graph_.addNode(neighbor_id);
            }
            local_keyframe_ids_.insert(neighbor_id);
        }*/
    }

    if (!do_map_sharing_) {
        for (auto itr = local_keyframe_ids_.begin(); itr != local_keyframe_ids_.end();) {
            const auto& id = *itr;
            if (id.first != id_) {
                itr = local_keyframe_ids_.erase(itr);
            } else {
                ++itr;
            }
        }
    }

    // Instantiate new entries for missing nodes, build requests, identify local map points.
    auto requests = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::KEYFRAME_REQUESTS>>>();
    for (const auto& id : local_keyframe_ids_) {
        const auto& observed_ids = map_.getObservedMapPoints(id);
        local_map_point_ids_.insert(observed_ids.begin(), observed_ids.end());
        auto& entry = fetchOrCreateKeyframeEntry_(id);
        entry.is_local = true;
        // DLOG(INFO) << "Local id " << id;
        if (entry.storage_status == StorageStatus::EMPTY || entry.storage_status == StorageStatus::REMOVED) {
            entry.storage_status = StorageStatus::REQUESTED;
            DCHECK(covisibility_graph_.hasNode(id));
            auto itr = requests.find(id.first);
            if (itr == requests.end()) {
                itr = requests.insert({id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::KEYFRAME_REQUESTS>>()}).first;
            }
            itr->second->keyframe_ids.insert({id, !entry.has_constant_data});
        }
    }

    // Mark local map point ids
    for (const auto& id : local_map_point_ids_) {
        DCHECK(map_points_.contains(id));
        auto& entry = map_points_.find(id)->second;
        entry.is_local = true;
    }

    for (auto& [id, request] : requests) {
        const auto num_requested = request->keyframe_ids.size();
        // DLOG(INFO) << "------------------------------------------";
        // DLOG(INFO) << id_ << ": Requesting " << num_requested << " keyframes from " << id;
        for (const auto& [frame_id, _] : request->keyframe_ids) {
            // DLOG(INFO) << frame_id;
        }
        request->receiver_id = id;
        communicator_->send(std::move(request));
        // DLOG(INFO) << id_ << ": Requesting " << num_requested << " keyframes from " << id << " done.";
        // DLOG(INFO) << "------------------------------------------";
    }
}

auto LocalStorage::trackingUpdate_() -> void {
    while (!ros::isShuttingDown()) {
        // Wait for reference change.
        auto lock = std::unique_lock<std::mutex>(tracking_update_mutex_);
        tracking_update_condition_.wait(lock, [this]() -> bool { return tracking_update_notified_ || ros::isShuttingDown(); });
        if (ros::isShuttingDown()) {
            break;
        }
        const auto reference_id = tracking_reference_;
        tracking_update_notified_ = false;
        lock.unlock();

        // Fetch the relevant part of the map.
        auto tracking_map = std::make_unique<LocalTracking::TrackingMap>();
        { // begin protected section.
            auto map_lock = mutex_handler_.lockAllStorages();
            loadLocalNeighborhood_(reference_id);
            tracking_map->reference_keyframe_id = reference_id;

            DCHECK(keyframes_.contains(reference_id));
            auto& reference_entry = keyframes_.find(reference_id)->second;
            tracking_map->reference_keyframe_pose = reference_entry.pose;

            for (const auto& frame_id : local_keyframe_ids_) {
                DCHECK(keyframes_.contains(frame_id));
                auto& entry = keyframes_.find(frame_id)->second;
                DCHECK(entry.is_local);
                DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
                if (entry.storage_status != StorageStatus::REQUESTED && entry.storage_status != StorageStatus::REMOVAL_REQUESTED) {
                    auto observed_map_points = map_.getObservedMapPoints(frame_id);
                    auto kf_itr = tracking_map->local_slam_graph.insert({frame_id, {}}).first;
                    DLOG_IF(INFO, entry.storage_status != StorageStatus::ACTIVE && entry.storage_status != StorageStatus::OPTIMIZED)
                        << id_ << ": Loading " << frame_id << " with status " << static_cast<int>(entry.storage_status);
                    for (const auto& map_point_id : observed_map_points) {
                        DCHECK(map_points_.contains(map_point_id));
                        auto& map_point_entry = map_points_.find(map_point_id)->second;
                        DCHECK(map_point_entry.storage_status != StorageStatus::EMPTY && map_point_entry.storage_status != StorageStatus::REMOVED);
                        if (map_point_entry.storage_status != StorageStatus::REQUESTED) {
                            map_point_entry.is_local = true;
                            kf_itr->second.insert(map_point_id);
                            tracking_map->map_points.insert({map_point_id, {map_point_entry.position, map_point_entry.descriptor}});
                        }
                    }
                }
            }

            tracking_map->world_frame_id = world_frame_id_;

        } // end protected section.

        lock.lock();
        const auto new_reference = tracking_reference_;
        lock.unlock();

        if (new_reference != reference_id) {
            DLOG(INFO) << ">>>>>>>>>>>>>>>>>>>>> Change in reference " << reference_id << " -> " << new_reference << " while fetching";
            continue;
        }

        // Submit fetched map to local tracking.
        {
            auto map_lock = std::lock_guard{local_tracking_->mutex()};
            DLOG(INFO) << id_ << ": Submit map with reference " << tracking_map->reference_keyframe_id;
            local_tracking_->setTrackingMap(std::move(tracking_map));
        }
    }
}

auto LocalStorage::optimize_() -> void {
    while (!ros::isShuttingDown()) {
        const auto time_before = std::chrono::high_resolution_clock::now();
        auto lock = std::unique_lock{optimization_mutex_};
        optimization_condition_.wait(lock, [this]() -> bool { return optimization_notified_ || ros::isShuttingDown(); });
        if (ros::isShuttingDown()) {
            break;
        }
        optimization_notified_ = false;
        lock.unlock();

        // Access the current tracking reference id.
        auto tracking_lock = std::unique_lock{tracking_update_mutex_};
        auto drift_reference_id = tracking_reference_;
        tracking_lock.unlock();

        // Instantiate new problem.
        auto global_problem = std::make_unique<BundleAdjustmentProblem>(pixel_noise_, huber_loss_threshold_, admm_gamma_);
        std::set<StateId> optimized_frame_ids, optimized_map_point_ids;
        std::set<ObservationKey> observations;
        Id reference_frame_id_before;
        {
            auto storage_lock = mutex_handler_.lockLocalStorage();
            new_keyframe_ids_.clear();
            new_map_point_ids_.clear();

            // Get world frame id and current reference frame pose.
            reference_frame_id_before = world_frame_id_;

            // Add all frame poses.
            optimized_frame_ids = map_.getFrameIds();
            for (const auto& id : optimized_frame_ids) {
                DCHECK(keyframes_.contains(id));
                auto& entry = keyframes_.find(id)->second;
                DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
                DCHECK(entry.storage_status != StorageStatus::REQUESTED);
                entry.optimizable_pose = entry.pose;
                global_problem->addFrame(entry.optimizable_pose, entry.calibration);
                if (entry.has_dual) {
                    global_problem->createFramePoseConsensusResidual(id, entry.optimizable_pose, entry.dual, entry.reference_pose, entry.cardinality);
                }

                if (id == StateId{reference_frame_id_before, 0}) {
                    global_problem->setFramePoseConstant(entry.optimizable_pose);
                }
            }

            // Add all map point position that are available.
            optimized_map_point_ids = map_.getMapPointIds();
            for (auto itr = optimized_map_point_ids.begin(); itr != optimized_map_point_ids.end();) {
                const auto& id = *itr;
                DCHECK(map_points_.contains(id));
                auto& entry = map_points_.find(id)->second;
                DCHECK(entry.storage_status != StorageStatus::EMPTY && entry.storage_status != StorageStatus::REMOVED);
                if (entry.storage_status != StorageStatus::REQUESTED) {
                    entry.optimizable_position = entry.position;
                    global_problem->addMapPoint(entry.optimizable_position);
                    if (entry.has_dual) {
                        global_problem->createMapPointConsensusResidual(id, entry.optimizable_position, entry.dual, entry.cardinality);
                    }
                    ++itr;
                } else {
                    itr = optimized_map_point_ids.erase(itr);
                }
            }

            // Add all residuals.
            observations = map_.getObservations();
            for (auto itr = observations.begin(); itr != observations.end();) {
                const auto& key = *itr;
                if (optimized_map_point_ids.contains(key.map_point_id)) {
                    auto& frame_entry = keyframes_.find(key.frame_id)->second;
                    auto& map_point_entry = map_points_.find(key.map_point_id)->second;
                    const auto& keypoint = frame_entry.image_data[key.camera_index].keypoints[key.keypoint_index];
                    global_problem->createReprojectionResidual(key, map_point_entry.optimizable_position,
                        frame_entry.optimizable_pose, frame_entry.calibration, Pixel{keypoint.pt.x, keypoint.pt.y});
                    ++itr;
                } else {
                    itr = observations.erase(itr);
                }
            }

            // Log size of collected map.
            map_size_log_.insert({ros::Time::now().toNSec(), {optimized_frame_ids.size(), optimized_map_point_ids.size()}});
        }

        // Solve
        auto [is_converged, num_iter] = global_problem->solve(max_num_iterations_);

        // Identify outliers.
        std::set<ObservationKey> outliers;
        if (is_converged) {
            auto storage_lock = mutex_handler_.lockLocalStorage();
            for (auto itr = observations.begin(); itr != observations.end();) {
                const auto& key = *itr;
                if (map_.hasObservation(key)) {
                    auto& map_point_entry = map_points_.find(key.map_point_id)->second;
                    if (!map_point_entry.has_dual) {
                        auto& frame_entry = keyframes_.find(key.frame_id)->second;
                        const Pixel error = global_problem->evaluateReprojectionResidual(key, map_point_entry.optimizable_position,
                            frame_entry.optimizable_pose, frame_entry.calibration);
                        if (error.norm() > outlier_threshold_) {
                            global_problem->removeReprojectionResidual(key);
                            outliers.insert(key);
                        }
                    }
                    ++itr;
                } else {
                    itr = observations.erase(itr);
                }
            }
        }

        if (!outliers.empty()) {
            // DLOG(INFO) << "Identified " << outliers.size() << " outlier observations.";
            auto [convergence_flag, niter] = global_problem->solve(max_num_iterations_);
            num_iter = niter;
            is_converged = convergence_flag;
        }

        // Check change in drift reference.
        tracking_lock.lock();
        drift_reference_id = optimized_frame_ids.contains(tracking_reference_) ? tracking_reference_ : drift_reference_id;
        tracking_lock.unlock();

        // Propagate solution.
        auto keyframe_update_messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::KEYFRAME_UPDATES>>>();
        auto map_point_update_messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_UPDATES>>>();
        SE3 T_drift;
        {
            auto storage_lock = mutex_handler_.lockLocalStorage();

            // Detect a change in reference frame id.
            auto reference_frame_id_after = world_frame_id_;
            auto frame_transformation = SE3::Identity();
            if (reference_frame_id_before != reference_frame_id_after) {
                // DLOG(INFO) << "Frame transformation " << reference_frame_id_before << " --> " << reference_frame_id_after << " in optimizer.";
                frame_transformation = alignment_->getTransformationToWorld(reference_frame_id_before);
            }

            // Calculate drift.
            DCHECK(keyframes_.contains(drift_reference_id) && optimized_frame_ids.contains(drift_reference_id));
            const auto& reference_entry = keyframes_.find(drift_reference_id)->second;
            DCHECK(reference_entry.storage_status == StorageStatus::ACTIVE || reference_entry.storage_status == StorageStatus::OPTIMIZED);
            const auto& drift_reference_pose_before = reference_entry.pose;
            const auto drift_reference_pose_after = frame_transformation.groupPlus(reference_entry.optimizable_pose);
            T_drift = drift_reference_pose_after.groupPlus(drift_reference_pose_before.groupInverse());

            // Propagate optimized keyframe poses.
            for (const auto& id : optimized_frame_ids) {
                DCHECK(keyframes_.contains(id));
                auto& entry = keyframes_.find(id)->second;
                DCHECK(entry.storage_status != StorageStatus::EMPTY);
                // const auto diff_norm = entry.pose.groupInverse().groupPlus(entry.optimizable_pose).norm();
                entry.pose = frame_transformation.groupPlus(entry.optimizable_pose);
                if (entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED) {
                    entry.storage_status = StorageStatus::OPTIMIZED;
                    if (entry.has_dual /*&& diff_norm > 1e-4*/) {
                        auto itr = keyframe_update_messages.find(id.first);
                        if (itr == keyframe_update_messages.end()) {
                            itr = keyframe_update_messages.insert({id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::KEYFRAME_UPDATES>>()}).first;
                        }
                        itr->second->keyframe_updates.emplace_back(id, entry.pose);
                    } else {
                    }
                } else {
                    // DLOG(INFO) << "Optimized KF " << id << " has now status " << static_cast<int>(entry.storage_status);
                }
            }

            // Propagate optimized map point positions.
            for (const auto& id : optimized_map_point_ids) {
                DCHECK(map_points_.contains(id));
                auto& entry = map_points_.find(id)->second;
                DCHECK(entry.storage_status != StorageStatus::EMPTY);
                // const auto diff_norm = (entry.position - entry.optimizable_position).norm();
                entry.position = frame_transformation.vectorPlus(entry.optimizable_position);
                if (entry.storage_status == StorageStatus::ACTIVE || entry.storage_status == StorageStatus::OPTIMIZED) {
                    entry.storage_status = StorageStatus::OPTIMIZED;
                    if (entry.has_dual /*&& diff_norm > 1e-4*/) {
                        auto itr = map_point_update_messages.find(id.first);
                        if (itr == map_point_update_messages.end()) {
                            itr = map_point_update_messages.insert({id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_UPDATES>>()}).first;
                        }
                        itr->second->map_point_updates.emplace_back(id, entry.position);
                    }
                }
            }
        }

        applyDriftCorrection(T_drift);

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

        requestKeyframeRemovals();
        triggerVisualization();

        const auto time_after = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> diff = time_after - time_before;
        DLOG(INFO) << "Optimization took " << diff.count() << " us";
        // const auto rate_us = 4e6;
        if (diff.count() < optimizer_rate_us_) {
            DLOG(INFO) << "Optimizer sleeping for " << static_cast<int>(optimizer_rate_us_ - diff.count()) << " microseconds.";
            usleep(static_cast<int>(optimizer_rate_us_ - diff.count()));
        }

        lock.lock();
        optimization_notified_ = optimization_notified_ || !is_converged;
        lock.unlock();
    }
}

} // namespace deco
