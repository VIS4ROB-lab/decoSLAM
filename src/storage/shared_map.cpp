//
// Created by philipp on 05.10.22.
//

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "map/frame.hpp"
#include "map/map_point.hpp"
#include "messages/local_storage_messages.hpp"
#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/alignment.hpp"
#include "system/communicator.hpp"

namespace deco {
namespace {
auto applyStorageAction(Map& map, const ObservationKey& key, const StorageAction& storage_action) -> void {
    switch (storage_action) {
        case StorageAction::ADD:
            if (!map.hasObservationForCamera(key.frame_id, key.map_point_id, key.camera_index)) {
                map.addObservation(key);
            }
            break;
        case StorageAction::REMOVE:
            map.removeObservation(key);
            break;
        default:
            LOG(FATAL) << "Unknown storage action.";
    }
}
} // namespace

SharedMap::SharedMap(const Id& id, StorageMutexHandler& mutex_handler)
    : id_{id},
      reference_frame_id_{id},
      mutex_handler_{mutex_handler} {}

SharedMap::~SharedMap() = default;

auto SharedMap::readParameters(const YAML::Node& node) -> void {
    admm_gamma_ = node["admm_gamma"].as<Scalar>();
    admm_stepsize_ = node["admm_stepsize"].as<Scalar>();
    dual_parameter_tolerance_ = node["dual_parameter_tolerance"].as<Scalar>();
}

auto SharedMap::writeLogs(const FilePath& output_path) -> void {
    const auto separator = ", ";
    const auto filepath = output_path / "pose_id_map.csv";
    std::ofstream file_stream;
    file_stream.open(filepath);

    // Header.
    file_stream << "timestamp" << separator
                << "id"
                << "\n";

    for (const auto& [id, keyframe] : keyframes_) {
        file_stream << keyframe.first->timestamp << separator
                    << id
                    << "\n";
    }
    file_stream.close();
}

auto SharedMap::setLocalStorage(LocalStorage* local_storage) -> void {
    local_storage_ = local_storage;
}

auto SharedMap::setCommunicator(Communicator* communicator) -> void {
    communicator_ = communicator;
}

auto SharedMap::setAlignment(Alignment* alignment) -> void {
    alignment_ = alignment;
}

auto SharedMap::submitMessage(std::unique_ptr<MessageBase>&& message) -> void {
    DCHECK(message->type == MessageType::SHARED_MAP);
    auto& shared_map_message = dynamic_cast<SharedMapMessageBase&>(*message);

    switch (shared_map_message.subtype) {
        case SharedMapMessageType::OBSERVATION_STORAGE:
            observationStorage_(shared_map_message);
            break;
        case SharedMapMessageType::EDGE_UPDATES:
            updateEdges_(shared_map_message);
            break;
        case SharedMapMessageType::MAP_POINT_MERGE_PROPOSALS:
            handleMergeProposals_(shared_map_message);
            break;
        case SharedMapMessageType::MAP_POINT_MERGE_REQUESTS:
            handleMergeRequests_(shared_map_message);
            break;
        case SharedMapMessageType::MAP_POINT_MERGE_CONFIRMATIONS:
            handleMergeConfirmations_(shared_map_message);
            break;
        case SharedMapMessageType::KEYFRAME_REQUESTS:
            handleKeyframeRequests_(shared_map_message);
            break;
        case SharedMapMessageType::KEYFRAME_SYNCHRONIZATIONS:
            handleKeyframeSynchronizations_(shared_map_message);
            break;
        case SharedMapMessageType::KEYFRAME_UPDATES:
            updateKeyframes_(shared_map_message);
            break;
        case SharedMapMessageType::KEYFRAME_DELETION_REQUESTS:
            tryRemoveKeyframeOwner_(shared_map_message);
            break;
        case SharedMapMessageType::MAP_POINT_REQUESTS:
            handleMapPointRequests_(shared_map_message);
            break;
        case SharedMapMessageType::MAP_POINT_SYNCHRONIZATIONS:
            handleMapPointSynchronizations_(shared_map_message);
            break;
        case SharedMapMessageType::MAP_POINT_UPDATES:
            updateMapPoints_(shared_map_message);
            break;
        case SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS:
            tryRemoveMapPointOwner_(shared_map_message);
            break;
        default:
            LOG(FATAL) << "Unknown message type.";
    }
}

auto SharedMap::applyTransformation(const Id& new_reference_frame_id, const SE3& transformation) -> void {
    DCHECK(reference_frame_id_ != new_reference_frame_id) << "Identical reference frame ids."
                                                          << reference_frame_id_ << " to " << new_reference_frame_id;

    // Transform keyframe storage.
    for (auto& [_, data] : keyframes_) {
        auto& [keyframe, entry] = data;

        // Transform assigned pose and reference frame id.
        keyframe->se3_world_body = transformation.groupPlus(keyframe->se3_world_body);
        keyframe->reference_frame_id = new_reference_frame_id;

        const auto reference_after = transformation.groupPlus(entry.reference_state);

        // Transform duals using estimates.
        auto& dual_table = entry.dual_table;
        for (auto& [id_pair, tangent] : dual_table) {
            DCHECK(entry.estimates.contains(id_pair.first)) << "Inconsistent estimates.";
            const auto& estimate_before = entry.estimates.find(id_pair.first)->second;
            const auto estimate_after = transformation.groupPlus(estimate_before);
            const auto tangent_before = (entry.reference_state.groupInverse().groupPlus(estimate_before)).toTangent();
            const auto tangent_after = (reference_after.groupInverse().groupPlus(estimate_after)).toTangent();
            tangent = tangent + admm_gamma_ * (tangent_before - tangent_after);
        }

        // Transform reference state.
        entry.reference_state = reference_after;

        // Transform estimates.
        for (auto& [owner_id, pose] : entry.estimates) {
            DCHECK(entry.owners.contains(owner_id)) << "Estimate for missing owner " << owner_id;
            pose = transformation.groupPlus(pose);
        }

        // Assign the new reference frame id.
        entry.reference_frame_id = new_reference_frame_id;
    }

    for (auto& [_, data] : map_points_) {
        auto& [map_point, entry] = data;

        // Transform assigned position and reference frame id.
        map_point->position = transformation.vectorPlus(map_point->position);
        map_point->reference_frame_id = new_reference_frame_id;

        // Transform reference position.
        entry.reference_state = transformation.vectorPlus(entry.reference_state);

        // Transform duals using estimates.
        auto& dual_table = entry.dual_table;
        for (auto& [id_pair, tangent] : dual_table) {
            DCHECK(entry.estimates.contains(id_pair.first)) << "Inconsistent estimates.";
            const auto& estimate = entry.estimates.find(id_pair.first)->second;
            tangent = transformation.rotation().vectorPlus(tangent) - admm_gamma_ * transformation.translation();
        }

        // Transform estimates.
        for (auto& [owner_id, position] : entry.estimates) {
            DCHECK(entry.owners.contains(owner_id)) << "Estimate for missing owner " << owner_id;
            position = transformation.vectorPlus(position);
        }
    }

    DLOG(INFO) << "Changed world frame id in shared map from " << reference_frame_id_ << " to " << new_reference_frame_id;
    reference_frame_id_ = new_reference_frame_id;
}

auto SharedMap::storeKeyframe(std::unique_ptr<Frame>&& frame) -> Frame* {
    // Force consistency with local storage and alignment.
    auto lock = mutex_handler_.lockAllStorages();

    // Check id.
    const auto frame_id = frame->id;
    DCHECK(frame_id.first == id_) << "Id mismatch.";

    // Add a node to the covisibility graph.
    covisibility_graph_.addNode(frame_id);

    // Transform the frame pose into the correct reference frame if necessary.
    if (frame->reference_frame_id != this->reference_frame_id_) {
        // DLOG(INFO) << "Transforming keyframe " << frame_id << " from " << frame->reference_frame_id << " into " << this->reference_frame_id_;
        const auto transformation = alignment_->getTransformationToWorld(frame->reference_frame_id);
        frame->se3_world_body = transformation.groupPlus(frame->se3_world_body);
        frame->reference_frame_id = reference_frame_id_;
    }

    // Create the storage entry.
    auto entry = StorageEntry<SE3>{
        .status = Status::ACTIVE,
        .reference_frame_id = frame->reference_frame_id, // TODO: Avoid this copy.
        .owners = std::set<Id>{id_},
        .requesters = {},
        .estimates = {},
        .reference_state = frame->se3_world_body, // TODO: Avoid this copy.
        .dual_table = {}};
    entry.estimates.insert({id_, frame->se3_world_body});

    // Propagate to local storage.
    local_storage_->addKeyframePose(frame_id, frame->se3_world_body);
    local_storage_->addConstantKeyframeData(frame_id, frame->image_data, frame->calibration);

    // Store KF and entry.
    auto [itr, is_inserted] = keyframes_.insert({frame_id, std::make_pair(std::move(frame), std::move(entry))});
    DCHECK(is_inserted) << "Tried to store frame " << frame_id << " twice.";
    return itr->second.first.get();
}

auto SharedMap::getKeyframe(const StateId& frame_id) -> Frame* {
    auto lock = mutex_handler_.lockSharedMap();
    DCHECK(keyframes_.contains(frame_id));
    return keyframes_.find(frame_id)->second.first.get();
}

auto SharedMap::storeMapPoints(std::vector<std::unique_ptr<MapPoint>>& map_points) -> void {
    // Force consistency with local storage and alignment.
    auto lock = mutex_handler_.lockAllStorages();

    for (auto& map_point : map_points) {
        const auto map_point_id = map_point->id;

        // Check id.
        DCHECK(map_point_id.first == id_) << "Id mismatch.";

        // Transform the map point position into the correct reference frame if necessary.
        if (map_point->reference_frame_id != reference_frame_id_) {
            // DLOG(INFO) << "Transforming map point " << map_point_id << " from " << map_point->reference_frame_id << " into " << this->reference_frame_id_;
            const auto transformation = alignment_->getTransformationToWorld(map_point->reference_frame_id);
            map_point->position = transformation.vectorPlus(map_point->position);
            map_point->reference_frame_id = reference_frame_id_;
        }

        // Create the storage entry.
        auto entry = StorageEntry<Position>{
            .status = Status::ACTIVE,
            .reference_frame_id = map_point->reference_frame_id,
            .owners = std::set<Id>{id_},
            .requesters = {},
            .estimates = {},
            .reference_state = map_point->position,
            .dual_table = {}};
        entry.estimates.insert({id_, map_point->position});

        // Propagate to local storage.
        local_storage_->addMapPointPosition(map_point_id, map_point->position);
        local_storage_->addConstantMapPointData(map_point_id, map_point->getDescriptor());

        // Store map point and entry.
        auto [itr, is_inserted] = map_points_.insert({map_point_id, std::make_pair(std::move(map_point), std::move(entry))});
        DCHECK(is_inserted) << "Tried to store map points " << map_point_id << " twice.";
    }
}

auto SharedMap::observationStorage(const std::set<ObservationKey>& keys, const StorageAction& storage_action) -> void {
    auto messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE>>>();
    for (const auto& key : keys) {
        const auto& map_point_id = key.map_point_id;
        auto itr = messages.find(map_point_id.first);
        if (itr == messages.end()) {
            itr = messages.insert({map_point_id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE>>()}).first;
        }
        itr->second->keys.emplace_back(key, storage_action);
    }

    for (auto& [id, message] : messages) {
        message->receiver_id = id;
        communicator_->send(std::move(message));
    }
}

auto SharedMap::getMap() const -> const Map* {
    return &map_;
}

auto SharedMap::isCovisible(const StateId& frame_id_A, const StateId& frame_id_B) -> bool {
    return covisibility_graph_.hasEdge(frame_id_A, frame_id_B, kMinCovisibilityWeight);
}

auto SharedMap::mergeMapPoints(const std::vector<std::pair<StateId, StateId>>& mergeable_pairs) -> void {
    auto merge_proposals = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_PROPOSALS>>>();
    for (const auto& [remaining_id, merged_id] : mergeable_pairs) {
        const auto receiver_id = remaining_id.first;
        auto itr = merge_proposals.find(receiver_id);
        if (itr == merge_proposals.end()) {
            itr = merge_proposals.insert({receiver_id, std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_PROPOSALS>>()}).first;
        }
        itr->second->mergeable_pairs.insert({remaining_id, merged_id});
    }

    for (auto& [id, merge_message] : merge_proposals) {
        merge_message->receiver_id = id;
        communicator_->send(std::move(merge_message));
    }
}

auto SharedMap::observationStorage_(SharedMapMessageBase& message) -> void {
    // Force consistency with local storage.
    auto lock = mutex_handler_.lockAllStorages();

    // Downcast message.
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE>&>(message);

    // Preprocess and split set of observations.
    const auto [map_point_observations, keyframe_only_observations] = preprocessObservations_(query.keys);
    query.keys.clear();

    auto relay_messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE>>>();
    auto edge_update_messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::EDGE_UPDATES>>>();
    auto local_storage_updates = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::OBSERVATION_STORAGE>>>();
    for (const auto& [key, storage_action] : map_point_observations) {
        const auto& [updated_edges, increment] = processMapPointObservation_(key, storage_action);
        for (const auto& [id_1, id_2] : updated_edges) {
            auto itr = edge_update_messages.find(id_1.first);
            if (itr == edge_update_messages.end()) {
                itr = edge_update_messages.insert({id_1.first, std::make_unique<SharedMapMessage<SharedMapMessageType::EDGE_UPDATES>>()}).first;
            }

            auto& edges = itr->second->edge_updates;

            auto edge_itr = edges.find({id_1, id_2});
            if (edge_itr == edges.end()) {
                edge_itr = edges.insert({{id_1, id_2}, 0}).first;
            }
            edge_itr->second += increment;
        }

        // Relay messages to keyframe coordinators.
        if (key.frame_id.first != id_) {
            auto itr = relay_messages.find(key.frame_id.first);
            if (itr == relay_messages.end()) {
                itr = relay_messages.insert({key.frame_id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE>>()}).first;
            }
            itr->second->keys.emplace_back(key, storage_action);
        } else {
            DCHECK(covisibility_graph_.hasNode(key.frame_id));
            auto frame_owners = keyframes_.find(key.frame_id)->second.second.owners;
            for (const auto& id : frame_owners) {
                auto itr = local_storage_updates.find(id);
                if (itr == local_storage_updates.end()) {
                    itr = local_storage_updates.insert({id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::OBSERVATION_STORAGE>>()}).first;
                }
                itr->second->keys.insert({key, storage_action});
            }
        }
    }

    for (const auto& [key, storage_action] : keyframe_only_observations) {
        applyStorageAction(map_, key, storage_action);
        DCHECK(covisibility_graph_.hasNode(key.frame_id));
        auto frame_owners = keyframes_.find(key.frame_id)->second.second.owners;
        for (const auto& id : frame_owners) {
            auto itr = local_storage_updates.find(id);
            if (itr == local_storage_updates.end()) {
                itr = local_storage_updates.insert({id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::OBSERVATION_STORAGE>>()}).first;
            }
            itr->second->keys.insert({key, storage_action});
        }
    }

    // Send out all messages.
    for (auto& [id, relayed_message] : relay_messages) {
        relayed_message->receiver_id = id;
        communicator_->send(std::move(relayed_message));
    }

    for (auto& [id, update_message] : edge_update_messages) {
        update_message->receiver_id = id;
        communicator_->send(std::move(update_message));
    }

    for (auto& [id, update_message] : local_storage_updates) {
        update_message->receiver_id = id;
        communicator_->send(std::move(update_message));
    }
}

auto SharedMap::preprocessObservations_(std::vector<ObservationStore>& keys) -> std::pair<std::vector<ObservationStore>, std::vector<ObservationStore>> {
    std::vector<ObservationStore> map_point_observations, keyframe_only_observations;

    auto relay_messages = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE>>>();
    for (auto& [key, storage_action] : keys) {
        const bool process_keyframe = key.frame_id.first == id_;
        const bool process_map_point = key.map_point_id.first == id_;
        DCHECK(process_keyframe || process_map_point) << id_ << " is the wrong storage location for " << key;

        // Lambda to check for repeated storage actions.
        auto is_repeated_action = [&](const ObservationKey& query_key, const StorageAction& query_action) -> bool {
            const bool is_observed_in_camera = map_.hasObservationForCamera(query_key.frame_id, query_key.map_point_id, query_key.camera_index);
            const bool exists = map_.hasObservation(query_key);
            return (is_observed_in_camera && query_action == StorageAction::ADD) || (!exists && query_action == StorageAction::REMOVE);
        };

        if (!is_repeated_action(key, storage_action)) {
            if (process_keyframe && !process_map_point) {
                keyframe_only_observations.emplace_back(key, storage_action);
            } else {
                DCHECK(map_points_.contains(key.map_point_id)) << key;
                const auto& [_, entry] = map_points_.find(key.map_point_id)->second;
                switch (entry.status) {
                    case Status::ACTIVE:
                    case Status::MERGING:
                        map_point_observations.emplace_back(key, storage_action);
                        break;
                    case Status::MERGED: {
                        if (storage_action == StorageAction::ADD) {
                            // Remap to new map point.
                            const auto id_before = key.map_point_id;
                            key.map_point_id = map_point_merges_.find(key.map_point_id)->second;
                            // DLOG(INFO) << "Remap " << id_before << " --> " << key.map_point_id;

                            // New map point is not stored here -> relay additions.
                            if (key.map_point_id.first != id_) {
                                auto itr = relay_messages.find(key.map_point_id.first);
                                if (itr == relay_messages.end()) {
                                    itr = relay_messages.insert({key.map_point_id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::OBSERVATION_STORAGE>>()}).first;
                                }
                                itr->second->keys.emplace_back(key, storage_action);
                            } else {
                                map_point_observations.emplace_back(key, storage_action);
                            }
                        } else {
                            map_point_observations.emplace_back(key, storage_action);
                        }
                    } break;
                    case Status::REMOVED:
                        break;
                }
            }
        } else {
            /*DLOG(INFO) << "Trying to repeatedly " << (storage_action == StorageAction::ADD ? " add " : " remove ")
                       << " observation " << key;*/
        }
    }

    // DLOG(INFO) << "Triage into " << keyframe_only_observations.size() << " KF only and " << map_point_observations.size() << " MP and KF observations.";

    // Send relay messages.
    for (auto& [id, relayed_message] : relay_messages) {
        DCHECK(id != id_) << " Relays cannot be internal.";
        relayed_message->receiver_id = id;
        communicator_->send(std::move(relayed_message));
    }

    return {map_point_observations, keyframe_only_observations};
}

auto SharedMap::processMapPointObservation_(const ObservationKey& key, const StorageAction& storage_action) -> std::pair<std::set<std::pair<StateId, StateId>>, int> {
    const auto& map_point_id = key.map_point_id;
    const auto& keyframe_id = key.frame_id;

    // Apply the storage action.
    auto observing_frames_before = map_.getObservingFrames(map_point_id);
    applyStorageAction(map_, key, storage_action);
    auto observing_frames_after = map_.getObservingFrames(map_point_id);

    // Detect covisibility changes.
    const auto increment_covisibility = !observing_frames_before.contains(keyframe_id) && observing_frames_after.contains(keyframe_id);
    const auto decrement_covisibility = observing_frames_before.contains(keyframe_id) && !observing_frames_after.contains(keyframe_id);
    const auto change_covisibility = increment_covisibility || decrement_covisibility;
    const auto increment = increment_covisibility ? 1 : (decrement_covisibility ? -1 : 0);

    std::set<std::pair<StateId, StateId>> edge_updates;
    if (change_covisibility) {
        auto observing_frames = increment_covisibility ? observing_frames_before : observing_frames_after; // else -> decrement_covisibility is true.
        for (const auto& observing_id : observing_frames) {
            edge_updates.insert({keyframe_id, observing_id});
            edge_updates.insert({observing_id, keyframe_id});
        }
    }

    return {edge_updates, increment};
}

auto SharedMap::updateEdges_(SharedMapMessageBase& message) -> void {
    // Force consistency with local storage.
    auto lock = mutex_handler_.lockAllStorages();

    // Downcast message.
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::EDGE_UPDATES>&>(message);

    auto edge_updates = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::EDGE_UPDATES>>>();

    // Apply weight changes.
    for (const auto& [edge, weight_increment] : query.edge_updates) {
        DCHECK(edge.first.first == id_);
        if (weight_increment != 0) {
            DCHECK(covisibility_graph_.hasNode(edge.first));
            if (!covisibility_graph_.hasEdge(edge.first, edge.second)) {
                DCHECK(weight_increment > 0) << "Negative weight on zero edge.";
                covisibility_graph_.addEdge(edge.first, edge.second, weight_increment);
                // DLOG(INFO) << id_ << ": Add edge " << edge.first << " --> " << edge.second << " with weight " << weight_increment;
            } else {
                auto new_weight = covisibility_graph_.incrementEdge(edge.first, edge.second, weight_increment);
                // DLOG(INFO) << id_ << ": Change weight of edge " << edge.first << " --> " << edge.second << " to " << new_weight;
            }
        } else {
            // DLOG(INFO) << "Zero weight increment.";
        }

        // Propagate to owners.
        auto owners = keyframes_.find(edge.first)->second.second.owners;
        for (const auto& id : owners) {
            auto itr = edge_updates.find(id);
            if (itr == edge_updates.end()) {
                itr = edge_updates.insert({id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::EDGE_UPDATES>>()}).first;
            }
            auto& edges = itr->second->edge_updates;
            auto [_, inserted] = edges.insert({edge, weight_increment});
            DCHECK(inserted);
        }
    }

    // Send out messages.
    for (auto& [id, update] : edge_updates) {
        update->receiver_id = id;
        communicator_->send(std::move(update));
    }
}

auto SharedMap::handleMergeProposals_(SharedMapMessageBase& message) -> void {
    // Downcast message.
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_PROPOSALS>&>(message);
    auto responses = std::map<Id, std::unique_ptr<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_REQUESTS>>>();
    // DLOG(INFO) << "Got " << query.mergeable_pairs.size() << " merge proposals.";
    for (const auto& [remaining_id, merged_id] : query.mergeable_pairs) {
        DCHECK(remaining_id.first == id_) << id_ << " is the wrong receiver for merge proposal on " << remaining_id;
        auto& [_, entry] = map_points_.find(remaining_id)->second;
        if (entry.status == Status::ACTIVE) {
            entry.status = Status::MERGING;
            auto itr = responses.find(merged_id.first);
            if (itr == responses.end()) {
                itr = responses.insert({merged_id.first, std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_REQUESTS>>()}).first;
            }
            itr->second->mergeable_pairs.insert({remaining_id, merged_id});
        } else {
            // DLOG(INFO) << "Cannot merge " << merged_id << " into " << remaining_id << " since " << remaining_id << " is "
            //            << (entry.status == Status::MERGED ? "merged" : "removed");
        }
    }

    for (auto& [id, response] : responses) {
        response->receiver_id = id;
        communicator_->send(std::move(response));
    }
}

auto SharedMap::handleMergeRequests_(SharedMapMessageBase& message) -> void {
    auto lock = mutex_handler_.lockAllStorages();
    // Downcast message.
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_REQUESTS>&>(message);
    auto response = std::make_unique<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_CONFIRMATIONS>>();
    response->receiver_id = query.sender_id;
    // DLOG(INFO) << "Got " << query.mergeable_pairs.size() << " merge requests.";
    // Check for repeated merges, construct observations to add / remove.
    std::set<ObservationKey> remove_observations, add_observations;
    for (const auto& [remaining_id, merged_id] : query.mergeable_pairs) {
        auto& [_, entry] = map_points_.find(merged_id)->second;
        if (entry.status == Status::ACTIVE && entry.requesters.empty()) {
            auto old_observations = map_.getObservationsForMapPoint(merged_id);
            remove_observations.insert(old_observations.begin(), old_observations.end());
            for (const auto& old_key : old_observations) {
                auto new_key = old_key;
                new_key.map_point_id = remaining_id; // Switch the map point id to the remaining one.
                add_observations.insert(new_key);
            }
            entry.status = Status::MERGED;
            map_point_merges_.insert({merged_id, remaining_id});
            // DLOG(INFO) << "Merge " << merged_id << " into " << remaining_id;
        } else {
            /*DLOG(INFO) << "Cannot merge " << merged_id << " into " << remaining_id << " since " << merged_id << " has status "
                       << static_cast<int>(entry.status);*/
        }
        DCHECK(remaining_id.first == response->receiver_id);
        response->map_point_ids.insert(remaining_id);
    }

    // Replace observations of merged points with observation of remaining ones.
    communicator_->send(std::move(response));
    observationStorage(remove_observations, StorageAction::REMOVE);
    observationStorage(add_observations, StorageAction::ADD);
}

auto SharedMap::handleMergeConfirmations_(SharedMapMessageBase& message) -> void {
    auto lock = mutex_handler_.lockAllStorages();
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::MAP_POINT_MERGE_CONFIRMATIONS>&>(message);
    for (const auto& id : query.map_point_ids) {
        auto& [_, entry] = map_points_.find(id)->second;
        DCHECK(entry.status == Status::MERGING);
        entry.status = Status::ACTIVE;
    }
    // DLOG(INFO) << "Moved " << query.map_point_ids.size() << " MPs from merging to active again.";
}

auto SharedMap::handleKeyframeRequests_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::KEYFRAME_REQUESTS>&>(message);
    auto lock = mutex_handler_.lockAllStorages();

    // DLOG(INFO) << id_ << ": Fetching " << query.keyframe_ids.size() << " keyframes for " << query.sender_id;

    // Initialize new dual messages
    auto new_dual_messages = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::NEW_KEYFRAME_DUALS>>>();
    std::set<std::pair<StateId, bool>> synchronized_ids;
    for (const auto& [keyframe_id, need_constant_data] : query.keyframe_ids) {
        auto& [keyframe, entry] = keyframes_.find(keyframe_id)->second;
        const auto& reference_pose = entry.reference_state;
        auto& owners = entry.owners;
        DCHECK(!owners.empty());
        if (owners.size() > 1) {
            synchronized_ids.insert({keyframe_id, need_constant_data});
        } else {
            if (entry.requesters.empty()) { // Send new dual message
                auto owner_id = *owners.begin();
                auto itr = new_dual_messages.find(owner_id);
                if (itr == new_dual_messages.end()) {
                    itr = new_dual_messages.insert({owner_id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::NEW_KEYFRAME_DUALS>>()}).first;
                }
                itr->second->duals.emplace_back(keyframe_id, reference_pose);
            }
        }
        entry.requesters.insert({query.sender_id, need_constant_data});
    }

    // Send new dual messages.
    for (auto& [id, new_dual_message] : new_dual_messages) {
        new_dual_message->receiver_id = id;
        // DLOG(INFO) << id_ << ": Synchronize " << new_dual_message->duals.size() << " keyframes from " << id;
        communicator_->send(std::move(new_dual_message));
    }

    if (!synchronized_ids.empty()) {
        sendKeyframes_(query.sender_id, synchronized_ids);
    }
}

auto SharedMap::handleKeyframeSynchronizations_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::KEYFRAME_SYNCHRONIZATIONS>&>(message);
    auto lock = mutex_handler_.lockAllStorages();

    std::map<Id, std::set<std::pair<StateId, bool>>> requests_per_agent;
    for (const auto& [keyframe_id, pose] : query.keyframe_poses_) {
        auto& [keyframe, entry] = keyframes_.find(keyframe_id)->second;
        DCHECK(entry.owners.contains(query.sender_id));
        entry.estimates.find(query.sender_id)->second = pose;

        for (const auto& [id, need_constant_data] : entry.requesters) {
            auto itr = requests_per_agent.find(id);
            if (itr == requests_per_agent.end()) {
                itr = requests_per_agent.insert({id, {}}).first;
            }
            itr->second.insert({keyframe_id, need_constant_data});
        }
    }
    for (const auto& [id, requests] : requests_per_agent) {
        sendKeyframes_(id, requests);
    }
}

auto SharedMap::sendKeyframes_(const Id& new_owner_id, const std::set<std::pair<StateId, bool>>& keyframe_requests) -> void {
    // Initialize response
    auto response = std::make_unique<LocalStorageMessage<LocalStorageMessageType::NEW_KEYFRAMES>>();
    response->receiver_id = new_owner_id;
    response->world_frame_id = reference_frame_id_;

    for (const auto& [keyframe_id, need_constant_data] : keyframe_requests) {
        DCHECK(keyframes_.contains(keyframe_id));

        // Fetch keyframe pose, owners and dual table.
        auto& [keyframe, entry] = keyframes_.find(keyframe_id)->second;
        DCHECK(entry.requesters.contains(new_owner_id));
        const auto& reference_pose = entry.reference_state;
        auto& owners = entry.owners;
        DCHECK(!owners.empty());

        DCHECK(!entry.estimates.empty());
        const auto& keyframe_pose = entry.estimates.begin()->second; // TODO: Average estimates?
        entry.estimates.insert({new_owner_id, keyframe_pose});
        auto& dual_table = entry.dual_table;

        // Compute initial dual.
        const hyper::Tangent<SE3> initial_dual = Scalar{-1.0} * admm_gamma_ * (reference_pose.groupInverse().groupPlus(keyframe_pose)).toTangent();

        // Introduce dual pairs between all current owners and the new owner.
        for (const auto& id : owners) {
            dual_table.insert({{new_owner_id, id}, initial_dual});
            dual_table.insert({{id, new_owner_id}, initial_dual});
        }

        // Register new owner.
        auto [_, inserted] = owners.insert(new_owner_id);
        DCHECK(inserted);
        entry.requesters.erase(new_owner_id);

        // Copy keyframe data.
        auto image_data = need_constant_data ? keyframe->image_data : std::vector<Frame::ImageData>();
        auto calibration = need_constant_data ? keyframe->calibration : std::vector<Frame::Calibration>();
        response->keyframes.emplace_back(keyframe_id, keyframe_pose, image_data, calibration, initial_dual, reference_pose, static_cast<int>(owners.size() - 1));

        // Fetch observations.
        const auto& observations = map_.getObservationsForFrame(keyframe_id);
        response->observations.insert(observations.begin(), observations.end());

        // Fetch edges.
        for (const auto& [neighbor_id, weight] : covisibility_graph_.getNeighbors(keyframe_id)) {
            response->edge_updates.insert({{keyframe_id, neighbor_id}, weight});
        }
        // DLOG(INFO) << keyframe_id;
    }

    // Send response.
    // DLOG(INFO) << "Send " << response->keyframes.size() << " keyframes to " << response->receiver_id;
    communicator_->send(std::move(response));
}

auto SharedMap::handleMapPointRequests_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::MAP_POINT_REQUESTS>&>(message);
    auto lock = mutex_handler_.lockAllStorages();

    // DLOG(INFO) << id_ << ": Fetching " << query.map_point_ids.size() << " map points for " << query.sender_id;
    // Initialize new dual messages
    auto new_dual_messages = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::NEW_MAP_POINT_DUALS>>>();
    std::set<std::pair<StateId, bool>> synchronized_ids;
    std::set<StateId> refused_ids;
    for (const auto& [map_point_id, need_constant_data] : query.map_point_ids) {
        auto& [map_point, entry] = map_points_.find(map_point_id)->second;
        auto& owners = entry.owners;
        switch (entry.status) {
            case Status::ACTIVE:
            case Status::MERGING: {
                DCHECK(!owners.empty());
                if (owners.size() > 1) {
                    synchronized_ids.insert({map_point_id, need_constant_data});
                } else {
                    if (entry.requesters.empty()) { // Send new dual message
                        auto owner_id = *owners.begin();
                        auto itr = new_dual_messages.find(owner_id);
                        if (itr == new_dual_messages.end()) {
                            itr = new_dual_messages.insert({owner_id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::NEW_MAP_POINT_DUALS>>()}).first;
                        }
                        itr->second->duals.emplace_back(map_point_id);
                    }
                }
                entry.requesters.insert({query.sender_id, need_constant_data});
            } break;
            case Status::MERGED: {
                refused_ids.insert(map_point_id);
                // DLOG(INFO) << "Request of merged map point " << map_point_id << " by " << query.sender_id;
            } break;
            case Status::REMOVED: {
                refused_ids.insert(map_point_id);
                // DLOG(INFO) << "Request of removed map point " << map_point_id << " by " << query.sender_id;
                DCHECK(owners.empty());
                /*if (!owners.empty()) {
                    for (const auto& id : owners) {
                        DLOG(INFO) << id;
                    }
                }*/
            } break;
        }
    }

    // Send new dual messages.
    for (auto& [id, new_dual_message] : new_dual_messages) {
        new_dual_message->receiver_id = id;
        communicator_->send(std::move(new_dual_message));
    }

    if (!synchronized_ids.empty() || !refused_ids.empty()) {
        sendMapPoints_(query.sender_id, synchronized_ids, refused_ids);
    }
}

auto SharedMap::handleMapPointSynchronizations_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::MAP_POINT_SYNCHRONIZATIONS>&>(message);
    auto lock = mutex_handler_.lockAllStorages();

    std::map<Id, std::set<std::pair<StateId, bool>>> requests_per_agent;
    for (const auto& [map_point_id, position] : query.map_point_positions) {
        auto& [map_point, entry] = map_points_.find(map_point_id)->second;
        DCHECK(entry.owners.contains(query.sender_id));
        DCHECK(entry.status == Status::ACTIVE || entry.status == Status::MERGING) << map_point_id << ", status " << static_cast<int>(entry.status);
        DCHECK(entry.estimates.contains(query.sender_id));
        entry.estimates.find(query.sender_id)->second = position;

        for (const auto& [id, need_constant_data] : entry.requesters) {
            auto itr = requests_per_agent.find(id);
            if (itr == requests_per_agent.end()) {
                itr = requests_per_agent.insert({id, {}}).first;
            }
            itr->second.insert({map_point_id, need_constant_data});
        }
    }
    for (const auto& [id, requests] : requests_per_agent) {
        sendMapPoints_(id, requests, {});
    }
}

auto SharedMap::sendMapPoints_(const Id& new_owner_id, const std::set<std::pair<StateId, bool>>& map_point_requests, const std::set<StateId>& refused_ids) -> void {
    auto lock = mutex_handler_.lockAllStorages();

    // Initialize response.
    auto response = std::make_unique<LocalStorageMessage<LocalStorageMessageType::NEW_MAP_POINTS>>();
    response->receiver_id = new_owner_id;
    response->refused_ids = refused_ids;
    response->world_frame_id = reference_frame_id_;

    for (const auto& [map_point_id, need_descriptor] : map_point_requests) {
        // Fetch map point position, owners and dual table.
        auto& [map_point, entry] = map_points_.find(map_point_id)->second;
        auto& owners = entry.owners;
        DCHECK(!owners.empty());
        DCHECK(entry.status == Status::ACTIVE || entry.status == Status::MERGING);
        DCHECK(!entry.estimates.empty());
        const auto& position = entry.estimates.begin()->second; // TODO: Average estimates?
        entry.estimates.insert({new_owner_id, position});

        // Compute initial dual.
        const Position initial_dual = Scalar{-1.0} * admm_gamma_ * position;

        // Introduce dual pairs between all current owners and the new owner.
        auto& dual_table = entry.dual_table;
        for (const auto& id : owners) {
            dual_table.insert({{new_owner_id, id}, initial_dual});
            dual_table.insert({{id, new_owner_id}, initial_dual});
        }

        // Register new owner.
        auto [_, inserted] = owners.insert(new_owner_id);
        DCHECK(inserted);
        DCHECK(entry.requesters.contains(new_owner_id));
        entry.requesters.erase(new_owner_id);

        auto descriptor = need_descriptor ? map_point->getDescriptor() : cv::Mat();
        response->map_points.emplace_back(map_point_id, position, descriptor, initial_dual, static_cast<int>(owners.size() - 1));
    }
    // Send response.
    communicator_->send(std::move(response));
    // DLOG(INFO) << id_ << ": Send " << map_point_requests.size() << " map points for " << new_owner_id << " done.";
}

auto SharedMap::updateKeyframes_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::KEYFRAME_UPDATES>&>(message);

    auto lock = mutex_handler_.lockAllStorages();
    auto dual_update_messages = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_UPDATES>>>();
    for (const auto& [frame_id, pose] : query.keyframe_updates) {
        // Fetch dual table and owner ids.
        auto& [_, entry] = keyframes_.find(frame_id)->second;
        auto& dual_table = entry.dual_table;
        const auto& reference_pose = entry.reference_state;
        const auto& owners = entry.owners;

        // Dual updates.
        auto update_ids = std::set<Id>();
        if (owners.contains(query.sender_id)) {
            // Update estimate.
            auto itr = entry.estimates.find(query.sender_id);
            DCHECK(itr != entry.estimates.end());
            itr->second = pose;

            // Update all duals involving sender
            for (const auto& id : owners) {
                if (id != query.sender_id) {
                    // Fetch dual pair.
                    auto& dual = dual_table.find({query.sender_id, id})->second;
                    const auto& other_dual = dual_table.find({id, query.sender_id})->second;

                    // Compute the step.
                    const auto tangent = (reference_pose.groupInverse().groupPlus(pose)).toTangent();
                    const auto step = admm_stepsize_ * (Scalar{0.5} * (dual + other_dual) + admm_gamma_ * tangent);

                    // Update dual.
                    dual -= step;

                    // Check for convergence.
                    if (step.norm() > (dual.norm() + dual_parameter_tolerance_) * dual_parameter_tolerance_) {
                        update_ids.insert(id);
                        // DLOG(INFO) << step.norm() << " > " << (dual.norm() + dual_parameter_tolerance_) * dual_parameter_tolerance_;
                    }
                }
            }
        } else {
            // DLOG(INFO) << "Discarding pose update on keyframe " << frame_id << " from " << query.sender_id;
        }

        // Send updated dual sums if not converged.
        for (const auto& id : update_ids) {
            hyper::Tangent<SE3> dual_sum = hyper::Tangent<SE3>::Zero();
            for (const auto& owner_id : owners) {
                if (owner_id != id) {
                    const auto& dual = dual_table.find({owner_id, id})->second;
                    dual_sum += dual;
                }
            }

            const auto cardinality = static_cast<int>(owners.size() - 1);

            auto itr = dual_update_messages.find(id);
            if (itr == dual_update_messages.end()) {
                itr = dual_update_messages.insert({id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_UPDATES>>()}).first;
            }
            itr->second->dual_updates.emplace_back(frame_id, dual_sum, reference_pose, cardinality);
        }
    }

    for (auto& [id, update] : dual_update_messages) {
        update->receiver_id = id;
        // DLOG(INFO) << "Send " << update->dual_updates.size() << " keyframe dual updates.";
        communicator_->send(std::move(update));
    }
}

auto SharedMap::updateMapPoints_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::MAP_POINT_UPDATES>&>(message);

    auto lock = mutex_handler_.lockAllStorages();
    auto dual_update_messages = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_UPDATES>>>();
    for (const auto& [map_point_id, position] : query.map_point_updates) {
        // Fetch dual table and owner ids.
        auto& [_, entry] = map_points_.find(map_point_id)->second;
        auto& dual_table = entry.dual_table;
        const auto& owners = entry.owners;

        // Dual updates.
        auto update_ids = std::set<Id>();
        if (owners.contains(query.sender_id)) {
            for (const auto& id : owners) {
                // Update estimate.
                auto itr = entry.estimates.find(query.sender_id);
                DCHECK(itr != entry.estimates.end());
                itr->second = position;

                if (id != query.sender_id) {
                    // Fetch dual pair.
                    auto& dual = dual_table.find({query.sender_id, id})->second;
                    const auto& other_dual = dual_table.find({id, query.sender_id})->second;

                    // Compute the step.
                    const auto step = admm_stepsize_ * (Scalar{0.5} * (dual + other_dual) + admm_gamma_ * position);

                    // Update dual.
                    dual -= step;

                    // Check for convergence.
                    if (step.norm() > (dual.norm() + dual_parameter_tolerance_) * dual_parameter_tolerance_) {
                        update_ids.insert(id);
                        // DLOG(INFO) << step.norm() << " > " << (dual.norm() + parameter_tolerance) * parameter_tolerance;
                    }
                }
            }
        } else {
            // DLOG(INFO) << "Discarding position update on map point " << map_point_id << " from " << query.sender_id;
        }

        // Send updated dual sums if not converged.
        for (const auto& id : update_ids) {
            Position dual_sum = Position::Zero();
            for (const auto& owner_id : owners) {
                if (owner_id != id) {
                    const auto& dual = dual_table.find({owner_id, id})->second;
                    dual_sum += dual;
                }
            }

            const auto cardinality = static_cast<int>(owners.size() - 1);

            auto itr = dual_update_messages.find(id);
            if (itr == dual_update_messages.end()) {
                itr = dual_update_messages.insert({id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_UPDATES>>()}).first;
            }
            itr->second->dual_updates.emplace_back(map_point_id, dual_sum, cardinality);
        }
    }

    for (auto& [id, update] : dual_update_messages) {
        update->receiver_id = id;
        // DLOG(INFO) << "Send " << update->dual_updates.size() << " map point dual updates.";
        communicator_->send(std::move(update));
    }
}

auto SharedMap::tryRemoveKeyframeOwner_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::KEYFRAME_DELETION_REQUESTS>&>(message);

    auto lock = mutex_handler_.lockAllStorages();
    // DLOG(INFO) << id_ << ": Checking removal of " << query.keyframe_ids.size() << " keyframes for " << query.sender_id;
    auto dual_remove_messages = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS>>>();
    auto keyframe_remove_message = std::make_unique<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_REMOVALS>>();
    keyframe_remove_message->receiver_id = query.sender_id;
    for (const auto& frame_id : query.keyframe_ids) {
        auto& [_, entry] = keyframes_.find(frame_id)->second;
        auto& owners = entry.owners;
        auto& dual_table = entry.dual_table;
        if (owners.size() > 1) {
            // Remove estimate.
            entry.estimates.erase(query.sender_id);

            // Remove sender's id and duals.
            owners.erase(query.sender_id);
            for (const auto& id : owners) {
                dual_table.erase({id, query.sender_id});
                dual_table.erase({query.sender_id, id});
            }

            // Inform the last remaining owner.
            if (owners.size() == 1) {
                const auto& remaining_owner_id = *owners.begin();
                auto itr = dual_remove_messages.find(remaining_owner_id);
                if (itr == dual_remove_messages.end()) {
                    itr = dual_remove_messages.insert({remaining_owner_id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::KEYFRAME_DUAL_REMOVALS>>()}).first;
                }
                itr->second->keyframe_ids.insert(frame_id);
            }

            keyframe_remove_message->keyframe_ids.insert({frame_id, true});
            // DLOG(INFO) << frame_id;

        } else {
            keyframe_remove_message->keyframe_ids.insert({frame_id, false});
            // DLOG(INFO) << "Remove of keyframe " << frame_id << " by " << query.sender_id << " failed: Last remaining owner.";
        }
    }

    for (auto& [id, remove_message] : dual_remove_messages) {
        remove_message->receiver_id = id;
        communicator_->send(std::move(remove_message));
    }

    const auto debug_num_removed = keyframe_remove_message->keyframe_ids.size();

    communicator_->send(std::move(keyframe_remove_message));
    // DLOG(INFO) << id_ << ": Removed " << debug_num_removed << " keyframes for " << query.sender_id;
}

auto SharedMap::tryRemoveMapPointOwner_(SharedMapMessageBase& message) -> void {
    auto& query = dynamic_cast<SharedMapMessage<SharedMapMessageType::MAP_POINT_REMOVAL_REQUESTS>&>(message);

    auto lock = mutex_handler_.lockAllStorages();
    // DLOG(INFO) << id_ << ": Checking removal of " << query.map_point_ids.size() << " map points for " << query.sender_id;
    auto dual_remove_messages = std::map<Id, std::unique_ptr<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS>>>();
    auto map_point_remove_message = std::make_unique<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_REMOVALS>>();
    map_point_remove_message->receiver_id = query.sender_id;

    std::set<ObservationKey> observations_to_remove;
    int num_underconstrained_removed = 0;
    std::set<StateId> map_point_ids_to_remove;
    for (const auto& map_point_id : query.map_point_ids) {
        DCHECK(map_points_.contains(map_point_id));
        auto& [_, entry] = map_points_.find(map_point_id)->second;
        auto& owners = entry.owners;
        if (!owners.contains(query.sender_id)) {
            // DLOG(INFO) << "Skip removal of " << map_point_id << " for " << query.sender_id;
            if (entry.requesters.contains(query.sender_id)) {
                // DLOG(INFO) << "Removed " << query.sender_id << " from requesters of " << map_point_id;
                entry.requesters.erase(query.sender_id);
            }
            map_point_remove_message->map_point_ids.insert({map_point_id, true});
            continue;
        }
        if (owners.size() > 1) {
            // Remove estimate.
            entry.estimates.erase(query.sender_id);

            // Remove sender's id and duals.
            auto& dual_table = entry.dual_table;
            owners.erase(query.sender_id);
            for (const auto& id : owners) {
                dual_table.erase({id, query.sender_id});
                dual_table.erase({query.sender_id, id});
            }

            // Inform the last remaining owner.
            if (owners.size() == 1) {
                const auto& remaining_owner_id = *owners.begin();
                auto itr = dual_remove_messages.find(remaining_owner_id);
                if (itr == dual_remove_messages.end()) {
                    itr = dual_remove_messages.insert({remaining_owner_id, std::make_unique<LocalStorageMessage<LocalStorageMessageType::MAP_POINT_DUAL_REMOVALS>>()}).first;
                }
                itr->second->map_point_ids.insert(map_point_id);
            }

            map_point_remove_message->map_point_ids.insert({map_point_id, true});

        } else {
            // If there is a single owner, remove only if underconstrained and not requested.
            auto observing_frames = map_.getObservingFrames(map_point_id);
            if (observing_frames.size() < 2 && entry.requesters.empty() && entry.status != Status::MERGING) {
                auto observations = map_.getObservationsForMapPoint(map_point_id);
                DCHECK(observations.size() <= 2);
                observations_to_remove.insert(observations.begin(), observations.end());
                map_point_remove_message->map_point_ids.insert({map_point_id, true});
                owners.erase(query.sender_id);
                entry.estimates.erase(query.sender_id);
                DCHECK(owners.empty());
                map_point_ids_to_remove.insert(map_point_id);
            } else {
                map_point_remove_message->map_point_ids.insert({map_point_id, false});
                // DLOG(INFO) << "Remove of map point " << map_point_id << " by " << query.sender_id << " failed.";
            }
        }
    }

    observationStorage(observations_to_remove, StorageAction::REMOVE);
    // DLOG_IF(INFO, num_underconstrained_removed > 0) << "Removed " << observations_to_remove.size() << " observations of "
    //                                                 << num_underconstrained_removed << " underconstrained map points.";

    for (const auto& map_point_id : map_point_ids_to_remove) {
        auto& entry = map_points_.find(map_point_id)->second.second;
        if (entry.status == Status::MERGED) {
            DCHECK(map_point_merges_.contains(map_point_id));
            map_point_merges_.erase(map_point_id);
            // DLOG(INFO) << "Removing merged point " << map_point_id;
        }
        entry.status = Status::REMOVED;
    }

    for (auto& [id, remove_message] : dual_remove_messages) {
        remove_message->receiver_id = id;
        communicator_->send(std::move(remove_message));
    }

    const auto debug_num_removed = map_point_remove_message->map_point_ids.size();

    communicator_->send(std::move(map_point_remove_message));
    // DLOG(INFO) << id_ << ": Removed " << debug_num_removed << " map points for " << query.sender_id;
}

} // namespace deco
