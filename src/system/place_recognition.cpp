//
// Created by philipp on 15.06.22.
//

#include <fstream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/features2d.hpp>

#include "map/map_point.hpp"
#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/alignment.hpp"
#include "system/communicator.hpp"
#include "system/place_recognition.hpp"
#include "utils/matching.hpp"
#include "utils/ransac.hpp"

namespace {
auto findWordIndex(const cv::Mat& descriptor, const cv::Mat& vocabulary) -> int {
    auto matcher = cv::BFMatcher(cv::NORM_L2);
    std::vector<cv::DMatch> word_match;
    matcher.match(descriptor, vocabulary, word_match);
    DCHECK(!word_match.empty());
    return word_match.front().trainIdx;
}

auto loadVocabulary(const deco::FilePath& path, const int feature_size) -> cv::Mat {
    std::ifstream file;

    file.open(path);
    CHECK(file.is_open()) << "Loading vocabulary from " << path << " failed.";

    std::string line;
    auto vocabulary = cv::Mat(0, feature_size, CV_32F);
    while (std::getline(file, line)) {
        std::stringstream stream(line);
        auto descriptor = cv::Mat(1, feature_size, CV_32F);
        for (auto i = 0; i < feature_size; ++i) {
            std::string entry;
            std::getline(stream, entry, ',');
            descriptor.col(i) = std::stod(entry);
        }
        vocabulary.push_back(descriptor);
    }
    return vocabulary;
}

auto loadDescriptors(const deco::FilePath& path, std::map<deco::Timestamp, cv::Mat>& descriptors, const int feature_size) -> void {
    std::ifstream file;
    file.open(path);
    CHECK(file.is_open()) << "Loading descriptors from " << path << " failed.";

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream stream(line);

        // Read timestamp
        std::string ts_str;
        std::getline(stream, ts_str, ',');
        const deco::Timestamp ts = std::stoull(ts_str);

        auto descriptor = cv::Mat(1, feature_size, CV_32F);
        for (auto i = 0; i < feature_size; ++i) {
            std::string entry;
            std::getline(stream, entry, ',');
            descriptor.col(i) = std::stod(entry);
        }
        descriptors.insert({ts, descriptor});
    }

    LOG(INFO) << "Loaded " << descriptors.size() << " descriptors.";
}

} // namespace

namespace deco {

PlaceRecognition::PlaceRecognition(const Id& id, StorageMutexHandler& mutex_handler)
    : id_{id},
      mutex_handler_{mutex_handler},
      communicator_{nullptr},
      shared_map_{nullptr},
      keyframe_queue_{"PlaceRecognition::KeyframeQueue", std::bind(&PlaceRecognition::processFrame_, this, std::placeholders::_1), ros::isShuttingDown},
      message_queue_{"PlaceRecognition::MessageQueue", std::bind(&PlaceRecognition::submitMessage_, this, std::placeholders::_1), ros::isShuttingDown} {}

auto PlaceRecognition::readParameters(const YAML::Node& node) -> void {
    netvlad_threshold_ = node["netvlad_threshold"].as<Scalar>();
    min_num_inliers_ = node["min_num_matches"].as<size_t>();
    min_inlier_ratio_ = node["min_inlier_ratio"].as<Scalar>();
    vocabulary_ = loadVocabulary(node["vocabulary_path"].as<std::string>(), node["feature_size"].as<int>());
    if (node["descriptor_path"]) {
        const auto path_string = node["descriptor_path"].as<std::string>();
        LOG(INFO) << "Using pre-extracted descriptors at " << path_string << " for agent " << id_ << ".";
        loadDescriptors(path_string, descriptors_, node["feature_size"].as<int>());
    } else {
        // TODO: Construct netvlad extractor here.
    }
}

auto PlaceRecognition::createWordMap(const std::set<Id>& agent_ids) -> void {
    auto itr = agent_ids.begin();
    for (auto i = 0; i < vocabulary_.rows; ++i) {
        word_map_.insert({i, *itr});
        ++itr;
        if (itr == agent_ids.end()) {
            itr = agent_ids.begin();
        }
    }

    LOG(INFO) << "Word map agent " << id_;
    for (const auto& [idx, id] : word_map_) {
        LOG(INFO) << idx << " --> " << id;
    }
}

auto PlaceRecognition::setMatchingParameters(const MatchingParameters& parameters) -> void {
    parameters_ = parameters;
}

auto PlaceRecognition::setSharedMap(SharedMap* shared_map) -> void {
    shared_map_ = shared_map;
}

auto PlaceRecognition::setLocalStorage(LocalStorage* local_storage) -> void {
    local_storage_ = local_storage;
}

auto PlaceRecognition::setCommunicator(Communicator* communicator) -> void {
    communicator_ = communicator;
}

auto PlaceRecognition::setAlignment(Alignment* alignment) -> void {
    alignment_ = alignment;
}

auto PlaceRecognition::start() -> void {
    keyframe_queue_.start();
    message_queue_.start();
}

auto PlaceRecognition::stop() -> void {
    keyframe_queue_.stop();
    message_queue_.stop();
}

auto PlaceRecognition::submit(const StateId& frame_id) -> void {
    auto id = frame_id;
    keyframe_queue_.submit(std::move(id));
}

auto PlaceRecognition::messageQueue() -> ProcessingQueue<std::unique_ptr<MessageBase>>& {
    return message_queue_;
}

auto PlaceRecognition::processFrame_(StateId&& frame_id) -> void {
    // DLOG(INFO) << "PR: Processing frame " << frame_id;

    Frame* frame;
    {
        auto lock = mutex_handler_.lockSharedMap();
        frame = shared_map_->getKeyframe(frame_id);
    }

    auto query = std::make_unique<PlaceRecognitionMessage<PlaceRecognitionMessageType::CANDIDATE_QUERY>>();
    {
        auto lock = std::lock_guard{frame->mutex_}; // TODO: Lock later once NetVLAD extraction is implemented.
        cv::Mat descriptor;
        if (descriptors_.empty()) {
            // TODO: Extract netvlad.
            LOG(FATAL) << "NetVLAD extraction not implemented.";
        } else {
            DCHECK(descriptors_.contains(frame->timestamp)) << frame->timestamp;
            descriptor = descriptors_.find(frame->timestamp)->second;
        }

        // Word and agent lookup.
        auto word_idx = findWordIndex(descriptor, vocabulary_);
        auto word_agent_id = word_map_.find(word_idx)->second;

        // Generate query.
        query->receiver_id = word_agent_id;
        query->descriptor = descriptor;
        query->query_keyframe_id = frame->id;
    }

    communicator_->send(std::move(query));
}

auto PlaceRecognition::submitMessage_(std::unique_ptr<MessageBase>&& message) -> void {
    DCHECK(message->type == MessageType::PLACE_RECOGNITION);
    auto& pr_message = dynamic_cast<PlaceRecognitionMessageBase&>(*message);

    switch (pr_message.subtype) {
        case PlaceRecognitionMessageType::CANDIDATE_QUERY:
            processQuery_(pr_message);
            break;
        case PlaceRecognitionMessageType::CANDIDATE_RESPONSE:
            requestCandidates_(pr_message);
            break;
        case PlaceRecognitionMessageType::VERIFICATION_QUERIES:
            handleVerificationQueries_(pr_message);
            break;
        /*case PlaceRecognitionMessageType::POSE_UPDATES:

            break;*/
        case PlaceRecognitionMessageType::VERIFICATION_RESPONSE:
            verifyCandidate_(pr_message);
            break;
        default:
            LOG(FATAL) << "Unknown message type.";
    }
}

auto PlaceRecognition::processQuery_(PlaceRecognitionMessageBase& message) -> void {
    auto& query = dynamic_cast<PlaceRecognitionMessage<PlaceRecognitionMessageType::CANDIDATE_QUERY>&>(message);

    // Look up the word.
    const auto word_idx = findWordIndex(query.descriptor, vocabulary_);

    // Candidate selection.
    auto itr = index_.find(word_idx);
    if (itr == index_.end()) { // No entry for word.
        itr = index_.insert({word_idx, {}}).first;
    } else {
        auto& entries = itr->second;
        DCHECK(!entries.empty());

        // Obtain all stored descriptors for the word.
        auto candidate_descriptors = cv::Mat();
        std::vector<StateId> all_ids;
        for (const auto& [id, candidate_descriptor] : entries) {
            candidate_descriptors.push_back(candidate_descriptor);
            all_ids.push_back(id);
        }
        // Linearly search for matches within threshold.
        auto matcher = cv::BFMatcher(cv::NORM_L2);
        std::vector<std::vector<cv::DMatch>> candidate_matches;
        matcher.radiusMatch(query.descriptor, candidate_descriptors, candidate_matches, static_cast<float>(netvlad_threshold_));

        // Send query response if we found matches.
        if (!candidate_matches[0].empty()) {
            auto response = std::make_unique<PlaceRecognitionMessage<PlaceRecognitionMessageType::CANDIDATE_RESPONSE>>();
            response->receiver_id = query.sender_id;
            response->query_keyframe_id = query.query_keyframe_id;
            for (const auto& match : candidate_matches[0]) {
                response->candidate_ids.insert(all_ids[match.trainIdx]);
            }
            communicator_->send(std::move(response));
        }
    }

    // Store query.
    auto [_, inserted] = itr->second.insert({query.query_keyframe_id, query.descriptor});
    DCHECK(inserted) << "Received multiple queries for the same keyframe.";
}

auto PlaceRecognition::requestCandidates_(PlaceRecognitionMessageBase& message) -> void {
    auto& response = dynamic_cast<PlaceRecognitionMessage<PlaceRecognitionMessageType::CANDIDATE_RESPONSE>&>(message);
    auto queries = std::map<Id, std::unique_ptr<PlaceRecognitionMessage<PlaceRecognitionMessageType::VERIFICATION_QUERIES>>>();
    {
        auto lock = mutex_handler_.lockSharedMap();

        for (const auto& candidate_keyframe_id : response.candidate_ids) {
            if (!shared_map_->isCovisible(response.query_keyframe_id, candidate_keyframe_id)) {
                auto itr = queries.find(candidate_keyframe_id.first);
                if (itr == queries.end()) {
                    itr = queries.insert({candidate_keyframe_id.first, std::make_unique<PlaceRecognitionMessage<PlaceRecognitionMessageType::VERIFICATION_QUERIES>>()}).first;
                }
                itr->second->query_keyframe_id = response.query_keyframe_id;
                // DLOG(INFO) << id_ << ": Try " << response.query_keyframe_id << " --> " << candidate_keyframe_id;
                itr->second->candidate_keyframe_ids.push_back(candidate_keyframe_id);
            }
        }
    }

    for (auto& [id, query] : queries) {
        query->receiver_id = id;
        communicator_->send(std::move(query));
    }
}

auto PlaceRecognition::handleVerificationQueries_(PlaceRecognitionMessageBase& message) -> void {
    auto& query = dynamic_cast<PlaceRecognitionMessage<PlaceRecognitionMessageType::VERIFICATION_QUERIES>&>(message);

    bool need_alignment;
    {
        auto lock = mutex_handler_.lockAlignment();
        need_alignment = !alignment_->isAligned(query.sender_id);
    }

    if (need_alignment) {
        // shared_map_->synchronizeKeyframePoses(query.query_keyframe_id, query.candidate_keyframe_ids);
        // DLOG(FATAL) << "Cannot align.";
        sendCandidates_(query.query_keyframe_id, query.candidate_keyframe_ids, true);
    } else {
        sendCandidates_(query.query_keyframe_id, query.candidate_keyframe_ids, false);
    }
}

auto PlaceRecognition::sendCandidates_(const StateId& query_keyframe_id, const std::vector<StateId>& candidate_ids, bool send_pose) -> void {
    auto response = std::make_unique<PlaceRecognitionMessage<PlaceRecognitionMessageType::VERIFICATION_RESPONSE>>();

    {
        auto lock = mutex_handler_.lockAlignment();
        response->candidate_root_id = alignment_->getRootId();
    }

    {
        auto lock = mutex_handler_.lockSharedMap();
        response->receiver_id = query_keyframe_id.first;
        response->query_keyframe_id = query_keyframe_id;

        for (const auto& id : candidate_ids) {
            // Look up the frame
            auto frame = shared_map_->getKeyframe(id);
            auto frame_lock = std::lock_guard{frame->mutex_};

            // Build response
            response->candidate_keyframe_ids.push_back(id);
            response->descriptors.push_back(frame->image_data[0].keypoint_descriptors);
            response->bearings.push_back(frame->image_data[0].bearings);
            response->observations.push_back(shared_map_->getMap()->getObservationsForCamera(id, 0));
            response->extrinsics.push_back(frame->calibration[0].se3_body_camera_);
            if (send_pose) {
                response->candidate_poses.push_back(frame->se3_world_body);
            }
        }
    }

    communicator_->send(std::move(response));
}

auto PlaceRecognition::verifyCandidate_(PlaceRecognitionMessageBase& message) -> void {
    auto& verification_response = dynamic_cast<PlaceRecognitionMessage<PlaceRecognitionMessageType::VERIFICATION_RESPONSE>&>(message);

    DLOG(INFO) << "Checking " << verification_response.candidate_keyframe_ids.size() << " candidates for " << verification_response.query_keyframe_id;

    // Collect map points.
    std::set<StateId> map_point_ids;
    cv::Mat map_point_descriptors;
    std::vector<Position> map_point_positions;
    bool need_alignment_before;
    Id world_frame_id_before;
    {
        auto lock = mutex_handler_.lockAllStorages();
        map_point_ids = local_storage_->getMap()->getObservedMapPoints(verification_response.query_keyframe_id);
        for (const auto& id : map_point_ids) {
            if (local_storage_->hasMapPointPosition(id)) {
                map_point_descriptors.push_back(local_storage_->getMapPointDescriptor(id));
                map_point_positions.push_back(local_storage_->getMapPointPosition(id));
            }
        }
        need_alignment_before = !alignment_->isAligned(verification_response.candidate_root_id);
        world_frame_id_before = alignment_->getRootId();
    }

    std::map<StateId, std::set<StateId>> merges;
    std::set<ObservationKey> new_observations;
    std::vector<SE3> alignment_transformations;
    for (auto i = 0u; i < verification_response.candidate_keyframe_ids.size(); ++i) {
        // Get map point to keypoint map.
        std::map<int, StateId> candidate_kp_to_mp;
        for (const auto& key : verification_response.observations[i]) {
            candidate_kp_to_mp.insert({key.keypoint_index, key.map_point_id});
        }

        // Match descriptors.
        auto matches = descriptorMatching(map_point_descriptors, verification_response.descriptors[i], parameters_.matching_threshold);
        if (matches.size() >= min_num_inliers_) {
            // Filter matches using 3D-2D Ransac.
            auto camera_matches = std::vector<std::vector<cv::DMatch>>{matches};
            const auto se3_queryworld_candidate = ransac3d2d(map_point_positions, {verification_response.bearings[i]}, camera_matches, {verification_response.extrinsics[i]}, SE3::Identity(), parameters_.ransac_threshold);

            // Confirm verification.
            if (camera_matches[0].size() >= min_num_inliers_) {
                DLOG(INFO) << id_ << ": Matched frames " << verification_response.query_keyframe_id << " <--> " << verification_response.candidate_keyframe_ids[i];

                // Register the found transformation.
                if (need_alignment_before) {
                    auto se3_queryworld_candidateworld = se3_queryworld_candidate.groupPlus(verification_response.candidate_poses[i].groupInverse());
                    alignment_transformations.push_back(se3_queryworld_candidateworld);
                }

                // Process matches.
                for (const auto& match : camera_matches[0]) {
                    auto query_map_point_id = *std::next(map_point_ids.begin(), match.queryIdx);
                    if (candidate_kp_to_mp.contains(match.trainIdx)) {
                        auto candidate_map_point_id = candidate_kp_to_mp.find(match.trainIdx)->second;
                        if (candidate_map_point_id != query_map_point_id) {
                            // DLOG(INFO) << id_ << ": Merge " << query_map_point_id << " into " << candidate_map_point_id;
                            auto itr = merges.find(query_map_point_id);
                            if (itr == merges.end()) {
                                itr = merges.insert({query_map_point_id, {}}).first;
                            }
                            itr->second.insert(candidate_map_point_id);
                        }
                    } else {
                        new_observations.insert(ObservationKey{
                            .frame_id = verification_response.candidate_keyframe_ids[i],
                            .map_point_id = query_map_point_id,
                            .camera_index = 0,
                            .keypoint_index = match.trainIdx});
                    }
                }
            }
        }
    }

    // Detect if we still need to align.
    if (need_alignment_before && !alignment_transformations.empty()) {
        auto lock = mutex_handler_.lockAlignment();
        if (!alignment_->isAligned(verification_response.candidate_root_id)) {
            const auto world_frame_id_after = alignment_->getRootId();
            SE3 alignment_transform = alignment_transformations.front(); // TODO: Using the first transformation found. Maybe average?

            for (const auto& debug_transformation : alignment_transformations) {
                const auto diff_norm = (alignment_transform.groupInverse().groupPlus(debug_transformation)).toTangent().norm();
                DLOG(INFO) << "Found additional transformation with difference norm " << diff_norm;
            }

            // Adapt found transformation if the world frame id changed in the meantime.
            if (world_frame_id_after != world_frame_id_before) {
                auto transform = alignment_->getTransformationToWorld(world_frame_id_before);
                alignment_transform = transform.groupPlus(alignment_transform);
            }
            alignment_->addTransformation(world_frame_id_after, verification_response.candidate_root_id, alignment_transform);
        } else {
            DLOG(INFO) << "Got aligned to " << verification_response.candidate_root_id << " in the meantime.";
        }
    }

    std::vector<std::pair<StateId, StateId>> mergeable_pairs;
    for (const auto& [remaining_id, mergeable_ids] : merges) {
        const auto& first_merge = *mergeable_ids.begin(); // TODO: Could account for multiple merges here.
        mergeable_pairs.emplace_back(remaining_id, first_merge);
    }

    {
        auto lock = mutex_handler_.lockAllStorages();
        shared_map_->mergeMapPoints(mergeable_pairs);
        shared_map_->observationStorage(new_observations, StorageAction::ADD);
    }
}

} // namespace deco
