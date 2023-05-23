//
// Created by philipp on 15.06.22.
//

#pragma once

#include <map>
#include <set>

#include <opencv2/core.hpp>

#include "global.hpp"
#include "messages/place_recognition_messages.hpp"
#include "utils/processing.hpp"

namespace YAML {
class Node;
}

namespace deco {
struct Frame;
class SharedMap;
class LocalStorage;
class Communicator;
class Alignment;

class PlaceRecognition {
  public:
    using Descriptor = cv::Mat;
    using WordMap = std::map<int, Id>;
    using Index = std::map<int, std::map<StateId, Descriptor>>;

    PlaceRecognition() = delete;
    PlaceRecognition(const Id& id, StorageMutexHandler& mutex_handler);
    ~PlaceRecognition() = default;

    auto readParameters(const YAML::Node& node) -> void;
    auto setMatchingParameters(const MatchingParameters& parameters) -> void;
    auto createWordMap(const std::set<Id>& agent_ids) -> void;

    auto setSharedMap(SharedMap* shared_map) -> void;
    auto setLocalStorage(LocalStorage* local_storage) -> void;

    /// Set the communcation module.
    /// \param communicator Pointer to communicator.
    auto setCommunicator(Communicator* communicator) -> void;

    auto setAlignment(Alignment* alignment) -> void;

    auto start() -> void;
    auto stop() -> void;

    auto submit(const StateId& frame_id) -> void;

    auto messageQueue() -> ProcessingQueue<std::unique_ptr<MessageBase>>&;

  private:
    auto processFrame_(StateId&& frame_id) -> void;

    auto submitMessage_(std::unique_ptr<MessageBase>&& message) -> void;

    /// Processes a place recognition query and sends a response with candidate ids.
    /// \param message Owning pointer to query message.
    auto processQuery_(PlaceRecognitionMessageBase& message) -> void;

    /// Sends KF queries for ids obtained from a place recognition query response.
    /// \param message Owning pointer to query response message.
    auto requestCandidates_(PlaceRecognitionMessageBase& message) -> void;

    auto handleVerificationQueries_(PlaceRecognitionMessageBase& message) -> void;

    /// Fetches keyframe data (descriptors and bearings) and sends a response to a KF query.
    /// \param message Owning pointer to KF query.
    auto sendCandidates_(const StateId& query_keyframe_id, const std::vector<StateId>& candidate_ids, bool send_pose) -> void;

    /// Processes KF query response by verifying the candidate and generating map point merges.
    /// \param message Owning pointer to KF query response.
    auto verifyCandidate_(PlaceRecognitionMessageBase& message) -> void;

    Id id_;

    cv::Mat vocabulary_; ///< Vocabulary.
    WordMap word_map_;   ///< Maps vocabulary indices to agent ids.
    Index index_;        ///< Stores KF ids and descriptors for assigned words.

    MatchingParameters parameters_;
    Scalar netvlad_threshold_; ///< Threshold for matching NetVLAD descriptors.
    size_t min_num_inliers_;
    Scalar min_inlier_ratio_;

    StorageMutexHandler& mutex_handler_;
    Communicator* communicator_; ///< Communicator.
    SharedMap* shared_map_;
    LocalStorage* local_storage_;
    Alignment* alignment_;

    ProcessingQueue<StateId> keyframe_queue_;
    ProcessingQueue<std::unique_ptr<MessageBase>> message_queue_;

    std::map<Timestamp, cv::Mat> descriptors_;
};
} // namespace deco
