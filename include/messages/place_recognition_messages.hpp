//
// Created by philipp on 20.08.22.
//

#pragma once

#include <cereal/types/polymorphic.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/vector.hpp>
#include <hyper/variables/bearing.hpp>
#include <hyper/variables/groups/se3.hpp>

#include "messages/base.hpp"
#include "messages/serialization.hpp"

namespace deco {
enum class PlaceRecognitionMessageType {
    CANDIDATE_QUERY,
    CANDIDATE_RESPONSE,
    VERIFICATION_QUERIES,
    VERIFICATION_RESPONSE,
};

template <>
class Message<MessageType::PLACE_RECOGNITION> : public MessageBase {
  public:
    explicit Message(const PlaceRecognitionMessageType& message_type) : MessageBase{MessageType::PLACE_RECOGNITION}, subtype{message_type} {};
    ~Message() override = default;

    auto getMessageCategory() -> MessageCategory override {
        return MessageCategory::LOOP_CLOSURE;
    }

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<MessageBase>(this), subtype);
    }

    PlaceRecognitionMessageType subtype; ///< Subtype of PR message.
};
using PlaceRecognitionMessageBase = Message<MessageType::PLACE_RECOGNITION>;
template <PlaceRecognitionMessageType> class PlaceRecognitionMessage;

template <>
class PlaceRecognitionMessage<PlaceRecognitionMessageType::CANDIDATE_QUERY> : public PlaceRecognitionMessageBase {
  public:
    PlaceRecognitionMessage() : PlaceRecognitionMessageBase{PlaceRecognitionMessageType::CANDIDATE_QUERY} {};
    ~PlaceRecognitionMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<PlaceRecognitionMessageBase>(this), query_keyframe_id, descriptor);
    }

    StateId query_keyframe_id; ///< Id of the query keyframe.
    cv::Mat descriptor;        ///< Full image descriptor of the query keyframe.
};

template <>
class PlaceRecognitionMessage<PlaceRecognitionMessageType::CANDIDATE_RESPONSE> : public PlaceRecognitionMessageBase {
  public:
    PlaceRecognitionMessage() : PlaceRecognitionMessageBase{PlaceRecognitionMessageType::CANDIDATE_RESPONSE} {};
    ~PlaceRecognitionMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<PlaceRecognitionMessageBase>(this), query_keyframe_id, candidate_ids);
    }

    StateId query_keyframe_id;       ///< Id of the query keyframe.
    std::set<StateId> candidate_ids; ///< Set of candidates.
};

template <>
class PlaceRecognitionMessage<PlaceRecognitionMessageType::VERIFICATION_QUERIES> : public PlaceRecognitionMessageBase {
  public:
    PlaceRecognitionMessage() : PlaceRecognitionMessageBase{PlaceRecognitionMessageType::VERIFICATION_QUERIES} {};
    ~PlaceRecognitionMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<PlaceRecognitionMessageBase>(this), query_keyframe_id, candidate_keyframe_ids);
    }

    StateId query_keyframe_id;                   ///< Id of the query keyframe.
    std::vector<StateId> candidate_keyframe_ids; ///< Id of the requested candidate.
};

template <>
class PlaceRecognitionMessage<PlaceRecognitionMessageType::VERIFICATION_RESPONSE> : public PlaceRecognitionMessageBase {
  public:
    PlaceRecognitionMessage() : PlaceRecognitionMessageBase{PlaceRecognitionMessageType::VERIFICATION_RESPONSE} {};
    ~PlaceRecognitionMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<PlaceRecognitionMessageBase>(this),
            query_keyframe_id,
            candidate_keyframe_ids,
            descriptors,
            bearings,
            observations,
            extrinsics,
            candidate_poses,
            candidate_root_id);
    }

    StateId query_keyframe_id;                          ///< Id of the query keyframe.
    std::vector<StateId> candidate_keyframe_ids;        ///< Id of the candidate keyframe.
    std::vector<cv::Mat> descriptors;                   ///< Keypoint descriptors of the left image of the candidate keyframe.
    std::vector<std::vector<Bearing>> bearings;         ///< Bearings of the left image of the candidate keyframe.
    std::vector<std::set<ObservationKey>> observations; ///< Map point observations of the left image of the candidate keyframe.
    std::vector<SE3> extrinsics;                        ///< Extrinsics of the left camera of the candidate keyframe.
    std::vector<SE3> candidate_poses;                   ///< Pose of the candidate keyframe.
    Id candidate_root_id;                               ///< Frame of reference of the candidate keyframe.
};

} // namespace deco

CEREAL_REGISTER_TYPE(deco::PlaceRecognitionMessage<deco::PlaceRecognitionMessageType::CANDIDATE_QUERY>)
CEREAL_REGISTER_TYPE(deco::PlaceRecognitionMessage<deco::PlaceRecognitionMessageType::CANDIDATE_RESPONSE>)
CEREAL_REGISTER_TYPE(deco::PlaceRecognitionMessage<deco::PlaceRecognitionMessageType::VERIFICATION_QUERIES>)
CEREAL_REGISTER_TYPE(deco::PlaceRecognitionMessage<deco::PlaceRecognitionMessageType::VERIFICATION_RESPONSE>)
