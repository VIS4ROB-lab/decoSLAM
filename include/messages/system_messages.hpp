//
// Created by philipp on 12.12.22.
//

#pragma once

#include <cereal/types/polymorphic.hpp>

#include "messages/base.hpp"

namespace deco {

enum class SystemMessageType {
    HEARTBEAT_QUERY,
    HEARTBEAT_RESPONSE
};

template <>
class Message<MessageType::SYSTEM> : public MessageBase {
  public:
    explicit Message(const SystemMessageType& message_type) : MessageBase{MessageType::SYSTEM}, subtype{message_type} {};
    ~Message() override = default;

    auto getMessageCategory() -> MessageCategory override {
        return MessageCategory::SYSTEM;
    }

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<MessageBase>(this), subtype);
    }

    SystemMessageType subtype;
};
using SystemMessageBase = Message<MessageType::SYSTEM>;
template <SystemMessageType> class SystemMessage;

template <>
class SystemMessage<SystemMessageType::HEARTBEAT_QUERY> : public SystemMessageBase {
  public:
    SystemMessage() : SystemMessageBase{SystemMessageType::HEARTBEAT_QUERY} {};
    ~SystemMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SystemMessageBase>(this), heartbeat_id);
    }

    Id heartbeat_id;
};

template <>
class SystemMessage<SystemMessageType::HEARTBEAT_RESPONSE> : public SystemMessageBase {
  public:
    SystemMessage() : SystemMessageBase{SystemMessageType::HEARTBEAT_RESPONSE} {};
    ~SystemMessage() override = default;

    template <class Archive>
    auto serialize(Archive& archive) -> void {
        archive(cereal::base_class<SystemMessageBase>(this), heartbeat_id);
    }

    Id heartbeat_id;
};

} // namespace deco

CEREAL_REGISTER_TYPE(deco::SystemMessage<deco::SystemMessageType::HEARTBEAT_QUERY>)
CEREAL_REGISTER_TYPE(deco::SystemMessage<deco::SystemMessageType::HEARTBEAT_RESPONSE>)
