//
// Created by philipp on 05.10.22.
//

#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include <zmq.hpp>

#include "global.hpp"
#include "messages/base.hpp"
#include "messages/system_messages.hpp"
#include "utils/processing.hpp"
#include "utils/timer.hpp"

namespace YAML {
class Node;
}

namespace deco {
class MessageBase;
class SharedMap;
class LocalStorage;
class PlaceRecognition;
class Alignment;

class Communicator {
  public:
    Communicator(const Id& id);
    ~Communicator();

    auto readParameters(const YAML::Node& yaml_node) -> void;
    auto getAllAgentIds() -> std::set<Id>;
    auto setSharedMap(SharedMap* shared_map) -> void;
    auto setLocalStorage(LocalStorage* local_storage) -> void;
    auto setPlaceRecognition(PlaceRecognition* place_recognition) -> void;
    auto setAlignment(Alignment* alignment) -> void;

    auto start() -> void;
    auto stop() -> void;
    auto writeLogs(const FilePath& output_path) -> void;
    auto send(std::unique_ptr<MessageBase>&& message) -> void;

  private:
    using Context = zmq::context_t;
    using Socket = zmq::socket_t;

    auto heartbeat_() -> void;
    auto send_(std::unique_ptr<MessageBase>&& message) -> void;
    auto receive_() -> void;
    auto distribute_(std::unique_ptr<MessageBase>&& message) -> void;

    auto submitSystemMessage_(std::unique_ptr<MessageBase>&& message) -> void;
    auto respondToHeartbeat_(SystemMessageBase& message) -> void;
    auto timeConnection_(SystemMessageBase& message) -> void;

    Id id_;

    SharedMap* shared_map_;
    LocalStorage* local_storage_;
    PlaceRecognition* place_recognition_;
    Alignment* alignment_;

    ProcessingQueue<std::unique_ptr<MessageBase>> sending_queue_;
    ProcessingQueue<std::unique_ptr<MessageBase>> distribution_queue_;

    std::thread receiver_thread_;

    // Sockets and endpoints.
    Context context_;

    std::map<Id, std::string> sending_endpoints_;
    std::map<Id, std::string> receiving_endpoints_;
    std::map<Id, Socket> senders_;
    Socket receiver_;

    // Heartbeat.
    std::mutex heartbeat_mutex_;
    Id beat_count_;
    std::thread heartbeat_thread_;
    std::map<Id, std::map<Id, Timer>> connection_timers_;

    // Logging.
    std::map<MessageCategory, Scalar> communication_log_;
};
} // namespace deco
