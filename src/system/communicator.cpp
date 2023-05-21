//
// Created by philipp on 05.10.22.
//

#include <fstream>

#include <glog/logging.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <cereal/archives/binary.hpp>

#include "messages/base.hpp"
#include "messages/system_messages.hpp"
#include "storage/local_storage.hpp"
#include "storage/shared_map.hpp"
#include "system/alignment.hpp"
#include "system/communicator.hpp"
#include "system/place_recognition.hpp"

namespace deco {

Communicator::Communicator(const Id& id)
    : id_{id},
      sending_queue_{"Communicator::SendingQueue", std::bind(&Communicator::send_, this, std::placeholders::_1), ros::isShuttingDown},
      distribution_queue_{"Communicator::DistributionQueue", std::bind(&Communicator::distribute_, this, std::placeholders::_1), ros::isShuttingDown},
      context_{zmq::context_t{1}},
      beat_count_{0} {}

Communicator::~Communicator() = default;

auto Communicator::readParameters(const YAML::Node& yaml_node) -> void {
    using Port = uint16_t;

    const auto network_config_path = yaml_node["network_config_file"].as<std::string>();
    YAML::Node network_node;
    try {
        network_node = YAML::LoadFile(network_config_path);
    } catch (...) {
        LOG(ERROR) << "YAML network configuration file at " << network_config_path << " not found.";
    }

    std::map<Id, std::pair<std::string, std::vector<Port>>> port_map;
    for (const auto& subnode : network_node["nodes"]) {
        const auto id = subnode["id"].as<Id>();
        port_map.insert({id, {subnode["ip_address"].as<std::string>(), subnode["ports"].as<std::vector<Port>>()}});
    }

    for (const auto& [id, connections] : port_map) {
        const auto& [ip_address, ports] = connections;

        if (id == id_) { // Define sending endpoints.
            for (auto i = 0u; i < ports.size(); ++i) {
                if (i != id_) {
                    sending_endpoints_.insert({i, "tcp://*:" + std::to_string(ports[i])});
                }
            }
        } else { // Define receiving endpoints
            receiving_endpoints_.insert({id, "tcp://" + ip_address + ":" + std::to_string(ports[id_])});
            connection_timers_.insert({id, {}});
        }
    }

    LOG(INFO) << "Network connections for agent " << id_;
    LOG(INFO) << "--------- Sending endpoints ---------";
    for (const auto& [id, endpoint] : sending_endpoints_) {
        LOG(INFO) << id_ << " --> " << id << ": " << endpoint;
    }

    LOG(INFO) << "--------- Receiving endpoints ---------";
    for (const auto& [id, endpoint] : receiving_endpoints_) {
        LOG(INFO) << id << " --> " << id_ << ": " << endpoint;
    }
}

auto Communicator::getAllAgentIds() -> std::set<Id> {
    std::set<Id> ids;
    for (const auto& [id, _] : sending_endpoints_) {
        ids.insert(id);
    }
    ids.insert(id_);
    return ids;
}

auto Communicator::setSharedMap(SharedMap* shared_map) -> void {
    shared_map_ = shared_map;
}

auto Communicator::setLocalStorage(LocalStorage* local_storage) -> void {
    local_storage_ = local_storage;
}

auto Communicator::setPlaceRecognition(PlaceRecognition* place_recognition) -> void {
    place_recognition_ = place_recognition;
}

auto Communicator::setAlignment(Alignment* alignment) -> void {
    alignment_ = alignment;
}

auto Communicator::start() -> void {
    for (const auto& [id, endpoint] : sending_endpoints_) {
        auto [itr, inserted] = senders_.insert({id, Socket{context_, ZMQ_PUSH}});
        DCHECK(inserted);
        itr->second.bind(endpoint);
    }

    receiver_ = Socket{context_, ZMQ_PULL};
    for (const auto& [id, endpoint] : receiving_endpoints_) {
        receiver_.connect(endpoint);
    }

    sending_queue_.start();
    distribution_queue_.start();
    receiver_thread_ = std::thread{&Communicator::receive_, this};
    heartbeat_thread_ = std::thread{&Communicator::heartbeat_, this};
}

auto Communicator::stop() -> void {
    heartbeat_thread_.join();

    sending_queue_.stop();
    distribution_queue_.stop();

    LOG(INFO) << "Shutting down ZMQ context.";
    for (auto& [id, sender] : senders_) {
        LOG(INFO) << id_ << ": Close outgoing connection to " << id;
        sender.close();
    }
    receiver_thread_.join();

    receiver_.close();
    context_.close();
    context_.shutdown();
}

auto Communicator::writeLogs(const FilePath& output_path) -> void {
    if (!communication_log_.empty()) {
        const auto separator = ", ";
        const auto file_path = output_path / "communication_log.csv";
        std::ofstream out_file;
        out_file.open(file_path);
        CHECK(out_file.is_open()) << "Failed to open file " << file_path;

        // Header.
        out_file << "message_id" << separator << "total_data_sent[kB]"
                 << "\n";
        for (const auto& [message_category, total_bytes_sent] : communication_log_) {
            out_file << static_cast<int>(message_category) << separator
                     << static_cast<Scalar>(total_bytes_sent) << "\n";
        }
        out_file.close();
    } else {
        LOG(INFO) << "Communication log is empty, no file written.";
    }
}

auto Communicator::send(std::unique_ptr<MessageBase>&& message) -> void {
    message->sender_id = this->id_;
    if (message->receiver_id == id_) {
        distribute_(std::move(message));
    } else {
        sending_queue_.submit(std::move(message));
    }
}

auto Communicator::heartbeat_() -> void {
    while (!ros::isShuttingDown()) {
        {
            // LOG(INFO) << "Sending Heartbeat " << beat_count_;
            auto lock = std::lock_guard{heartbeat_mutex_};
            for (auto& [id, timers] : connection_timers_) {
                DCHECK(id != id_);
                auto itr = timers.insert({beat_count_, Timer(std::to_string(id_) + " --> " + std::to_string(id) + ": " + std::to_string(beat_count_))}).first;
                auto message = std::make_unique<SystemMessage<SystemMessageType::HEARTBEAT_QUERY>>();
                message->receiver_id = id;
                message->heartbeat_id = beat_count_;
                this->send(std::move(message));
                itr->second.tic();
            }
        }
        ++beat_count_;
        sleep(1);
    }
    LOG(INFO) << "Stop sending heartbeats.";
}

auto Communicator::send_(std::unique_ptr<MessageBase>&& message) -> void {
    DCHECK(message->receiver_id != this->id_);

    // Find message type in log.
    auto itr = communication_log_.find(message->getMessageCategory());
    if (itr == communication_log_.end()) {
        itr = communication_log_.insert({message->getMessageCategory(), 0}).first;
    }
    auto& log_entry = itr->second;

    // Serialize message.
    std::stringstream stream;
    {
        cereal::BinaryOutputArchive output_archive(stream);
        output_archive(message);
    }

    // Send.
    zmq::message_t zmq_message(stream.str().c_str(), stream.str().length());
    log_entry += static_cast<Scalar>(zmq_message.size()) / 1000.0;
    senders_.find(message->receiver_id)->second.send(zmq_message, zmq::send_flags::dontwait);
}

auto Communicator::receive_() -> void {
    while (!ros::isShuttingDown()) {
        // Read binary message into stringstream.
        zmq::message_t zmq_message;
        zmq::recv_result_t msg_size = receiver_.recv(zmq_message, zmq::recv_flags::dontwait);
        if (msg_size > 0) {
            std::string message_string(static_cast<char*>(zmq_message.data()), zmq_message.size());
            std::istringstream input_stream(message_string);

            // De-serialize.
            auto message = std::unique_ptr<MessageBase>();
            {
                cereal::BinaryInputArchive input_archive(input_stream);
                input_archive(message);
            }

            // Submit to internal distributor.
            distribution_queue_.submit(std::move(message));
        }
    }
}

auto Communicator::distribute_(std::unique_ptr<MessageBase>&& message) -> void {
    switch (message->type) {
        case MessageType::PLACE_RECOGNITION:
            place_recognition_->messageQueue().submit(std::move(message));
            break;
        case MessageType::ALIGNMENT:
            alignment_->submitMessage(std::move(message));
            break;
        case MessageType::SHARED_MAP:
            shared_map_->submitMessage(std::move(message));
            break;
        case MessageType::LOCAL_STORAGE:
            local_storage_->submitMessage(std::move(message));
            break;
        case MessageType::SYSTEM:
            this->submitSystemMessage_(std::move(message));
            break;
        default:
            LOG(FATAL) << "Unknown message type.";
    }
}

auto Communicator::submitSystemMessage_(std::unique_ptr<MessageBase>&& message) -> void {
    DCHECK(message->type == MessageType::SYSTEM);
    auto& system_message = dynamic_cast<SystemMessageBase&>(*message);

    switch (system_message.subtype) {
        case SystemMessageType::HEARTBEAT_QUERY:
            respondToHeartbeat_(system_message);
            break;
        case SystemMessageType::HEARTBEAT_RESPONSE:
            timeConnection_(system_message);
            break;
        default:
            LOG(FATAL) << "Unknown message type.";
    }
}

auto Communicator::respondToHeartbeat_(SystemMessageBase& message) -> void {
    auto& query = dynamic_cast<SystemMessage<SystemMessageType::HEARTBEAT_QUERY>&>(message);
    auto response = std::make_unique<SystemMessage<SystemMessageType::HEARTBEAT_RESPONSE>>();
    response->receiver_id = query.sender_id;
    response->heartbeat_id = query.heartbeat_id;
    this->send(std::move(response));
}

auto Communicator::timeConnection_(SystemMessageBase& message) -> void {
    auto& response = dynamic_cast<SystemMessage<SystemMessageType::HEARTBEAT_RESPONSE>&>(message);
    {
        auto lock = std::lock_guard{heartbeat_mutex_};
        auto& timers = connection_timers_.find(response.sender_id)->second;
        auto itr = timers.find(response.heartbeat_id);
        itr->second.toc();
        // itr->second.display();
        timers.erase(itr);
    }
}

} // namespace deco
