//
// Created by philipp on 05.10.22.
//

#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace deco {

class StorageMutexHandler {
  public:
    auto lockAllStorages() -> std::scoped_lock<std::recursive_mutex, std::recursive_mutex, std::recursive_mutex> {
        return std::scoped_lock{shared_map_mutex_, local_storage_mutex_, alignment_mutex_};
    }

    auto lockSharedMap() -> std::lock_guard<std::recursive_mutex> {
        return std::lock_guard{shared_map_mutex_};
    }

    auto lockLocalStorage() -> std::lock_guard<std::recursive_mutex> {
        return std::lock_guard{local_storage_mutex_};
    }

    auto lockAlignment() -> std::lock_guard<std::recursive_mutex> {
        return std::lock_guard{alignment_mutex_};
    }

  private:
    mutable std::recursive_mutex shared_map_mutex_, local_storage_mutex_, alignment_mutex_;
};

template <typename DataType>
class ProcessingQueue {
  public:
    ProcessingQueue(std::string identifier, std::function<void(DataType&&)>&& process_function, std::function<bool(void)>&& shutdown_function);
    auto start() -> void;
    auto stop() -> void;
    auto submit(DataType&& data) -> void;
    auto submit(std::vector<DataType>&& data) -> void;

  private:
    auto spin_() -> void;

    std::thread thread_;                              ///< Thread.
    mutable std::mutex mutex_;                        ///< Mutex.
    mutable std::condition_variable queue_condition_; ///< Queue condition.
    mutable std::deque<DataType> queue_;              ///< Queue.
    mutable bool notified_ = false;                   ///< Notified flag.

    std::function<void(DataType&&)> process_function_;
    std::function<bool()> shutdown_function_;

    std::string identifier_;
};

template <typename DataType>
ProcessingQueue<DataType>::ProcessingQueue(std::string identifier, std::function<void(DataType&&)>&& process_function, std::function<bool()>&& shutdown_function)
    : process_function_{process_function},
      shutdown_function_{shutdown_function},
      identifier_{std::move(identifier)} {}

template <typename DataType>
auto ProcessingQueue<DataType>::start() -> void {
    thread_ = std::thread{&ProcessingQueue::spin_, this};
}

template <typename DataType>
auto ProcessingQueue<DataType>::stop() -> void {
    queue_condition_.notify_one();
    thread_.join();
}

template <typename DataType>
auto ProcessingQueue<DataType>::submit(DataType&& data) -> void {
    auto lock = std::unique_lock<std::mutex>{mutex_};
    queue_.push_back(std::move(data));
    notified_ = true;
    lock.unlock();
    queue_condition_.notify_one();
}

template <typename DataType>
auto ProcessingQueue<DataType>::submit(std::vector<DataType>&& data) -> void {
    if (data.empty()) {
        return;
    }
    auto lock = std::unique_lock<std::mutex>{mutex_};
    for (auto& item : data) {
        queue_.push_back(std::move(item));
    }
    notified_ = true;
    lock.unlock();
    queue_condition_.notify_one();
}

template <typename DataType>
auto ProcessingQueue<DataType>::spin_() -> void {
    while (!shutdown_function_()) {
        auto lock = std::unique_lock<std::mutex>{mutex_};

        if (queue_.empty()) {
            // LOG(INFO) << identifier_ << ": Waiting for notification.";
            queue_condition_.wait(lock, [this]() -> bool { return notified_ || shutdown_function_(); });
            if (shutdown_function_()) {
                break;
            }
        }

        DCHECK(!queue_.empty());
        std::vector<DataType> data;
        while (!queue_.empty()) {
            data.template emplace_back(std::move(queue_.front()));
            queue_.pop_front();
        }
        notified_ = false;
        lock.unlock();

        for (auto& item : data) {
            process_function_(std::move(item));
        }
    }
    LOG(INFO) << identifier_ << " stopped.";
}

} // namespace deco
