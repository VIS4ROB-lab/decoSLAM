//
// Created by philipp on 09.11.22.
//

#pragma once

#include <chrono>

#include "global.hpp"

namespace deco {
class Timer {
  public:
    explicit Timer(std::string&& name);
    Timer() = delete;
    ~Timer();

    auto tic() -> void;
    auto toc() -> void;
    auto display() -> void;

  private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    TimePoint t_start_;
    TimePoint t_stop_;

    std::string name_;
};
} // namespace deco
