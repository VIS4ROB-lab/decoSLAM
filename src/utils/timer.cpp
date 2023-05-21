//
// Created by philipp on 09.11.22.
//

#include <iostream>

#include <glog/logging.h>

#include "utils/timer.hpp"

namespace deco {

Timer::Timer(std::string&& name) : name_{name} {}

Timer::~Timer() = default;

auto Timer::tic() -> void {
    t_start_ = Clock::now();
}

auto Timer::toc() -> void {
    t_stop_ = Clock::now();
}

auto Timer::display() -> void {
    std::chrono::duration<double, std::milli> diff = t_stop_ - t_start_;
    DLOG(INFO) << name_ << " took " << diff.count() << " ms.";
}

} // namespace deco
