//
// Created by philipp on 05.10.22.
//

#include <glog/logging.h>

#include "map/map_point.hpp"

namespace deco {

auto MapPoint::addDescriptor(const cv::Mat& descriptor) -> void {
    // Update the medioid (representative descriptor).
    if (descriptors_.rows < 1) {
        // First descriptor added is always the mediod.
        medioid_index_ = 0;
        scores_.push_back(0.0);
    } else {
        // Compute distances to all existing descriptor for new descriptor. Update scores of existing descriptors.
        scores_.push_back(0.0);
        for (auto i = 0; i < descriptors_.rows; ++i) {
            auto score = cv::norm(descriptors_.row(i), descriptor, cv::NORM_HAMMING);
            scores_[i] += score;
            scores_.back() += score;
        }
        // Mediod is the descriptor with the smallest cumulative distance to all others.
        medioid_index_ = static_cast<int>(std::distance(scores_.begin(), std::min_element(scores_.begin(), scores_.end())));
    }

    // Store descriptor.
    descriptors_.push_back(descriptor);
}

auto MapPoint::getDescriptor() const -> cv::Mat {
    DCHECK(descriptors_.rows > 1) << "No descriptor added to map point yet.";
    return descriptors_.row(medioid_index_);
}

} // namespace deco
