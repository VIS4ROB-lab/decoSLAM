//
// Created by philipp on 05.10.22.
//

#pragma once
#include <hyper/variables/cartesian.hpp>
#include <opencv2/core.hpp>

#include "global.hpp"

namespace deco {

struct MapPoint {
  public:
    /// Add a descriptor to the map point.
    /// \param descriptor Reference to the descriptor.
    auto addDescriptor(const cv::Mat& descriptor) -> void;

    /// Get the representative descriptor (medioid of all descriptors).
    /// \return Representative map point descriptor.
    auto getDescriptor() const -> cv::Mat;

    StateId id;        ///< Id of the map point.
    Position position; ///< Map point position.
    Id reference_frame_id;
    std::mutex mutex;  ///< Mutex.

  private:
    int medioid_index_;          ///< The index of the representative descriptor (medioid).
    cv::Mat descriptors_;        ///< All map point descriptors.
    std::vector<Scalar> scores_; ///< Descriptor scores to determine medioid.
};

} // namespace deco
