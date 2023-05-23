//
// Created by philipp on 20.06.22.
//

#pragma once
#include <vector>

#include "global.hpp"

namespace deco {

/// Filter a vector by indices
/// \tparam ElementType Storage type.
/// \param elements Input vector.
/// \param indices Indices to keep.
template <typename ElementType>
auto filterByIndex(std::vector<ElementType>& elements, const std::vector<int>& indices) -> void {
    std::vector<ElementType> tmp;
    tmp.reserve(indices.size());
    for (const auto& index : indices) {
        tmp.emplace_back(elements[index]);
    }
    elements = std::move(tmp);
}

/// Filter a vector by a condition.
/// \tparam ElementType Storage type.
/// \tparam ConditionType Type of condition, must be convertible to bool.
/// \param elements Input vector (will be modified).
/// \param C Vector of conditions of the same length as input.
template <typename ElementType, typename ConditionType>
auto filterByCondition(std::vector<ElementType>& elements, const std::vector<ConditionType>& C) -> void {
    // Sanity check.
    CHECK(elements.size() == C.size());

    // Select elements by condition.
    auto last = 0;
    for (auto i = 0; i < elements.size(); ++i) {
        if (C[i]) elements[last++] = std::move(elements[i]);
    }

    // Shrink vector.
    elements.erase(elements.cbegin() + last, elements.cend());
}

/// Computes the essential matrix. Currently only 2 cameras are supported.
/// \param se3_body_camera Vector of camera transformations (must have length 2).
/// \return 3x3 essential matrix.
auto computeEssentialMatrix(const std::vector<SE3>& se3_body_camera) -> Eigen::Matrix<Scalar, 3, 3>;

} // namespace deco
