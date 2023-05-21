//
// Created by philipp on 27.07.22.
//

#include <hyper/variables/groups/se3.hpp>

#include "utils/common.hpp"

namespace deco {
auto computeEssentialMatrix(const std::vector<SE3>& se3_body_camera) -> Eigen::Matrix<Scalar, 3, 3> {
    const SE3 se3_left_right = se3_body_camera[0].groupInverse().groupPlus(se3_body_camera[1]);
    Eigen::Matrix<Scalar, 3, 3> essential_matrix_left_right = se3_left_right.translation().hat() * se3_left_right.rotation().toRotationMatrix();
    return essential_matrix_left_right;
}
} // namespace deco
