//
// Created by philipp on 24.11.22.
//
#pragma once

#include "global.hpp"

namespace cereal {

// std::pair
template <class Archive, typename T, typename U>
auto save(Archive& archive, const std::pair<T, U>& data) -> void {
    archive(data.first, data.second);
}

template <class Archive, typename T, typename U>
auto load(Archive& archive, std::pair<T, U>& data) -> void {
    archive(data.first, data.second);
}

// deco::ObservationKey
template <class Archive>
auto save(Archive& archive, const deco::ObservationKey& key) -> void {
    archive(key.frame_id, key.map_point_id, key.camera_index, key.keypoint_index);
}

template <class Archive>
auto load(Archive& archive, deco::ObservationKey& key) -> void {
    archive(key.frame_id, key.map_point_id, key.camera_index, key.keypoint_index);
}

// cv::Mat.
// Adapted from COVINS.
// https://github.com/VIS4ROB-lab/covins/blob/master/covins_comm/include/covins/covins_base/msgs/msg_keyframe.hpp
template <class Archive>
auto save(Archive& archive, const cv::Mat& matrix) -> void {
    const auto rows = matrix.rows;
    const auto cols = matrix.cols;
    const auto type = matrix.type();
    const auto is_continuous = matrix.isContinuous();

    archive(rows, cols, type, is_continuous);

    if (is_continuous) {
        const int data_size = rows * cols * static_cast<int>(matrix.elemSize());
        archive(binary_data(matrix.ptr(), data_size));
    } else {
        const int row_size = cols * static_cast<int>(matrix.elemSize());
        for (int i = 0; i < rows; i++) {
            archive(binary_data(matrix.ptr(i), row_size));
        }
    }
}

template <class Archive>
auto load(Archive& archive, cv::Mat& matrix) -> void {
    int rows, cols, type;
    bool is_continuous;
    archive(rows, cols, type, is_continuous);

    if (is_continuous) {
        matrix.create(rows, cols, type);
        const int data_size = rows * cols * static_cast<int>(matrix.elemSize());
        archive(binary_data(matrix.ptr(), data_size));
    } else {
        matrix.create(rows, cols, type);
        const int row_size = cols * static_cast<int>(matrix.elemSize());
        for (int i = 0; i < rows; i++) {
            archive(binary_data(matrix.ptr(i), row_size));
        }
    }
}

// cv::KeyPoint.
template <class Archive>
auto serialize(Archive& archive, cv::KeyPoint& keypoint) -> void {
    archive(keypoint.pt.x, keypoint.pt.y, keypoint.response, keypoint.size, keypoint.angle, keypoint.class_id);
}


// hyper::AbstractVariable.
template <class Archive, typename TScalar>
auto save(Archive& archive, const hyper::AbstractVariable<TScalar>& variable) -> void {
    archive(binary_data(variable.asVector().data(), static_cast<std::size_t>(variable.asVector().size() * sizeof(TScalar))));
}

template <class Archive, typename TScalar>
auto load(Archive& archive, hyper::AbstractVariable<TScalar>& variable) -> void {
    archive(binary_data(variable.asVector().data(), static_cast<std::size_t>(variable.asVector().size() * sizeof(TScalar))));
}

// Eigen::Matrix.
// Adapted from COVINS.
// https://github.com/VIS4ROB-lab/covins/blob/master/covins_comm/include/covins/covins_base/msgs/msg_keyframe.hpp
/*template <class Archive, typename TScalar, int NumRows, int NumCols, int Options, int MaxNumRows, int MaxNumCols>
auto save(Archive& archive, const Eigen::Matrix<TScalar, NumRows, NumCols, Options, MaxNumRows, MaxNumCols>& matrix) -> void {
    const auto rows = matrix.rows;
    const auto cols = matrix.cols;
    archive(rows, cols, binary_data(matrix.data(), rows * cols * sizeof(TScalar)));
}

template <class Archive, typename TScalar, int NumRows, int NumCols, int Options, int MaxNumRows, int MaxNumCols>
auto load(Archive& archive, Eigen::Matrix<TScalar, NumRows, NumCols, Options, MaxNumRows, MaxNumCols>& matrix) -> void {
    std::int32_t rows;
    std::int32_t cols;
    archive(rows, cols);
    matrix.resize(rows, cols);
    archive(binary_data(matrix.data(), static_cast<std::size_t>(rows * cols * sizeof(TScalar))));
}*/

} // namespace cereal
