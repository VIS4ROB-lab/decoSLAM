//
// Created by philipp on 24.11.22.
//

#include <gtest/gtest.h>
#include <cereal/archives/binary.hpp>
#include <hyper/variables/cartesian.hpp>
#include <hyper/variables/groups/se3.hpp>
#include <opencv2/core.hpp>

#include "global.hpp"
#include "messages/serialization.hpp"

namespace deco::tests {
template <typename TValue>
class SerializationTest : public testing::Test {
  public:
    using Value = TValue;
    static auto GetRandom() -> TValue;
    static auto isEqual(const Value& value_1, const Value& value_2) -> bool;
};

template <>
auto SerializationTest<StateId>::GetRandom() -> StateId {
    return {static_cast<Id>(std::rand()), static_cast<Id>(std::rand())};
}
template <>
auto SerializationTest<ObservationKey>::GetRandom() -> ObservationKey {
    return ObservationKey{
        .frame_id = {static_cast<Id>(std::rand()), static_cast<Id>(std::rand())},
        .map_point_id = {static_cast<Id>(std::rand()), static_cast<Id>(std::rand())},
        .camera_index = static_cast<size_t>(std::rand()),
        .keypoint_index = std::rand()};
}

template <>
auto SerializationTest<Position>::GetRandom() -> Position {
    return Position::Random();
}

template <>
auto SerializationTest<SE3>::GetRandom() -> SE3 {
    return SE3::Random();
}

template <typename TValue>
auto SerializationTest<TValue>::isEqual(const Value& value_1, const Value& value_2) -> bool {
    return value_1 == value_2;
}

template <>
auto SerializationTest<cv::Mat>::isEqual(const Value& value_1, const Value& value_2) -> bool {
    cv::Mat diff = value_1 != value_2;
    return cv::countNonZero(diff) == 0;
}

template <>
auto SerializationTest<cv::Mat>::GetRandom() -> cv::Mat {
    cv::Mat mat(2, 2, CV_32F);
    auto low = -10.0;
    auto high = 10.0;
    cv::randu(mat, cv::Scalar(low), cv::Scalar(high));
    return mat;
}

using TestTypes = ::testing::Types<StateId, ObservationKey, Position, SE3, cv::Mat>;
TYPED_TEST_SUITE(SerializationTest, TestTypes);

TYPED_TEST(SerializationTest, CheckConsistency) {
    for (auto i = 0; i < 1000; ++i) {
        typename TestFixture::Value value_in = this->GetRandom();
        std::stringstream stream;
        {
            cereal::BinaryOutputArchive output_archive(stream);
            output_archive(value_in);
        }

        typename TestFixture::Value value_out;
        {
            cereal::BinaryInputArchive input_archive(stream);
            input_archive(value_out);
        }
        EXPECT_TRUE(this->isEqual(value_in, value_out));
    }
}

} // namespace deco::tests
