//
// Created by philipp on 28.06.22.
//

#pragma once

#include <ros/publisher.h>

#include "global.hpp"

namespace ros {
class NodeHandle;
class Publisher;
} // namespace ros

namespace deco {
class Visualizer {
  public:
    using Color = std::tuple<uint8_t, uint8_t, uint8_t>;
    Visualizer(Id agent_id, ros::NodeHandle& nh);
    auto visualizeGraph(const std::map<StateId, std::pair<Translation, Color>>& positions, const std::vector<std::pair<StateId, StateId>>& edges) -> void;
    auto visualizeMapPoints(const std::vector<Translation>& positions, const std::vector<Color>& colors, const std::vector<StateId>& ids) -> void;
    auto showCameraPose(const SE3& se3_reference, const SE3& se3_camera) -> void;
    auto deleteNodes(const std::set<StateId>& ids) -> void;
    auto deleteMapPoints(const std::set<StateId>& ids) -> void;
    auto setTimestamp(const Timestamp& timestamp) -> void;

  private:
    Id id_;
    Timestamp current_timestamp_;
    ros::Publisher edge_publisher_, node_publisher_, map_point_publisher_, pose_publisher_;
};
} // namespace deco
