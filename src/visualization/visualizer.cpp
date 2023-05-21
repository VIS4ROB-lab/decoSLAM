//
// Created by philipp on 28.06.22.
//

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <hyper/variables/cartesian.hpp>
#include <hyper/variables/groups/se3.hpp>

#include "visualization/helpers.hpp"
#include "visualization/visualizer.hpp"

namespace deco {
Visualizer::Visualizer(Id agent_id, ros::NodeHandle& nh)
    : id_{agent_id},
      edge_publisher_{nh.advertise<visualization_msgs::Marker>("/edges", 1)},
      node_publisher_{nh.advertise<visualization_msgs::MarkerArray>("/nodes", 1)},
      map_point_publisher_{nh.advertise<visualization_msgs::MarkerArray>("/map_points", 1)},
      pose_publisher_{nh.advertise<visualization_msgs::Marker>("/tracked_pose", 1)} {}

auto Visualizer::visualizeGraph(const std::map<StateId, std::pair<Translation, Color>>& positions, const std::vector<std::pair<StateId, StateId>>& edges) -> void {
    const auto [red, green, blue] = getColor(id_);
    const auto total = red + green + blue;
    visualization_msgs::Marker edge_line_list;
    edge_line_list.header.frame_id = "world";
    edge_line_list.header.stamp.fromNSec(current_timestamp_);
    edge_line_list.action = visualization_msgs::Marker::ADD;
    edge_line_list.pose.orientation.w = 1.0;
    edge_line_list.ns = "/agent_" + std::to_string(id_);
    edge_line_list.id = 0;
    edge_line_list.type = visualization_msgs::Marker::LINE_LIST;
    edge_line_list.scale.x = 0.04;
    edge_line_list.color.r = static_cast<float>(red) / total;
    edge_line_list.color.g = static_cast<float>(green) / total;
    edge_line_list.color.b = static_cast<float>(blue) / total;
    edge_line_list.color.a = 0.4;

    visualization_msgs::MarkerArray node_array;
    std::set<StateId> added_nodes;
    for (const auto& [id_1, id_2] : edges) {
        const auto& position_1 = positions.find(id_1)->second.first;
        if (!added_nodes.contains(id_1)) {
            const auto [red_1, green_1, blue_1] = positions.find(id_1)->second.second;
            auto max_color = std::max({red_1, green_1, blue_1});
            visualization_msgs::Marker node_1;
            node_1.header.frame_id = "world";
            node_1.header.stamp.fromNSec(current_timestamp_);
            node_1.action = visualization_msgs::Marker::ADD;
            node_1.pose.position.x = position_1.x();
            node_1.pose.position.y = position_1.y();
            node_1.pose.position.z = position_1.z();
            node_1.pose.orientation.x = 0.0;
            node_1.pose.orientation.y = 0.0;
            node_1.pose.orientation.z = 0.0;
            node_1.pose.orientation.w = 1.0;
            node_1.ns = "/agent_" + std::to_string(id_1.first);
            node_1.id = id_1.second;
            node_1.type = visualization_msgs::Marker::CUBE;
            node_1.scale.x = 0.15;
            node_1.scale.y = 0.15;
            node_1.scale.z = 0.15;
            node_1.color.r = static_cast<float>(red_1) / max_color;
            node_1.color.g = static_cast<float>(green_1) / max_color;
            node_1.color.b = static_cast<float>(blue_1) / max_color;
            node_1.color.a = 1.0;
            node_array.markers.push_back(node_1);
            added_nodes.insert(id_1);
        }

        const auto& position_2 = positions.find(id_2)->second.first;

        geometry_msgs::Point point_1, point_2;
        point_1.x = position_1.x();
        point_1.y = position_1.y();
        point_1.z = position_1.z();
        point_2.x = position_2.x();
        point_2.y = position_2.y();
        point_2.z = position_2.z();
        edge_line_list.points.push_back(point_1);
        edge_line_list.points.push_back(point_2);
    }

    edge_publisher_.publish(edge_line_list);
    node_publisher_.publish(node_array);
}

auto Visualizer::visualizeMapPoints(const std::vector<Translation>& positions, const std::vector<Color>& colors, const std::vector<StateId>& ids) -> void {
    visualization_msgs::MarkerArray marker_array_msg;
    for (auto i = 0; i < positions.size(); ++i) {
        auto [red, green, blue] = colors[i];
        auto max_color = std::max({red, green, blue});
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp.fromNSec(current_timestamp_);
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.position.x = positions[i].x();
        marker.pose.position.y = positions[i].y();
        marker.pose.position.z = positions[i].z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.ns = "/agent_" + std::to_string(ids[i].first);
        marker.id = ids[i].second;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = static_cast<float>(red) / max_color;
        marker.color.g = static_cast<float>(green) / max_color;
        marker.color.b = static_cast<float>(blue) / max_color;
        marker.color.a = 1.0;
        marker_array_msg.markers.push_back(marker);
    }
    map_point_publisher_.publish(marker_array_msg);
}

auto Visualizer::showCameraPose(const SE3& se3_reference, const SE3& se3_camera) -> void {
    const auto [red, green, blue] = getColor(id_);
    const auto total = red + green + blue;
    visualization_msgs::Marker camera;
    camera.type = visualization_msgs::Marker::LINE_LIST;
    camera.header.frame_id = "world";
    camera.header.stamp.fromNSec(current_timestamp_);
    camera.action = visualization_msgs::Marker::ADD;
    camera.ns = "agent_" + std::to_string(id_);
    camera.id = 0;
    camera.scale.x = 0.05;
    camera.scale.y = 0.05;
    camera.scale.z = 0.05;
    camera.pose.orientation.x = 0.0;
    camera.pose.orientation.y = 0.0;
    camera.pose.orientation.z = 0.0;
    camera.pose.orientation.w = 1.0;
    camera.color.r = static_cast<float>(red) / total;
    camera.color.g = static_cast<float>(green) / total;
    camera.color.b = static_cast<float>(blue) / total;
    camera.color.a = 1.0;
    geometry_msgs::Point reference, camera_origin;
    reference.x = se3_reference.translation().x();
    reference.y = se3_reference.translation().y();
    reference.z = se3_reference.translation().z();
    camera_origin.x = se3_camera.translation().x();
    camera_origin.y = se3_camera.translation().y();
    camera_origin.z = se3_camera.translation().z();
    camera.points.push_back(reference);
    camera.points.push_back(camera_origin);

    const auto camera_scale = 0.3;
    geometry_msgs::Point top_left, bottom_left, top_right, bottom_right;
    // Top left.
    const Position top_left_pos = se3_camera.vectorPlus(camera_scale * Position{-1.0, 1.0, 1.0});
    top_left.x = top_left_pos.x();
    top_left.y = top_left_pos.y();
    top_left.z = top_left_pos.z();
    camera.points.push_back(camera_origin);
    camera.points.push_back(top_left);

    // Bottom left.
    const Position bottom_left_pos = se3_camera.vectorPlus(camera_scale * Position{-1.0, -1.0, 1.0});
    bottom_left.x = bottom_left_pos.x();
    bottom_left.y = bottom_left_pos.y();
    bottom_left.z = bottom_left_pos.z();
    camera.points.push_back(camera_origin);
    camera.points.push_back(bottom_left);
    camera.points.push_back(bottom_left);
    camera.points.push_back(top_left);

    // Top right.
    const Position top_right_pos = se3_camera.vectorPlus(camera_scale * Position{1.0, 1.0, 1.0});
    top_right.x = top_right_pos.x();
    top_right.y = top_right_pos.y();
    top_right.z = top_right_pos.z();
    camera.points.push_back(camera_origin);
    camera.points.push_back(top_right);
    camera.points.push_back(top_right);
    camera.points.push_back(top_left);

    // Bottom right.
    const Position bottom_right_pos = se3_camera.vectorPlus(camera_scale * Position{1.0, -1.0, 1.0});
    bottom_right.x = bottom_right_pos.x();
    bottom_right.y = bottom_right_pos.y();
    bottom_right.z = bottom_right_pos.z();
    camera.points.push_back(camera_origin);
    camera.points.push_back(bottom_right);
    camera.points.push_back(bottom_right);
    camera.points.push_back(top_right);
    camera.points.push_back(bottom_right);
    camera.points.push_back(bottom_left);

    pose_publisher_.publish(camera);
}

auto Visualizer::deleteNodes(const std::set<StateId>& ids) -> void {
    visualization_msgs::MarkerArray marker_array_msg;
    for (const auto& id : ids) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp.fromNSec(current_timestamp_);
        marker.action = visualization_msgs::Marker::DELETE;
        marker.ns = "/agent_" + std::to_string(id.first);
        marker.id = id.second;
        marker_array_msg.markers.push_back(marker);
    }
    node_publisher_.publish(marker_array_msg);
}

auto Visualizer::deleteMapPoints(const std::set<StateId>& ids) -> void {
    visualization_msgs::MarkerArray marker_array_msg;
    for (const auto& id : ids) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp.fromNSec(current_timestamp_);
        marker.action = visualization_msgs::Marker::DELETE;
        marker.ns = "/agent_" + std::to_string(id.first);
        marker.id = id.second;
        marker_array_msg.markers.push_back(marker);
    }
    map_point_publisher_.publish(marker_array_msg);
}

auto Visualizer::setTimestamp(const Timestamp& timestamp) -> void {
    current_timestamp_ = timestamp;
}

} // namespace deco
