//
// Created by philipp on 01.02.23.
//

#include <fstream>

#include <glog/logging.h>
#include <hyper/variables/groups/se3.hpp>

#include "global.hpp"

namespace {

auto readStateIdFromStream(std::stringstream& stream) -> deco::StateId {
    // remove leading whitespace.
    std::string str;
    std::getline(stream, str, '(');

    // Get the first number
    std::getline(stream, str, ',');
    const deco::Id first = std::stoull(str);

    // Get the second number
    std::getline(stream, str, ')');
    const deco::Id second = std::stoull(str);

    // Remove trailing comma
    std::getline(stream, str, ',');

    return {first, second};
}

auto readPoseFromStream(std::stringstream& stream) -> deco::SE3 {
    deco::SE3 pose;
    std::string str;
    // Read component-wise.
    std::getline(stream, str, ',');
    pose.translation().x() = std::stod(str);
    std::getline(stream, str, ',');
    pose.translation().y() = std::stod(str);
    std::getline(stream, str, ',');
    pose.translation().z() = std::stod(str);
    std::getline(stream, str, ',');
    pose.rotation().x() = std::stod(str);
    std::getline(stream, str, ',');
    pose.rotation().y() = std::stod(str);
    std::getline(stream, str, ',');
    pose.rotation().z() = std::stod(str);
    std::getline(stream, str, ',');
    pose.rotation().w() = std::stod(str);

    return pose;
}

auto getTimestampedIds(const deco::FilePath& path) -> std::map<deco::StateId, deco::Timestamp> {
    const auto file_path = path / "pose_id_map.csv";
    std::ifstream pose_id_file;
    pose_id_file.open(file_path);
    CHECK(pose_id_file.is_open()) << "Couldn't find or open file at " << file_path;

    std::string line;
    std::getline(pose_id_file, line); // Consume header
    auto timestamped_ids = std::map<deco::StateId, deco::Timestamp>();
    while (std::getline(pose_id_file, line)) {
        // Read timestamp.
        std::stringstream line_stream(line);
        std::string ts_str;
        std::getline(line_stream, ts_str, ',');
        const deco::Timestamp timestamp = std::stoull(ts_str);
        const deco::StateId id = readStateIdFromStream(line_stream);
        timestamped_ids.insert({id, timestamp});
    }
    return timestamped_ids;
}

auto getPosesWithId(const deco::FilePath& path) -> std::map<deco::StateId, deco::SE3> {
    const auto file_path = path / "poses.csv";
    std::ifstream pose_file;
    pose_file.open(file_path);
    CHECK(pose_file.is_open()) << "Couldn't find or open file at " << file_path;

    std::string line;
    std::getline(pose_file, line); // Consume header
    auto poses_with_id = std::map<deco::StateId, deco::SE3>();
    while (std::getline(pose_file, line)) {
        // Read timestamp.
        std::stringstream line_stream(line);
        const deco::StateId id = readStateIdFromStream(line_stream);
        const deco::SE3 pose = readPoseFromStream(line_stream);
        poses_with_id.insert({id, pose});
    }
    return poses_with_id;
}

auto getFolderPaths(const deco::FilePath& root_path) -> std::vector<deco::FilePath> {
    auto folder_paths = std::vector<deco::FilePath>();
    for (const auto& entry : std::filesystem::directory_iterator(root_path)) {
        if (is_directory(entry.path())) {
            LOG(INFO) << "Got path " << entry.path();
            folder_paths.push_back(entry.path());
        } else {
            LOG(INFO) << "Skipping path " << entry.path();
        }
    }
    return folder_paths;
}

auto writePosesToFile(const std::map<deco::Timestamp, deco::SE3>& poses, const deco::FilePath& path) -> void {
    const auto separator = " ";
    // const auto file_path = path / "combined_poses.csv";
    std::ofstream pose_file;
    pose_file << std::scientific << std::setprecision(18);
    pose_file.open(path);
    CHECK(pose_file.is_open()) << "Couldn't find or open file at " << path;

    // Header.
    pose_file << "#timestamp" << separator
              << "tx" << separator
              << "ty" << separator
              << "tz" << separator
              << "qx" << separator
              << "qy" << separator
              << "qz" << separator
              << "qw"
              << "\n";

    for (const auto& [timestamp, pose] : poses) {
        pose_file << timestamp / 1e9 << separator
                  << pose.translation().x() << separator
                  << pose.translation().y() << separator
                  << pose.translation().z() << separator
                  << pose.rotation().x() << separator
                  << pose.rotation().y() << separator
                  << pose.rotation().z() << separator
                  << pose.rotation().w() << "\n";
    }
    pose_file.close();
}

} // namespace

int main(int argc, char** argv) {
    // Set logging flags.
    FLAGS_log_dir = "../log/";
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = 2;
    google::InitGoogleLogging("combine_poses");

    CHECK(argc == 2) << "Usage: combine_poses <result_directory>";

    const deco::FilePath root_path = std::string(argv[1]);
    const auto agent_folder_paths = getFolderPaths(root_path);

    auto timestamped_ids = std::map<deco::StateId, deco::Timestamp>();
    for (const auto& path : agent_folder_paths) {
        auto agent_timestamped_ids = getTimestampedIds(path);
        timestamped_ids.insert(agent_timestamped_ids.begin(), agent_timestamped_ids.end());
    }

    auto timestamped_combined_poses = std::map<deco::Timestamp, deco::SE3>();
    auto timestamped_individual_poses = std::map<deco::Id, std::map<deco::Timestamp, deco::SE3>>();
    for (const auto& path : agent_folder_paths) {
        const auto poses_with_id = getPosesWithId(path);
        for (const auto& [id, pose] : poses_with_id) {
            const auto& timestamp = timestamped_ids.find(id)->second;
            // Combined poses.
            auto itr = timestamped_combined_poses.find(timestamp);
            if (itr == timestamped_combined_poses.end()) {
                timestamped_combined_poses.insert({timestamp, pose});
            } else {
                const auto& existing_pose = itr->second;
                const auto diff = (existing_pose.groupInverse().groupPlus(pose)).toTangent().norm();
                LOG(INFO) << "Found duplicate pose at " << timestamp << " with difference norm " << diff;
            }

            // Individual poses.
            auto itr_ind = timestamped_individual_poses.find(id.first);
            if (itr_ind == timestamped_individual_poses.end()) {
                itr_ind = timestamped_individual_poses.insert({id.first, {}}).first;
            }

            auto& agent_poses = itr_ind->second;
            auto itr_ts = agent_poses.find(timestamp);
            if (itr_ts == agent_poses.end()) {
                agent_poses.insert({timestamp, pose});
            } else {
                const auto& existing_pose = itr_ts->second;
                const auto diff = (existing_pose.groupInverse().groupPlus(pose)).toTangent().norm();
                LOG(INFO) << "Found duplicate pose at " << timestamp << " with difference norm " << diff;
            }
        }
    }

    for (const auto& [id, trajectory] : timestamped_individual_poses) {
        writePosesToFile(trajectory, root_path / ("poses_agent_" + std::to_string(id)));
    }

    writePosesToFile(timestamped_combined_poses, root_path / "combined_poses.csv");

    return 0;
}
