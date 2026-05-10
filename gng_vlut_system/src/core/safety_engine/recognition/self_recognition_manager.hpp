#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <map>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "robot_model/robot_model.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "safety_engine/recognition/urdf_geometry_simplifier.hpp"
#include "safety_engine/recognition/primitive_spatial_mapper.hpp"
#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"

namespace robot_sim {
namespace recognition {

struct LinkVoxelData {
    std::string name;
    std::vector<Eigen::Vector3d> local_voxel_centers;
    Eigen::Vector3d local_min;
    Eigen::Vector3d local_max;
};

class SelfRecognitionManager {
public:
    // 第4引数に確実にデフォルト値を設定し、引数3つでも呼べるようにする
    void initialize(const simulation::RobotModel& model, 
                    std::shared_ptr<kinematics::KinematicChain> chain,
                    double voxel_size,
                    const std::vector<std::string>& exclude_links = std::vector<std::string>()) {
        chain_ = chain;
        voxel_size_ = voxel_size;
        link_data_list_.clear();

        std::unordered_set<std::string> exclude_set(exclude_links.begin(), exclude_links.end());
        
        std::cout << "[Recognition] Initializing Manager (Name-based)." << std::endl;

        for (int i = 0; i < chain_->getNumJoints(); ++i) {
            std::string full_name = chain_->getLinkName(i);
            if (exclude_set.count(full_name)) continue;

            const simulation::LinkProperties* props = findLinkProperties(model, full_name);
            if (!props || props->collisions.empty()) continue;

            auto slink = UrdfGeometrySimplifier::simplifyLink(*props);
            
            LinkVoxelData data;
            data.name = full_name;
            data.local_min.setConstant(std::numeric_limits<double>::max());
            data.local_max.setConstant(std::numeric_limits<double>::lowest());

            std::unordered_set<long> vids;
            for (size_t m_idx = 0; m_idx < slink.meshes.size(); ++m_idx) {
                const auto& mesh = slink.meshes[m_idx];
                const auto& origin = slink.mesh_origins[m_idx];
                for (size_t t = 0; t < mesh.vertices.size() / 9; ++t) {
                    Eigen::Vector3d v[3];
                    for(int k=0; k<3; ++k) v[k] = origin * Eigen::Vector3d(mesh.vertices[t*9+k*3], mesh.vertices[t*9+k*3+1], mesh.vertices[t*9+k*3+2]);
                    double d12 = (v[1]-v[0]).norm(), d13 = (v[2]-v[0]).norm();
                    int s1 = std::max(1, (int)std::ceil(d12/voxel_size_)), s2 = std::max(1, (int)std::ceil(d13/voxel_size_));
                    for(int i1=0; i1<=s1; ++i1) {
                        double t1 = (double)i1/s1;
                        for(int i2=0; i2<=std::max(0, (int)(s2*(1.0-t1))); ++i2) {
                            double t2 = (double)i2/s2;
                            Eigen::Vector3d p = v[0] + t1*(v[1]-v[0]) + t2*(v[2]-v[0]);
                            vids.insert(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)voxel_size_)));
                        }
                    }
                }
            }

            for (long vid : vids) {
                Eigen::Vector3i idx = ::GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
                Eigen::Vector3d p = ::common::geometry::VoxelUtils::voxelToWorld(idx, (float)voxel_size_).cast<double>();
                data.local_voxel_centers.push_back(p);
                data.local_min = data.local_min.cwiseMin(p);
                data.local_max = data.local_max.cwiseMax(p);
            }

            if (!data.local_voxel_centers.empty()) {
                link_data_list_.push_back(data);
                std::cout << "[Recognition]   Mapped: " << full_name << std::endl;
            }
        }
    }

    void updateRobotState(const std::vector<double>& joints) {
        if (chain_) chain_->updateKinematics(joints);
        current_joints_ = joints;
    }

    std::map<std::string, Eigen::Isometry3d> getCurrentLinkTransforms() {
        if (!chain_) return {};
        std::map<std::string, Eigen::Isometry3d> link_tfs;
        chain_->buildAllLinkTransforms(
            chain_->getLinkPositions(), 
            chain_->getLinkOrientations(), 
            {}, 
            link_tfs
        );
        return link_tfs;
    }

    std::vector<long> getSelfVoxelMask() {
        auto tfs = getCurrentLinkTransforms();
        if (tfs.empty()) return {};

        std::vector<long> all_vids;
        for (const auto& data : link_data_list_) {
            if (tfs.count(data.name)) {
                const auto& tf = tfs.at(data.name);
                for (const auto& lp : data.local_voxel_centers) {
                    Eigen::Vector3d wp = tf * lp;
                    all_vids.push_back(::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(::common::geometry::VoxelUtils::worldToVoxel(wp.cast<float>(), (float)voxel_size_)));
                }
            }
        }
        std::sort(all_vids.begin(), all_vids.end());
        all_vids.erase(std::unique(all_vids.begin(), all_vids.end()), all_vids.end());
        return all_vids;
    }

    std::vector<long> getSelfVoxelMask(const std::vector<double>& joints) {
        updateRobotState(joints);
        return getSelfVoxelMask();
    }

    struct VisualizationMarkers {
        visualization_msgs::msg::MarkerArray voxels;
        visualization_msgs::msg::MarkerArray aabb;
    };

    VisualizationMarkers getVisualizationMarkers(const std::string& frame_id, double time_sec) {
        VisualizationMarkers msg;
        auto tfs = getCurrentLinkTransforms();
        if (tfs.empty()) return msg;

        int marker_id = 0;
        for (const auto& data : link_data_list_) {
            if (tfs.count(data.name)) {
                const auto& tf = tfs.at(data.name);
                
                visualization_msgs::msg::Marker m;
                m.header.frame_id = frame_id; m.header.stamp.sec = (int32_t)time_sec;
                m.header.stamp.nanosec = static_cast<uint32_t>((time_sec - (double)m.header.stamp.sec) * 1e9);
                m.ns = "self_voxels"; m.id = marker_id++;
                m.type = visualization_msgs::msg::Marker::CUBE_LIST;
                m.scale.x = m.scale.y = m.scale.z = voxel_size_;
                m.color.g = 1.0f; m.color.a = 0.5f; m.pose.orientation.w = 1.0;
                for (const auto& lp : data.local_voxel_centers) {
                    Eigen::Vector3d wp = tf * lp;
                    geometry_msgs::msg::Point p; p.x = wp.x(); p.y = wp.y(); p.z = wp.z();
                    m.points.push_back(p);
                }
                if (!m.points.empty()) msg.voxels.markers.push_back(m);

                visualization_msgs::msg::Marker b;
                b.header = m.header; b.ns = "self_aabb"; b.id = marker_id++;
                b.type = visualization_msgs::msg::Marker::LINE_LIST;
                b.action = visualization_msgs::msg::Marker::ADD;
                b.scale.x = 0.002; b.color.r = 1.0f; b.color.g = 0.5f; b.color.a = 0.8f; b.pose.orientation.w = 1.0;
                Eigen::Vector3d corners[8];
                for(int i=0; i<8; ++i) {
                    Eigen::Vector3d c = data.local_min;
                    if(i&1) c.x()=data.local_max.x(); if(i&2) c.y()=data.local_max.y(); if(i&4) c.z()=data.local_max.z();
                    corners[i] = tf * c;
                }
                int ed[12][2]={{0,1},{1,3},{3,2},{2,0},{4,5},{5,7},{7,6},{6,4},{0,4},{1,5},{2,6},{3,7}};
                for(auto& e:ed) {
                    geometry_msgs::msg::Point p1,p2;
                    p1.x=corners[e[0]].x(); p1.y=corners[e[0]].y(); p1.z=corners[e[0]].z();
                    p2.x=corners[e[1]].x(); p2.y=corners[e[1]].y(); p2.z=corners[e[1]].z();
                    b.points.push_back(p1); b.points.push_back(p2);
                }
                msg.aabb.markers.push_back(b);
            }
        }
        return msg;
    }

    VisualizationMarkers getVisualizationMarkers(const std::vector<double>& joints, const std::string& frame_id, double time_sec) {
        updateRobotState(joints);
        return getVisualizationMarkers(frame_id, time_sec);
    }

    void filterPointCloud(const std::vector<Eigen::Vector3d>& all_points, const std::vector<double>& joints, 
                          std::vector<Eigen::Vector3d>& filtered, std::vector<Eigen::Vector3d>& self) {
        updateRobotState(joints);
        auto vids = getSelfVoxelMask();
        std::unordered_set<long> vset(vids.begin(), vids.end());
        filtered.clear(); self.clear();
        for (const auto& p : all_points) {
            long vid = ::GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(::common::geometry::VoxelUtils::worldToVoxel(p.cast<float>(), (float)voxel_size_));
            if (vset.count(vid)) self.push_back(p); else filtered.push_back(p);
        }
    }

    const std::vector<LinkVoxelData>& getLinkVoxelDataList() const { return link_data_list_; }
    std::shared_ptr<kinematics::KinematicChain> getKinematicChain() { return chain_; }

private:
    const simulation::LinkProperties* findLinkProperties(const simulation::RobotModel& model, const std::string& name) {
        if (model.getLinks().count(name)) return &model.getLinks().at(name);
        for (const auto& [m_name, m_props] : model.getLinks()) {
            if (name.size() > m_name.size() && name.compare(name.size() - m_name.size(), m_name.size(), m_name) == 0) return &m_props;
        }
        return nullptr;
    }

    std::shared_ptr<kinematics::KinematicChain> chain_;
    double voxel_size_;
    std::vector<LinkVoxelData> link_data_list_;
    std::vector<double> current_joints_;
};

} // namespace recognition
} // namespace robot_sim
