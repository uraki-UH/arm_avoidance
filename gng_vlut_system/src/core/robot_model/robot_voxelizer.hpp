#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <unordered_set>
#include <iostream>

#include "robot_model/robot_model.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "core/common/voxelizer_engine.hpp"
#include "safety_engine/recognition/urdf_geometry_simplifier.hpp"

namespace simulation {

/**
 * @brief リンクごとのボクセルデータ（ローカル座標系）
 */
struct LinkVoxelData {
    std::string name;
    std::vector<Eigen::Vector3d> local_voxel_centers;
    std::vector<Eigen::Vector3d> local_mesh_triangles;
    std::vector<Eigen::Vector3d> local_primitive_voxel_centers;
    Eigen::Vector3d local_min;
    Eigen::Vector3d local_max;
};

/**
 * @brief RobotModelからボクセル表現を構築する汎用クラス
 */
class RobotVoxelizer {
public:
    // Backward compatibility wrapper
    static std::vector<LinkVoxelData> build(const RobotModel& model, 
                                            std::shared_ptr<::kinematics::KinematicChain> chain,
                                            const ::GNG::Analysis::IndexVoxelGrid& grid,
                                            const std::vector<std::string>& exclude_links = {},
                                            double padding = 0.0) {
        std::vector<std::string> target_links;
        for (int i = 0; i < chain->getNumJoints(); ++i) {
            target_links.push_back(chain->getLinkName(i));
        }
        std::unordered_set<std::string> exclude_set(exclude_links.begin(), exclude_links.end());
        return build(model, target_links, grid, exclude_set, padding);
    }

    // New signature for all-link voxelization
    static std::vector<LinkVoxelData> build(const RobotModel& model, 
                                            const std::vector<std::string>& target_links,
                                            const ::GNG::Analysis::IndexVoxelGrid& grid,
                                            const std::unordered_set<std::string>& exclude_set = {},
                                            double padding = 0.0) {
        std::vector<LinkVoxelData> link_data_list;
        double voxel_size = grid.getVoxelSize();
        // ... (rest of implementation below)

        for (const auto& link_name : target_links) {
            if (exclude_set.count(link_name)) {
                std::cout << "[Voxelizer] Skipping excluded link: " << link_name << std::endl;
                continue;
            }

            const LinkProperties* props = model.getLink(link_name);
            if (!props) {
                std::cout << "[Voxelizer] Link NOT found in RobotModel: " << link_name << std::endl;
                continue;
            }

            if (props->collisions.empty()) {
                std::cout << "[Voxelizer] Link has NO collision geometries: " << link_name << std::endl;
                continue;
            }
            LinkVoxelData data;
            data.name = link_name;
            data.local_min.setConstant(std::numeric_limits<double>::max());
            data.local_max.setConstant(std::numeric_limits<double>::lowest());

            std::unordered_set<long> primitive_vids;
            std::unordered_set<long> mesh_vids;
            bool has_mesh = false;
            for (const auto& col : props->collisions) {
                if (col.geometry.type == GeometryType::MESH) {
                    has_mesh = true;
                } else if (col.geometry.type == GeometryType::BOX) {
                    ::robot_sim::common::VoxelizerEngine::voxelizeBox(col.geometry.size + Eigen::Vector3d::Constant(padding * 2.0), col.origin, grid, primitive_vids);
                } else if (col.geometry.type == GeometryType::SPHERE) {
                    ::robot_sim::common::VoxelizerEngine::voxelizeSphere(col.geometry.size.x() + padding, col.origin, grid, primitive_vids);
                } else if (col.geometry.type == GeometryType::CYLINDER) {
                    ::robot_sim::common::VoxelizerEngine::voxelizeCylinder(col.geometry.size.x() + padding, col.geometry.size.y() + padding * 2.0, col.origin, grid, primitive_vids);
                }
            }

            if (has_mesh) {
                auto slink = ::robot_sim::recognition::UrdfGeometrySimplifier::simplifyLink(*props);
                for (size_t m_idx = 0; m_idx < slink.meshes.size(); ++m_idx) {
                    const auto& mesh = slink.meshes[m_idx];
                    const auto& origin = slink.mesh_origins[m_idx];
                    
                    // Transform all vertices first
                    std::vector<Eigen::Vector3d> transformed_verts;
                    transformed_verts.reserve(mesh.vertices.size() / 3);
                    for (size_t i = 0; i < mesh.vertices.size() / 3; ++i) {
                        Eigen::Vector3d v(mesh.vertices[i*3], mesh.vertices[i*3+1], mesh.vertices[i*3+2]);
                        if (padding > 0.0 && v.norm() > 1e-6) {
                            v += v.normalized() * padding;
                        }
                        transformed_verts.push_back(origin * v);
                    }

                    // Voxelize each triangle using the indices
                    std::vector<Eigen::Vector3d> triangle_soup;
                    triangle_soup.reserve(mesh.indices.size());
                    for (size_t i = 0; i < mesh.indices.size() / 3; ++i) {
                        triangle_soup.push_back(transformed_verts[mesh.indices[i*3 + 0]]);
                        triangle_soup.push_back(transformed_verts[mesh.indices[i*3 + 1]]);
                        triangle_soup.push_back(transformed_verts[mesh.indices[i*3 + 2]]);
                    }

                    if (!triangle_soup.empty()) {
                        data.local_mesh_triangles.insert(
                            data.local_mesh_triangles.end(),
                            triangle_soup.begin(), triangle_soup.end());
                        ::robot_sim::common::VoxelizerEngine::voxelizeMeshTriangles(
                            triangle_soup, grid, mesh_vids);
                    }
                }
            }

            std::unordered_set<long> vids = primitive_vids;
            vids.insert(mesh_vids.begin(), mesh_vids.end());
            for (long vid : vids) {
                Eigen::Vector3i idx = grid.getIndexFromFlatId(vid);
                Eigen::Vector3d p = ::common::geometry::VoxelUtils::voxelToWorld(idx, (float)voxel_size).template cast<double>();
                data.local_voxel_centers.push_back(p);
                data.local_min = data.local_min.cwiseMin(p);
                data.local_max = data.local_max.cwiseMax(p);
            }

            data.local_primitive_voxel_centers.reserve(primitive_vids.size());
            for (long vid : primitive_vids) {
                const Eigen::Vector3i idx = grid.getIndexFromFlatId(vid);
                data.local_primitive_voxel_centers.push_back(
                    ::common::geometry::VoxelUtils::voxelToWorld(
                        idx, static_cast<float>(voxel_size)).template cast<double>());
            }

            if (!data.local_voxel_centers.empty()) {
                link_data_list.push_back(data);
                std::cout << "[Voxelizer]   Built: " << link_name << " (" << data.local_voxel_centers.size() << " voxels)" << std::endl;
            }
        }
        return link_data_list;
    }
};

} // namespace simulation
