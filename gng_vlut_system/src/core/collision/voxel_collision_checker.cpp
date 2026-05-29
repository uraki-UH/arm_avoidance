#include "collision/voxel_collision_checker.hpp"
#include "common/voxel_utils.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include <algorithm>

namespace simulation {

VoxelCollisionChecker::VoxelCollisionChecker(const RobotModel& model, 
                                             const kinematics::KinematicChain& chain, 
                                             double voxel_size,
                                             double padding)
    : model_(model), chain_(chain), voxel_size_(voxel_size) {
    
    ::GNG::Analysis::IndexVoxelGrid grid(voxel_size);
    
    // 衝突形状を持つ全リンクを抽出
    std::vector<std::string> collision_links;
    for (const auto& [name, props] : model.getLinks()) {
        if (!props.collisions.empty()) {
            collision_links.push_back(name);
        }
    }

    auto voxel_data = RobotVoxelizer::build(model, collision_links, grid, {}, padding);

    // 自己衝突では、URDF上で joint で直接つながる隣接リンクは除外する。
    // これは geometric self checker の挙動に合わせるため。
    for (const auto& [joint_name, joint] : model.getJoints()) {
        addCollisionExclusion(joint.parent_link, joint.child_link);
    }
    
    // マネージャの初期化（共有ロジックを利用）
    auto chain_ptr = std::make_shared<kinematics::KinematicChain>(chain);
    auto model_ptr = std::make_shared<simulation::RobotModel>(model);
    manager_.initialize(chain_ptr, model_ptr, voxel_data, voxel_size);
}

void VoxelCollisionChecker::updateBodyPoses(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
    const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& orientations) {
    
    // KinematicChain の全リンクのトランスフォームを構築
    std::map<std::string, Eigen::Isometry3d> link_tfs;
    auto fixed_info = model_.getFixedLinkInfo();
    chain_.buildAllLinkTransforms(positions, orientations, fixed_info, link_tfs);

    augmentBranchLinkTransforms(link_tfs);

    const auto& link_data = manager_.getLinkVoxelDataList();
    current_tfs_.clear();
    current_tfs_.reserve(link_data.size());
    for (const auto& data : link_data) {
        auto it = link_tfs.find(data.name);
        if (it != link_tfs.end()) {
            current_tfs_.push_back(it->second);
        } else {
            current_tfs_.push_back(Eigen::Isometry3d::Identity());
        }
    }

    cached_link_voxel_masks_ = computeLinkVoxelMasks();
    cached_link_voxel_masks_valid_ = true;
}

void VoxelCollisionChecker::addCollisionExclusion(const std::string& link1, const std::string& link2) {
    std::string l1 = link1, l2 = link2;
    if (l1 > l2) std::swap(l1, l2);
    exclusion_pairs_.insert({l1, l2});
}

void VoxelCollisionChecker::addEnvironmentIgnoreLink(const std::string& link_name) {
    environment_ignore_links_.insert(link_name);
}

double VoxelCollisionChecker::getJointValueHint(const std::string& joint_name) const {
    auto it = joint_value_hints_.find(joint_name);
    if (it == joint_value_hints_.end()) {
        return 0.0;
    }
    return it->second;
}

Eigen::Isometry3d VoxelCollisionChecker::computeJointMotionTransform(
    const simulation::JointProperties& joint, double joint_value) const {
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    switch (joint.type) {
    case kinematics::JointType::Revolute:
      motion.rotate(Eigen::AngleAxisd(joint_value, joint.axis.normalized()));
      break;
    case kinematics::JointType::Prismatic:
      motion.translate(joint.axis.normalized() * joint_value);
      break;
    default:
      break;
    }
    return motion;
}

void VoxelCollisionChecker::augmentBranchLinkTransforms(
    std::map<std::string, Eigen::Isometry3d>& link_tfs) const {
    bool changed = true;
    const auto& joints = model_.getJoints();
    std::size_t iter = 0;
    while (changed && iter < joints.size() + 1) {
        changed = false;
        ++iter;
        for (const auto& [joint_name, joint] : joints) {
            (void)joint_name;
            if (link_tfs.count(joint.child_link) > 0) {
                continue;
            }
            auto parent_it = link_tfs.find(joint.parent_link);
            if (parent_it == link_tfs.end()) {
                continue;
            }

            double joint_value = 0.0;
            if (joint.has_mimic) {
                const double source_value = getJointValueHint(joint.mimic_joint_name);
                joint_value = source_value * joint.mimic_multiplier + joint.mimic_offset;
            } else {
                joint_value = getJointValueHint(joint.name);
            }

            link_tfs[joint.child_link] =
                parent_it->second * joint.origin *
                computeJointMotionTransform(joint, joint_value);
            changed = true;
        }
    }
}

std::vector<std::vector<long>> VoxelCollisionChecker::getLinkVoxelMasks() const {
    return getCachedLinkVoxelMasks();
}

std::vector<std::pair<std::string, Eigen::Isometry3d>>
VoxelCollisionChecker::getCurrentLinkTransforms() const {
    std::vector<std::pair<std::string, Eigen::Isometry3d>> named_tfs;
    const auto& link_data = manager_.getLinkVoxelDataList();
    named_tfs.reserve(link_data.size());
    for (std::size_t i = 0; i < link_data.size(); ++i) {
        const auto& data = link_data[i];
        Eigen::Isometry3d tf = (i < current_tfs_.size()) ? current_tfs_[i] : Eigen::Isometry3d::Identity();
        named_tfs.emplace_back(data.name, tf);
    }
    return named_tfs;
}

std::vector<std::pair<std::string, std::string>>
VoxelCollisionChecker::collectSelfCollisionPairs() const {
    std::vector<std::pair<std::string, std::string>> pairs;
    const auto& link_data = manager_.getLinkVoxelDataList();
    const auto& link_vids = getCachedLinkVoxelMasks();

    for (size_t i = 0; i < link_data.size(); ++i) {
        const auto& n1_raw = link_data[i].name;
        if (environment_ignore_links_.count(n1_raw) > 0) {
            continue;
        }
        for (size_t j = i + 1; j < link_data.size(); ++j) {
            const auto& n2_raw = link_data[j].name;
            if (environment_ignore_links_.count(n2_raw) > 0) {
                continue;
            }

            std::string n1 = n1_raw;
            std::string n2 = n2_raw;
            if (n1 > n2) std::swap(n1, n2);
            if (exclusion_pairs_.count({n1, n2})) {
                continue;
            }

            const auto& vids1 = link_vids[i];
            const auto& vids2 = link_vids[j];
            auto it1 = vids1.begin();
            auto it2 = vids2.begin();
            while (it1 != vids1.end() && it2 != vids2.end()) {
                if (*it1 == *it2) {
                    pairs.emplace_back(n1, n2);
                    break;
                }
                if (*it1 < *it2) ++it1;
                else ++it2;
            }
        }
    }

    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    return pairs;
}

std::vector<std::vector<long>> VoxelCollisionChecker::computeLinkVoxelMasks() const {
    std::vector<std::vector<long>> link_vids;
    const auto& link_data = manager_.getLinkVoxelDataList();
    link_vids.resize(link_data.size());

    auto* grid = manager_.getIndexGrid();
    if (!grid || current_tfs_.size() != link_data.size()) {
        return link_vids;
    }

    for (size_t i = 0; i < link_data.size(); ++i) {
        const auto& data = link_data[i];
        const auto& tf = current_tfs_[i];
        auto& vids = link_vids[i];
        vids.reserve(data.local_voxel_centers.size());
        for (const auto& lp : data.local_voxel_centers) {
            Eigen::Vector3d wp = tf * lp;
            vids.push_back(grid->getFlatVoxelId(
                ::common::geometry::VoxelUtils::worldToVoxel(
                    wp.template cast<float>(), (float)voxel_size_)));
        }
        if (vids.size() > 1) {
            ::common::geometry::VoxelUtils::radixSort(vids);
            vids.erase(std::unique(vids.begin(), vids.end()), vids.end());
        }
    }

    return link_vids;
}

const std::vector<std::vector<long>>&
VoxelCollisionChecker::getCachedLinkVoxelMasks() const {
    if (!cached_link_voxel_masks_valid_) {
        cached_link_voxel_masks_ = computeLinkVoxelMasks();
        cached_link_voxel_masks_valid_ = true;
    }
    return cached_link_voxel_masks_;
}

bool VoxelCollisionChecker::checkCollision() {
    const auto& link_data = manager_.getLinkVoxelDataList();

    // 1. 床との衝突判定
    if (ground_z_threshold_ > -1000.0) {
        for (size_t i = 0; i < link_data.size(); ++i) {
            const auto& data = link_data[i];
            if (environment_ignore_links_.count(data.name) > 0) {
                continue;
            }
            const auto& tf = current_tfs_[i];
            
            // AABBでまずチェック
            Eigen::Vector3d world_min = tf * data.local_min;
            Eigen::Vector3d world_max = tf * data.local_max;
            
            if (world_min.z() < ground_z_threshold_ || world_max.z() < ground_z_threshold_) {
                for (const auto& lp : data.local_voxel_centers) {
                    if ((tf * lp).z() < ground_z_threshold_) return true;
                }
            }
        }
    }

    // 2. 環境障害物との衝突判定
    if (env_checker_) {
        for (size_t i = 0; i < link_data.size(); ++i) {
            const auto& data = link_data[i];
            if (environment_ignore_links_.count(data.name) > 0) {
                continue;
            }
            const auto& tf = current_tfs_[i];
            for (const auto& lp : data.local_voxel_centers) {
                Eigen::Vector3d wp = tf * lp;
                collision::SelfCollisionChecker::CollisionObject p_obj;
                p_obj.id = -1;
                p_obj.name.clear();
                p_obj.type = collision::SelfCollisionChecker::ShapeType::SPHERE;
                p_obj.sphere.center = wp;
                p_obj.sphere.radius = 0.0;
                p_obj.is_fixed_to_base = false;
                if (env_checker_->checkCollision(p_obj)) return true;
            }
        }
    }

    // 3. 自己干渉判定
    if (enable_self_collision_) {
        // 現在姿勢に基づいてリンクごとのボクセルIDリストを作る
        const auto& link_vids = getCachedLinkVoxelMasks();
        
        for (size_t i = 0; i < link_data.size(); ++i) {
            const auto& n1_raw = link_data[i].name;
            if (environment_ignore_links_.count(n1_raw) > 0) {
                continue;
            }
            for (size_t j = i + 1; j < link_data.size(); ++j) {
                const auto& n2_raw = link_data[j].name;
                if (environment_ignore_links_.count(n2_raw) > 0) {
                    continue;
                }

                std::string n1 = n1_raw;
                std::string n2 = n2_raw;
                if (n1 > n2) std::swap(n1, n2);
                if (exclusion_pairs_.count({n1, n2})) continue;

                // ボクセルIDの重なりチェック（ソート済みなので高速）
                const auto& vids1 = link_vids[i];
                const auto& vids2 = link_vids[j];
                auto it1 = vids1.begin();
                auto it2 = vids2.begin();
                while (it1 != vids1.end() && it2 != vids2.end()) {
                    if (*it1 == *it2) return true;
                    if (*it1 < *it2) ++it1; else ++it2;
                }
            }
        }
    }

    return false;
}

} // namespace simulation
