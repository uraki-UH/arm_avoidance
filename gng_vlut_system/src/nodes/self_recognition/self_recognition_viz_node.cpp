#include "self_recognition/self_recognition_viz_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <cctype>
#include <algorithm>
#include <map>
#include <chrono>
#include <functional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "common/resource_utils.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "core/common/constants.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

static std::string getStringWithFallback(
    rclcpp::Node &node, const std::string &nested_key,
    const std::string &legacy_key) {
    const std::string nested = node.get_parameter(nested_key).as_string();
    if (!nested.empty()) {
        return nested;
    }
    return node.get_parameter(legacy_key).as_string();
}

static std::vector<std::string> getStringsWithFallback(
    rclcpp::Node &node, const std::string &nested_key,
    const std::string &legacy_key) {
    auto nested = node.get_parameter(nested_key).as_string_array();
    if (!nested.empty()) {
        return nested;
    }
    return node.get_parameter(legacy_key).as_string_array();
}

static std::string findParentLink(
    const simulation::RobotModel &model, const std::string &child_link) {
    for (const auto &[joint_name, joint_props] : model.getJoints()) {
        if (joint_props.child_link == child_link) {
            return joint_props.parent_link;
        }
    }
    return "";
}

static std::vector<std::string> collectTerminalLeafLinks(
    const simulation::RobotModel &model) {
    std::unordered_set<std::string> parent_links;
    for (const auto &[joint_name, joint_props] : model.getJoints()) {
        (void)joint_name;
        parent_links.insert(joint_props.parent_link);
    }

    std::vector<std::string> leaves;
    for (const auto &[link_name, link_props] : model.getLinks()) {
        (void)link_props;
        if (link_name == model.getRootLinkName()) {
            continue;
        }
        if (parent_links.count(link_name) == 0) {
            leaves.push_back(link_name);
        }
    }

    std::sort(leaves.begin(), leaves.end());
    leaves.erase(std::unique(leaves.begin(), leaves.end()), leaves.end());
    return leaves;
}

static std::vector<std::string> collectPathJointNames(
    const simulation::RobotModel &model, const std::string &root_link_name,
    const std::string &leaf_link_name) {
    std::map<std::string, const simulation::JointProperties *> child_to_joint;
    for (const auto &[joint_name, joint_props] : model.getJoints()) {
        (void)joint_name;
        child_to_joint[joint_props.child_link] = &joint_props;
    }

    const std::string stop_root =
        root_link_name.empty() ? model.getRootLinkName() : root_link_name;
    std::vector<std::string> joint_names_reversed;
    std::string current_link = leaf_link_name;
    std::unordered_set<std::string> visited_links;

    while (!current_link.empty() && current_link != stop_root) {
        if (!visited_links.insert(current_link).second) {
            break;
        }
        const auto it = child_to_joint.find(current_link);
        if (it == child_to_joint.end()) {
            break;
        }
        joint_names_reversed.push_back(it->second->name);
        current_link = it->second->parent_link;
    }

    std::reverse(joint_names_reversed.begin(), joint_names_reversed.end());
    return joint_names_reversed;
}

static std::vector<std::string> collectPathLinkNames(
    const simulation::RobotModel &model, const std::string &root_link_name,
    const std::string &leaf_link_name) {
    std::vector<std::string> link_names;

    if (!root_link_name.empty()) {
        const auto &links = model.getLinks();
        if (links.find(root_link_name) != links.end()) {
            link_names.push_back(root_link_name);
        }
    }

    const auto joint_names =
        collectPathJointNames(model, root_link_name, leaf_link_name);
    link_names.reserve(joint_names.size());
    for (const auto &joint_name : joint_names) {
        const auto &joints = model.getJoints();
        const auto it = joints.find(joint_name);
        if (it == joints.end()) {
            continue;
        }
        if (it->second.child_link.empty()) {
            continue;
        }
        if (std::find(link_names.begin(), link_names.end(), it->second.child_link) ==
            link_names.end()) {
            link_names.push_back(it->second.child_link);
        }
    }
    return link_names;
}

static std::vector<std::string> collectDescendantLinkNames(
    const simulation::RobotModel &model, const std::string &start_link_name) {
    std::unordered_map<std::string, std::vector<std::string>> children_by_parent;
    for (const auto &[joint_name, joint_props] : model.getJoints()) {
        (void)joint_name;
        children_by_parent[joint_props.parent_link].push_back(joint_props.child_link);
    }

    std::vector<std::string> descendants;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string &)> dfs = [&](const std::string &link_name) {
        if (!visited.insert(link_name).second) {
            return;
        }
        const auto it = children_by_parent.find(link_name);
        if (it == children_by_parent.end()) {
            return;
        }
        for (const auto &child : it->second) {
            if (std::find(descendants.begin(), descendants.end(), child) ==
                descendants.end()) {
                descendants.push_back(child);
            }
            dfs(child);
        }
    };

    dfs(start_link_name);
    return descendants;
}

static std::vector<std::string> collectBranchDescendantLinkNames(
    const simulation::RobotModel &model, const std::string &branch_root_link_name) {
    std::unordered_map<std::string, std::vector<std::string>> children_by_parent;
    for (const auto &[joint_name, joint_props] : model.getJoints()) {
        (void)joint_name;
        children_by_parent[joint_props.parent_link].push_back(joint_props.child_link);
    }

    const auto child_it = children_by_parent.find(branch_root_link_name);
    if (child_it == children_by_parent.end() || child_it->second.size() < 2) {
        return {};
    }

    std::vector<std::string> branch_links;
    std::unordered_set<std::string> unique_names;
    for (const auto &branch_child : child_it->second) {
        for (const auto &link_name : collectDescendantLinkNames(model, branch_child)) {
            if (unique_names.insert(link_name).second) {
                branch_links.push_back(link_name);
            }
        }
        if (unique_names.insert(branch_child).second) {
            branch_links.push_back(branch_child);
        }
    }
    return branch_links;
}

static std::vector<std::string> buildPathBasedLinkNames(
    const simulation::RobotModel &model, const std::string &root_link_name,
    const std::vector<std::string> &leaf_link_names) {
    std::vector<std::string> link_names;
    std::unordered_set<std::string> unique_names;
    for (const auto &leaf_link_name : leaf_link_names) {
        for (const auto &link_name :
             collectPathLinkNames(model, root_link_name, leaf_link_name)) {
            if (unique_names.insert(link_name).second) {
                link_names.push_back(link_name);
            }
        }
        for (const auto &link_name :
             collectBranchDescendantLinkNames(model, leaf_link_name)) {
            if (unique_names.insert(link_name).second) {
                link_names.push_back(link_name);
            }
        }
    }
    return link_names;
}

static std::string resolveFrameWithNamespace(
    const rclcpp::Node &node, const std::string &frame) {
    if (frame.empty()) {
        return {};
    }
    if (frame == "world" || frame[0] == '/') {
        return frame;
    }
    std::string ns = node.get_namespace();
    if (ns != "/" && !ns.empty()) {
        if (ns[0] == '/') {
            ns = ns.substr(1);
        }
        return ns + "/" + frame;
    }
    return frame;
}

} // namespace

namespace robot_sim::self_recognition {

SelfRecognitionVizNode::SelfRecognitionVizNode(const rclcpp::NodeOptions & options)
: Node("self_recognition_viz_node", options) {
    
    // パラメータ：計算に必要な最小限の設定
    declare_parameter("urdf_path", "");
    declare_parameter("resource_root_dir", "");
    declare_parameter("mesh_root_dir", "");
    declare_parameter("joint_topic", "joint_states");
    declare_parameter("robot.voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter("voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter("update_hz", 50.0); 
    declare_parameter("self_recognition.update_hz", 10.0);
    declare_parameter<std::vector<std::string>>("root_links", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("self_recognition.root_links", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("leaf_links", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("self_recognition.leaf_links", std::vector<std::string>{});
    declare_parameter<std::string>("robot.arm_leaf_link_names", "");
    declare_parameter<std::string>("self_recognition.arm_leaf_link_names", "");
    declare_parameter<std::vector<std::string>>("exclude_links", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("self_recognition.exclude_links", std::vector<std::string>{});
    declare_parameter<std::string>("robot.voxel_link_names", "");
    declare_parameter<std::string>("self_recognition.voxel_link_names", "");
    declare_parameter<std::string>("root_link", "");
    declare_parameter<std::string>("self_recognition.root_link", "");
    declare_parameter<std::string>("leaf_link", "");
    declare_parameter<std::string>("self_recognition.leaf_link", "");
    declare_parameter("self_recognition.inflation", 0.02);
    declare_parameter("robot.inflation", 0.0);
 
    // ボクセル展開パラメータの宣言
    declare_parameter("voxel_idx_shift.x_shift", ::robot_sim::common::Constants::DEFAULT_X_SHIFT);
    declare_parameter("voxel_idx_shift.y_shift", ::robot_sim::common::Constants::DEFAULT_Y_SHIFT);
    declare_parameter("voxel_idx_shift.z_shift", ::robot_sim::common::Constants::DEFAULT_Z_SHIFT);
    declare_parameter("voxel_idx_shift.offset", ::robot_sim::common::Constants::DEFAULT_OFFSET);
    declare_parameter("target_frame_id", ""); // 空ならロボットのベースを使用
    declare_parameter("self_recognition.target_frame_id", "");
    declare_parameter("self_recognition.marker_frame_id", "");
    declare_parameter("mask_topic", "/self_recognition/voxel_mask");
    declare_parameter("self_recognition.mask_topic", "");
    declare_parameter("self_output_topic", "");
    declare_parameter("self_recognition.self_output_topic", "");

    const std::string urdf_rel = get_parameter("urdf_path").as_string();
    const std::string resource_root_dir = get_parameter("resource_root_dir").as_string();
    const std::string mesh_root_dir = get_parameter("mesh_root_dir").as_string();
    const std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    if (urdf_path.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to resolve URDF path: %s", urdf_rel.c_str());
        throw std::runtime_error("Could not find URDF file");
    }
    RCLCPP_INFO(get_logger(), "Loading URDF from: %s", urdf_path.c_str());

    const std::string joint_topic = get_parameter("joint_topic").as_string();
    
    // Prefer self_recognition.resolution if specified, otherwise fallback to robot.voxel_size
    declare_parameter("self_recognition.resolution", 0.0);
    double res_param = get_parameter("self_recognition.resolution").as_double();
    if (res_param <= 0.0) {
        res_param = get_parameter("robot.voxel_size").as_double();
        if (res_param <= 0.0) {
            res_param = get_parameter("voxel_size").as_double();
        }
    }
    const double voxel_size_param = res_param;

    // モデルとチェインの構築
    try {
        model_ = std::make_shared<simulation::RobotModel>(
            simulation::loadRobotFromUrdf(urdf_path, resource_root_dir, mesh_root_dir));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load robot model: %s", e.what());
        throw;
    }

    auto root_links =
        getStringsWithFallback(*this, "self_recognition.root_links", "root_links");
    auto leaf_links =
        getStringsWithFallback(*this, "self_recognition.leaf_links", "leaf_links");
    std::string global_root_link = getStringWithFallback(*this, "self_recognition.root_link", "root_link");
    std::string explicit_leaf_link = getStringWithFallback(*this, "self_recognition.leaf_link", "leaf_link");
    if (global_root_link.empty()) {
        root_link_ = resolveFrameWithNamespace(*this, model_->getRootLinkName());
    } else {
        root_link_ = resolveFrameWithNamespace(*this, global_root_link);
    }
    std::string voxel_chain_root_link = global_root_link;
    std::string explicit_voxel_root_link = getStringWithFallback(*this, "self_recognition.root_link", "root_link");
    if (!explicit_voxel_root_link.empty()) {
        std::string parent_link = findParentLink(*model_, explicit_voxel_root_link);
        if (!parent_link.empty()) {
            voxel_chain_root_link = parent_link;
        }
    }

    // 木構造の全末端リンクを抽出
    auto expandTerminalLeafLinks = [&](const std::vector<std::string>& roots) {
        std::unordered_map<std::string, std::vector<std::string>> children_by_parent;
        for (const auto &[j_name, j_props] : model_->getJoints()) {
            children_by_parent[j_props.parent_link].push_back(j_props.child_link);
        }

        std::vector<std::string> expanded;
        std::unordered_set<std::string> visited;

        std::function<void(const std::string&)> collect_terminals = [&](const std::string& link_name) {
            if (visited.count(link_name)) return;
            visited.insert(link_name);

            auto it = children_by_parent.find(link_name);
            if (it == children_by_parent.end() || it->second.empty()) {
                expanded.push_back(link_name);
                return;
            }

            bool has_child = false;
            for (const auto& child : it->second) {
                has_child = true;
                collect_terminals(child);
            }
            if (!has_child) {
                expanded.push_back(link_name);
            }
        };

        for (const auto& root : roots) {
            if (!root.empty()) {
                collect_terminals(root);
            }
        }

        std::sort(expanded.begin(), expanded.end());
        expanded.erase(std::unique(expanded.begin(), expanded.end()), expanded.end());
        return expanded;
    };

    auto splitCommaSeparated = [](const std::string& input) {
        std::vector<std::string> out;
        std::stringstream ss(input);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token.erase(token.begin(), std::find_if(token.begin(), token.end(),
                                                    [](unsigned char ch) { return !std::isspace(ch); }));
            token.erase(std::find_if(token.rbegin(), token.rend(),
                                     [](unsigned char ch) { return !std::isspace(ch); }).base(),
                        token.end());
            if (!token.empty()) {
                out.push_back(token);
            }
        }
        return out;
    };

    const auto chain_leaf_links = collectTerminalLeafLinks(*model_);
    if (chain_leaf_links.empty()) {
        throw std::runtime_error("No terminal leaf links were found in the robot model");
    }

    std::vector<simulation::ArmConfig> arm_configs;
    arm_configs.reserve(chain_leaf_links.size());
    for (const auto &leaf_name : chain_leaf_links) {
        simulation::ArmConfig cfg;
        cfg.leaf_link = leaf_name;
        cfg.prefix = "";
        cfg.root_link = model_->getRootLinkName();
        arm_configs.push_back(cfg);
    }

    auto multi_chain = simulation::createMultiArmKinematicChain(*model_, arm_configs, Eigen::Vector3d::Zero());
    chain_ = std::shared_ptr<kinematics::KinematicChain>(std::move(multi_chain));

    // マネージャーの初期化
    recognition_manager_ = std::make_unique<robot_sim::recognition::SelfRecognitionManager>();
    
    // グリッド設定の初期化（パラメータから反映）
    auto* grid = recognition_manager_->getIndexGrid();
    grid->setVoxelSize(voxel_size_param);
    grid->setIndexingParams(
        get_parameter("voxel_idx_shift.x_shift").as_int(),
        get_parameter("voxel_idx_shift.y_shift").as_int(),
        get_parameter("voxel_idx_shift.z_shift").as_int(),
        get_parameter("voxel_idx_shift.offset").as_int());

    double hz = get_parameter("self_recognition.update_hz").as_double();
    if (hz <= 0.0) {
        hz = get_parameter("update_hz").as_double();
    }
    double inflation = get_parameter("self_recognition.inflation").as_double();
    if (inflation < 0.0) {
        inflation = get_parameter("robot.inflation").as_double();
    }
    if (inflation < 0.0) {
        inflation = 0.0;
    }

    std::vector<std::string> voxel_links;
    const std::string explicit_voxel_links =
        getStringWithFallback(*this, "self_recognition.voxel_link_names",
                              "robot.voxel_link_names");
    if (!explicit_voxel_links.empty()) {
        voxel_links = splitCommaSeparated(explicit_voxel_links);
    } else {
        const auto all_chain_links = [&]() {
            std::vector<std::string> links;
            links.reserve(static_cast<std::size_t>(chain_->getNumJoints()));
            for (int i = 0; i < chain_->getNumJoints(); ++i) {
                links.push_back(chain_->getLinkName(i));
            }
            return links;
        }();

        if (!global_root_link.empty() || !explicit_leaf_link.empty()) {
            if (!global_root_link.empty() && !explicit_leaf_link.empty()) {
                voxel_links = buildPathBasedLinkNames(
                    *model_, global_root_link, {explicit_leaf_link});
            } else if (!global_root_link.empty()) {
                voxel_links = buildPathBasedLinkNames(
                    *model_, global_root_link, chain_leaf_links);
            } else {
                voxel_links = buildPathBasedLinkNames(
                    *model_, model_->getRootLinkName(), {explicit_leaf_link});
            }
        } else {
            voxel_links = all_chain_links;
        }
    }

    if (voxel_links.empty()) {
        voxel_links.reserve(static_cast<std::size_t>(chain_->getNumJoints()));
        for (int i = 0; i < chain_->getNumJoints(); ++i) {
            voxel_links.push_back(chain_->getLinkName(i));
        }
    }

    // ボクセル化の実行（チェインは全体、voxel_links だけを範囲指定）
    auto voxel_data = ::simulation::RobotVoxelizer::build(*model_, voxel_links, *grid, {}, inflation);
    RCLCPP_INFO(
        get_logger(),
        "自己認識ボクセル化: 対象リンク=%zu, コリジョン有効リンク=%zu, 解像度=%.3fm, 膨張=%.3fm",
        voxel_links.size(), voxel_data.size(), voxel_size_param, inflation);
    recognition_manager_->initialize(chain_, model_, voxel_data, voxel_size_param);

    // 通信
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10,
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            chain_->updateJointValuesByName(msg->name, msg->position);
            current_joints_ = chain_->getJointValues();
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    mask_topic_ = getStringWithFallback(*this, "self_recognition.mask_topic", "mask_topic");
    if (mask_topic_.empty()) {
        mask_topic_ = getStringWithFallback(*this, "self_recognition.self_output_topic", "self_output_topic");
    }
    if (mask_topic_.empty()) {
        mask_topic_ = "/self_voxel";
    }
    mask_pub_ = create_publisher<voxel_msgs::msg::Voxel>(
        mask_topic_, rclcpp::QoS(1).reliable().transient_local());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(hz, 0.1))),
        [this]() { this->updateAndPublish(); });

    graph_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
            RCLCPP_INFO(
                get_logger(),
                "Topic graph: mask pubs=%zu joint pubs=%zu",
                count_publishers(mask_topic_),
                count_publishers(joint_sub_->get_topic_name()));
        });

    RCLCPP_INFO(get_logger(), "Voxel Mask Provider started (ID-only mode). Hz: %.1f, mask_topic: %s", hz, mask_topic_.c_str());
}

void SelfRecognitionVizNode::updateAndPublish() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (current_joints_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for joint states on topic: %s", joint_sub_->get_topic_name());
        return;
    }

    try {
        // 1. 関節角度の更新と順運動学 (FK)
        auto t_start = this->now();
        recognition_manager_->updateRobotState(current_joints_);
        auto t_fk = (this->now() - t_start).seconds() * 1000.0;

        std::string target_frame = getStringWithFallback(*this,
            "self_recognition.target_frame_id", "target_frame_id");
        target_frame = resolveFrameWithNamespace(*this, target_frame.empty() ? root_link_ : target_frame);

        Eigen::Isometry3d target_to_base = Eigen::Isometry3d::Identity();
        if (target_frame != root_link_) {
            try {
                auto tf_stamped = tf_buffer_->lookupTransform(
                    target_frame, root_link_, tf2::TimePointZero);
                target_to_base = tf2::transformToEigen(tf_stamped.transform);
                
                if (recognition_manager_->isTfChanged(target_to_base)) {
                    const auto& t = target_to_base.translation();
                    RCLCPP_INFO(get_logger(), "Robot moved! TF [%s -> %s]: pos=(%.2f, %.2f, %.2f)", 
                               root_link_.c_str(), target_frame.c_str(), t.x(), t.y(), t.z());
                }
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                                   "TF lookup failed! target_frame='%s', root_link='%s'. Error: %s", 
                                   target_frame.c_str(), root_link_.c_str(), ex.what());
            }
        }

        // 3. ボクセルマスクの生成 (座標変換 + 重複排除)
        auto mask_msg = std::make_shared<voxel_msgs::msg::Voxel>();
        mask_msg->header.stamp = get_clock()->now();
        mask_msg->header.frame_id = target_frame; 
        mask_msg->voxel_size = static_cast<float>(recognition_manager_->getVoxelSize());
        
        mask_msg->x_shift = get_parameter("voxel_idx_shift.x_shift").as_int();
        mask_msg->y_shift = get_parameter("voxel_idx_shift.y_shift").as_int();
        mask_msg->z_shift = get_parameter("voxel_idx_shift.z_shift").as_int();
        mask_msg->offset = get_parameter("voxel_idx_shift.offset").as_int();
        
        // 計算前にインデックスパラメータを同期
        recognition_manager_->getIndexGrid()->setIndexingParams(
            mask_msg->x_shift, mask_msg->y_shift, mask_msg->z_shift, mask_msg->offset
        );

        const auto vids = recognition_manager_->getSelfVoxelMask(target_to_base);
        mask_msg->data.assign(vids.begin(), vids.end());
        mask_pub_->publish(*mask_msg);

        // 毎フレームログ出力（学習検討用）
        RCLCPP_INFO(get_logger(), "Voxel Gen [%s]: Total %.2f ms | Vids: %zu (Pre: %zu)", 
                    target_frame.c_str(), recognition_manager_->getLastCalcTimeMs(), 
                    vids.size(), recognition_manager_->getLastPreUniqueCount());

    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error in updateAndPublish: %s", e.what());
    }
}

} // namespace robot_sim::self_recognition

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::self_recognition::SelfRecognitionVizNode)

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_sim::self_recognition::SelfRecognitionVizNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
