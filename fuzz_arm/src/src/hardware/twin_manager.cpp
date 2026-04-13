#include "hardware/twin_manager.hpp"
#include <iostream>
#include <sstream>

namespace robot_sim {
namespace digital_twin {

DigitalTwinManager::DigitalTwinManager(int rx_port, int tx_port)
    : rx_port_(rx_port), tx_port_(tx_port), receptor_engine_(0.02) { // 2cm voxel for receptors
}

DigitalTwinManager::~DigitalTwinManager() {
    stop();
}

void DigitalTwinManager::start() {
    if (running_) return;

    receiver_ = std::make_unique<UdpReceiver>(rx_port_);
    sender_ = std::make_unique<UdpSender>(real_robot_ip_, tx_port_);

    running_ = true;
    rx_thread_ = std::thread(&DigitalTwinManager::receiverLoop, this);
    std::cout << "[DigitalTwin] Started: RX Port=" << rx_port_ << ", TX " << real_robot_ip_ << ":" << tx_port_ << std::endl;
}

void DigitalTwinManager::stop() {
    running_ = false;
    // Note: UdpReceiver::receive is blocking. In a real production app, 
    // we'd use select() or non-blocking sockets to join cleanly.
    // For now, we'll let the OS cleanup or rely on the process exit.
    if (rx_thread_.joinable()) {
        // rx_thread_.join(); // This might hang due to blocking recv
        rx_thread_.detach();
    }
}

void DigitalTwinManager::receiverLoop() {
    while (running_) {
        std::string packet = receiver_->receive();
        if (!packet.empty()) {
            handlePacket(packet);
        }
    }
}

void DigitalTwinManager::handlePacket(const std::string& packet) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Packet formats:
    // R,j1,j2,j3,j4,j5,j6,g (Robot states)
    // N,id,x,y,z,r (Environment Node)
    
    std::stringstream ss(packet);
    std::string type;
    std::getline(ss, type, ',');

    if (type == "R") {
        // Parse robot joints (mock/stub for now)
    } else if (type == "N") {
        // Parse environment node
        std::string s_id, s_x, s_y, s_z, s_r;
        if (std::getline(ss, s_id, ',') && std::getline(ss, s_x, ',') && 
            std::getline(ss, s_y, ',') && std::getline(ss, s_z, ',') && 
            std::getline(ss, s_r, ',')) {
            
            Eigen::Vector3d pos(std::stod(s_x), std::stod(s_y), std::stod(s_z));
            // Transform to World Frame immediately
            Eigen::Vector3d world_pos = transformer_.transformEnvToWorld(pos);
            
            latest_env_nodes_.push_back(world_pos);
            has_new_env_data_ = true;
        }
    }
}

void DigitalTwinManager::update(std::vector<int>& node_collision_counts, 
                               const robot_sim::analysis::ISpatialIndex* spatial_index) {
    if (mode_ == SyncMode::OFF) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_new_env_data_) return;

    // Process new LIDAR-GNG nodes
    for (const auto& pos : latest_env_nodes_) {
        if (mode_ == SyncMode::RECEPTOR_SYNC) {
            // Mode A: Receptor Node Engine (World Frame)
            std::vector<int> hit_nodes = receptor_engine_.queryAffectedNodes(pos);
            for (int id : hit_nodes) {
                if (id >= 0 && id < (int)node_collision_counts.size()) {
                    node_collision_counts[id]++;
                }
            }
        } 
        else if (mode_ == SyncMode::VLUT_SYNC && spatial_index) {
            // Mode B: VLUT Sync (Inverse Transform to Robot Frame)
            // 1. Convert World-Absolute point to Robot-Relative point
            Eigen::Vector3d local_pos = transformer_.transformWorldToRobot(pos);
            
            // 2. Query the existing VLUT / Spatial Index using robot-local position
            std::vector<int> hit_nodes = spatial_index->getNodesInVoxel(local_pos);
            for (int id : hit_nodes) {
                if (id >= 0 && id < (int)node_collision_counts.size()) {
                    node_collision_counts[id]++;
                }
            }
        }
    }

    latest_env_nodes_.clear();
    has_new_env_data_ = false;
}

void DigitalTwinManager::sendCommandToReal(const std::vector<double>& joint_angles, double gripper) {
    if (mode_ != SyncMode::RECEPTOR_SYNC && mode_ != SyncMode::VLUT_SYNC) return;

    // Format: C,j1,j2,j3,j4,j5,j6,g
    // (Actual ToPoArm format might vary, this is for our DT loop)
    std::stringstream ss;
    ss << "C";
    for (double val : joint_angles) {
        ss << "," << (val * 180.0 / M_PI) * 10.0; // degree * 10
    }
    ss << "," << (gripper * 1800.0 / 0.02); // Mock gripper scale
    
    if (sender_) {
        sender_->send(ss.str());
    }
}

} // namespace digital_twin
} // namespace robot_sim
