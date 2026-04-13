#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <atomic>
#include <thread>

#include "hardware/coordinate_system.hpp"
#include "hardware/receptor_node_engine.hpp"
#include "hardware/udp_comm.hpp"
#include "spatial/ispatial_index.hpp"

namespace robot_sim {
namespace digital_twin {

enum class SyncMode { OFF, RECEPTOR_SYNC, VLUT_SYNC };

/**
 * @brief Main manager for the Digital Twin system.
 * 
 * Orchestrates UDP communication, coordinate transforms, and the receptor engine.
 */
class DigitalTwinManager {
public:
    DigitalTwinManager(int rx_port, int tx_port);
    ~DigitalTwinManager();

    // --- Control ---
    void start();
    void stop();
    void setMode(SyncMode mode) { mode_ = mode; }
    SyncMode getMode() const { return mode_; }

    // --- Infrastructure ---
    CoordinateTransformer& getTransformer() { return transformer_; }
    ReceptorNodeEngine& getReceptorEngine() { return receptor_engine_; }

    // --- Simulation Sync ---
    /**
     * @brief Periodically called by the simulation update loop.
     * Processes received UDP packets and updates the receptor statuses.
     */
    void update(std::vector<int>& node_collision_counts, 
                const robot_sim::analysis::ISpatialIndex* spatial_index = nullptr);

    /**
     * @brief Send current simulation robot state back to the real robot.
     */
    void sendCommandToReal(const std::vector<double>& joint_angles, double gripper);

private:
    void receiverLoop();
    void handlePacket(const std::string& packet);

    // Networking
    int rx_port_;
    int tx_port_;
    std::string real_robot_ip_ = "192.168.4.40"; // Default ToPoArm IP
    
    std::unique_ptr<UdpReceiver> receiver_;
    std::unique_ptr<UdpSender> sender_;
    
    std::atomic<bool> running_{false};
    std::thread rx_thread_;
    std::mutex data_mutex_;

    // Core Components
    SyncMode mode_ = SyncMode::OFF;
    CoordinateTransformer transformer_;
    ReceptorNodeEngine receptor_engine_;

    // Received State
    std::vector<double> latest_real_joints_;
    std::vector<Eigen::Vector3d> latest_env_nodes_;
    bool has_new_env_data_ = false;
};

} // namespace digital_twin
} // namespace robot_sim
