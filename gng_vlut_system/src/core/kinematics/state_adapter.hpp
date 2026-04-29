#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace kinematics {

/**
 * @brief Spatial conversion engine between Planning (GNG), Communication (UDP), and Simulation (Physical).
 * 
 * Domains:
 * - Planning: 6D (Arm 1-6) for topological pathfinding.
 * - Communication: 7D (Arm 1-6 + Gripper) for UDP commands and control logic.
 * - Physical: 8D (Arm 1-6 + Gripper_L + Gripper_R) for ODE simulation.
 */
class JointStateAdapter {
public:
    /**
     * @brief Initialize mappings by identifying which joints belong to which domain.
     */
    void init(const std::vector<std::string>& planning_names,
              const std::vector<std::string>& physical_names) {
        planning_names_ = planning_names;
        physical_names_ = physical_names;

        // In this project:
        // Planning (6D) = First 6 joints of the chain (usually arm)
        // Communication (7D) = Planning (6D) + 1 Gripper axis (logical)
        // Physical (8D) = Planning (6D) + gripper_left + gripper_right (physical mimic)

        plan_to_phys_.clear();
        std::cout << "[JointStateAdapter] Building Name-Based Mapping Table:\n";
        for (size_t i = 0; i < planning_names_.size(); ++i) {
            auto it = std::find(physical_names_.begin(), physical_names_.end(), planning_names_[i]);
            if (it != physical_names_.end()) {
                int p_idx = std::distance(physical_names_.begin(), it);
                plan_to_phys_[static_cast<int>(i)] = p_idx;
                std::cout << "  Plan[" << i << "] (" << planning_names_[i] << ") -> Phys[" << p_idx << "]\n";
            }
        }

        // Gripper indices (Hardcoded for topoarm but identified by count for safety)
        if (physical_names.size() >= 8) {
            phys_gripper_l_ = 6;
            phys_gripper_r_ = 7;
            std::cout << "  Gripper Mimic: Phys[" << phys_gripper_l_ << "] -> Phys[" << phys_gripper_r_ << "]\n";
        }
    }

    // --- Domain: Planning (6D) ---

    /**
     * @brief Extract Planning state from Physical state (8D -> 6D).
     */
    Eigen::VectorXd toPlanning(const Eigen::VectorXd& phys_q) const {
        Eigen::VectorXd plan_q(planning_names_.size());
        for (const auto& [plan_idx, phys_idx] : plan_to_phys_) {
            if (phys_idx < phys_q.size()) plan_q(plan_idx) = phys_q(phys_idx);
        }
        return plan_q;
    }

    /**
     * @brief Expand Planning state to Physical state (6D -> 8D).
     * @param base_q Optional base state to preserve non-planning joints (grippers).
     */
    Eigen::VectorXd fromPlanning(const Eigen::VectorXd& plan_q, 
                                 const Eigen::VectorXd& base_q = Eigen::VectorXd()) const {
        Eigen::VectorXd phys_q = Eigen::VectorXd::Zero(physical_names_.size());
        
        // If base_q is provided and valid, use it as starting point
        if (base_q.size() == physical_names_.size()) {
            phys_q = base_q;
        }

        for (const auto& [plan_idx, phys_idx] : plan_to_phys_) {
            if (plan_idx < plan_q.size() && phys_idx < phys_q.size()) {
                phys_q(phys_idx) = plan_q(plan_idx);
            }
        }
        return phys_q;
    }

    // --- Domain: Communication (7D) ---

    /**
     * @brief Extract Communication state from Physical state (8D -> 7D).
     */
    Eigen::VectorXd toCommunication(const Eigen::VectorXd& phys_q) const {
        Eigen::VectorXd comm_q(planning_names_.size() + 1); // 6 + 1 = 7
        for (const auto& [plan_idx, phys_idx] : plan_to_phys_) {
            if (phys_idx < phys_q.size()) comm_q(plan_idx) = phys_q(phys_idx);
        }
        // Copy one gripper value
        if (phys_gripper_l_ != -1 && phys_gripper_l_ < phys_q.size()) {
            comm_q(planning_names_.size()) = phys_q(phys_gripper_l_);
        }
        return comm_q;
    }

    /**
     * @brief Expand Communication state to Physical state (7D -> 8D).
     * Handles gripper mimic logic (left -> right).
     * @param base_q Optional base state to preserve non-communication joints.
     */
    Eigen::VectorXd fromCommunication(const Eigen::VectorXd& comm_q,
                                      const Eigen::VectorXd& base_q = Eigen::VectorXd()) const {
        Eigen::VectorXd phys_q = Eigen::VectorXd::Zero(physical_names_.size());
        
        // If base_q is provided and valid, use it as starting point
        if (base_q.size() == physical_names_.size()) {
            phys_q = base_q;
        }

        for (const auto& [plan_idx, phys_idx] : plan_to_phys_) {
            if (plan_idx < comm_q.size() && phys_idx < phys_q.size()) {
                phys_q(phys_idx) = comm_q(plan_idx);
            }
        }
        // Deploy gripper command to physical fingers
        int gripper_idx = static_cast<int>(planning_names_.size());
        if (gripper_idx < comm_q.size()) {
            double g_val = comm_q(gripper_idx);
            if (phys_gripper_l_ != -1) phys_q(phys_gripper_l_) = g_val;
            if (phys_gripper_r_ != -1) phys_q(phys_gripper_r_) = g_val;
        }
        return phys_q;
    }

    // --- Legacy Bridge for Kinematics (Internal Use) ---
    
    std::vector<double> toPhysicalVec(const Eigen::VectorXd& plan_q) const {
        Eigen::VectorXd phys = fromPlanning(plan_q);
        return std::vector<double>(phys.data(), phys.data() + phys.size());
    }

    Eigen::VectorXd toLogical(const Eigen::VectorXd& phys_q) const {
        return toPlanning(phys_q);
    }

    /**
     * @brief Ensure the vector is in the planning domain (6D).
     * If the vector is larger (e.g. 8D), it is clipped. If smaller, it warns.
     */
    Eigen::VectorXd ensurePlanningDim(const Eigen::VectorXd& q) const {
        if (q.size() == (int)planning_names_.size()) return q;
        if (q.size() > (int)planning_names_.size()) {
            return q.head(planning_names_.size());
        }
        std::cerr << "[JointStateAdapter] Warning: Input vector size (" << q.size() 
                  << ") is less than planning DOF (" << planning_names_.size() << ")" << std::endl;
        Eigen::VectorXd res = Eigen::VectorXd::Zero(planning_names_.size());
        res.head(q.size()) = q;
        return res;
    }

    /**
     * @brief Ensure the vector is in the physical domain (8D).
     */
    Eigen::VectorXd ensurePhysicalDim(const Eigen::VectorXd& q) const {
        if (q.size() == (int)physical_names_.size()) return q;
        if (q.size() > (int)physical_names_.size()) {
            return q.head(physical_names_.size());
        }
        // If it's planning dim, we should use toPhysicalVec instead of raw resizing
        if (q.size() == (int)planning_names_.size()) {
            std::vector<double> pv = toPhysicalVec(q);
            return Eigen::Map<Eigen::VectorXd>(pv.data(), pv.size());
        }
        std::cerr << "[JointStateAdapter] Warning: Input vector size (" << q.size() 
                  << ") is less than physical DOF (" << physical_names_.size() << ")" << std::endl;
        Eigen::VectorXd res = Eigen::VectorXd::Zero(physical_names_.size());
        res.head(q.size()) = q;
        return res;
    }

private:
    std::vector<std::string> planning_names_;
    std::vector<std::string> physical_names_;
    std::map<int, int> plan_to_phys_;
    int phys_gripper_l_ = -1;
    int phys_gripper_r_ = -1;
};

} // namespace kinematics
