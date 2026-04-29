// 非activeノードを削除

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "collision/geometric_self_collision_checker.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "safety_engine/runtime/gng_status_providers.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "common/resource_utils.hpp"

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

class OfflineGngMain5CleanNode : public rclcpp::Node {
public:
    OfflineGngMain5CleanNode(const rclcpp::NodeOptions& options) : Node("offline_gng_main5_clean", options) {
        RCLCPP_INFO(this->get_logger(), "--- Offline GNG Phase 4: Cleanup Inactive Elements ---");

        this->declare_parameter("robot_urdf_path", "custom_robot.urdf");
        this->declare_parameter("leaf_link_name", "link_7");
        this->declare_parameter("phase4_input_suffix", "_phase3");
        this->declare_parameter("phase4_output_suffix", "_phase4");
        this->declare_parameter("experiment_id", "default_run");
        this->declare_parameter("data_directory", "gng_results");
    }

    void run() {
        // Robot Load (Needed for Constructor)
        kinematics::KinematicChain arm;
        simulation::RobotModel *model = nullptr;
        try {
            std::string urdf_path = this->get_parameter("robot_urdf_path").as_string();
            std::string full_urdf = robot_sim::common::resolvePath(urdf_path);
            std::string leaf_link = this->get_parameter("leaf_link_name").as_string();
            auto model_obj = simulation::loadRobotFromUrdf(full_urdf);
            model = new simulation::RobotModel(model_obj);
            arm = simulation::createKinematicChainFromModel(*model, leaf_link);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s", e.what());
            return;
        }

        GNG2 gng(arm.getTotalDOF(), 3, &arm);
        // gng.loadParameters("gng_offline.cfg"); // This should be handled by ROS params if needed

        std::string data_dir = robot_sim::common::resolveDataPath(
            this->get_parameter("data_directory").as_string());
        std::string exp_id = this->get_parameter("experiment_id").as_string();
        std::string input_suffix = this->get_parameter("phase4_input_suffix").as_string();
        std::string input_file = data_dir + "/" + exp_id + "/" + exp_id + input_suffix + ".bin";

        if (!gng.load(input_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", input_file.c_str());
            delete model;
            return;
        }

        // Refresh coordinates to match current Kinematic Model (EEF Tip)
        gng.refresh_coord_weights();
        RCLCPP_INFO(this->get_logger(), "[GNG] Refreshed node coordinates to EEF Tip.");

        RCLCPP_INFO(this->get_logger(), "Pruning inactive nodes and edges...");
        gng.removeInactiveElements();

        RCLCPP_INFO(this->get_logger(), "Calculating node features (Manipulability, etc.)...");
        // Register Providers
        auto manip_provider = std::make_shared<
            GNG::ManipulabilityProvider<Eigen::VectorXf, Eigen::Vector3f>>(&arm);
        gng.registerStatusProvider(manip_provider);
        // registerEEProvider if needed...

        // Compute
        gng.triggerBatchUpdates();

        std::string output_suffix = this->get_parameter("phase4_output_suffix").as_string();
        std::string output_file = data_dir + "/" + exp_id + "/" + exp_id + output_suffix + ".bin";
        gng.save(output_file);
        RCLCPP_INFO(this->get_logger(), "Saved clean map to %s", output_file.c_str());

        // Also export result dat
        std::string dat_file =
            output_file.substr(0, output_file.find_last_of('.')) + "_vis.dat";

        FILE *fp = fopen(dat_file.c_str(), "w");
        if (fp) {
            for (int i = 0; i < (int)gng.getMaxNodeNum(); ++i) {
                const auto &node = gng.nodeAt(i);
                if (node.id == -1)
                    continue;

                fprintf(fp, "%d %f %f %f ", node.id, node.weight_coord.x(),
                        node.weight_coord.y(), node.weight_coord.z());
                fprintf(fp, "%d ", (int)node.weight_angle.size());
                for (int j = 0; j < node.weight_angle.size(); ++j)
                    fprintf(fp, "%f ", node.weight_angle(j));

                auto neighbors = gng.getNeighborsAngle(node.id);
                fprintf(fp, "%d ", (int)neighbors.size());
                for (int n : neighbors)
                    fprintf(fp, "%d ", n);
                fprintf(fp, "\n");
            }
            fclose(fp);
        }
        RCLCPP_INFO(this->get_logger(), "Exported %s", dat_file.c_str());

        delete model;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<OfflineGngMain5CleanNode>(options);
    node->run();
    rclcpp::shutdown();
    return 0;
}
