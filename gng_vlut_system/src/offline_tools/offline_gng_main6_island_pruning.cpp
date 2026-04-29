#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "common/resource_utils.hpp"

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

class OfflineGngMain6IslandPruningNode : public rclcpp::Node {
public:
    OfflineGngMain6IslandPruningNode(const rclcpp::NodeOptions& options) : Node("offline_gng_main6_island_pruning", options) {
        RCLCPP_INFO(this->get_logger(), "--- Offline GNG Phase 5: Island Pruning (Largest Component) ---");

        this->declare_parameter("robot_urdf_path", "custom_robot.urdf");
        this->declare_parameter("leaf_link_name", "link_7");
        this->declare_parameter("island_pruning_input_suffix", "_phase4");
        this->declare_parameter("island_pruning_output_suffix", "_pruned");
        this->declare_parameter("island_pruning_permanent_removal", true);
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
            RCLCPP_ERROR(this->get_logger(), "Error initializing robot: %s", e.what());
            return;
        }

        GNG2 gng(arm.getTotalDOF(), 3, &arm);
        // gng.loadParameters("gng_offline.cfg"); // Handled by ROS params if needed

        std::string data_dir = robot_sim::common::resolveDataPath(
            this->get_parameter("data_directory").as_string());
        std::string exp_id = this->get_parameter("experiment_id").as_string();
        std::string input_suffix = this->get_parameter("island_pruning_input_suffix").as_string();
        std::string input_file = data_dir + "/" + exp_id + "/" + exp_id + input_suffix + ".bin";

        if (!gng.load(input_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load %s", input_file.c_str());
            delete model;
            return;
        }

        // Refresh coordinates to match current Kinematic Model (EEF Tip)
        gng.refresh_coord_weights();
        RCLCPP_INFO(this->get_logger(), "[GNG] Refreshed node coordinates to EEF Tip.");

        RCLCPP_INFO(this->get_logger(), "Extracting largest connected component...");
        gng.pruneToLargestComponent();

        // Optionally remove deactivated elements to keep file small
        bool permanent_removal = this->get_parameter("island_pruning_permanent_removal").as_bool();
        if (permanent_removal) {
            gng.removeInactiveElements();
        }

        std::string output_suffix = this->get_parameter("island_pruning_output_suffix").as_string();
        std::string output_file = data_dir + "/" + exp_id + "/" + exp_id + output_suffix + ".bin";

        gng.save(output_file);
        RCLCPP_INFO(this->get_logger(), "Saved pruned map to %s", output_file.c_str());

        delete model;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<OfflineGngMain6IslandPruningNode>(options);
    node->run();
    rclcpp::shutdown();
    return 0;
}
