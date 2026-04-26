#include "rclcpp/rclcpp.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "description/kinematic_adapter.hpp"
#include "description/robot_model.hpp"
#include "description/urdf_loader.hpp"
#include "core_safety/management/gng_status_providers.hpp"
#include "common/resource_utils.hpp"
#include <iostream>
#include <memory>
#include <string>

#include "spatial/boundary_node_classifier.hpp"
#include "spatial/centroid_shift_surface_classifier.hpp"
#include "spatial/occupancy_grid_surface_classifier.hpp"
#include "spatial/surface_node_classifier.hpp"

class OfflineGngStatusUpdaterNode : public rclcpp::Node {
public:
    OfflineGngStatusUpdaterNode(const rclcpp::NodeOptions& options) : Node("offline_gng_status_updater", options) {
        RCLCPP_INFO(this->get_logger(), "--- Offline GNG Status Updater ---");

        this->declare_parameter("robot_urdf_path", "custom_robot.urdf");
        this->declare_parameter("leaf_link_name", "link_7");
        this->declare_parameter("updater_target_suffix", "_pruned");
        this->declare_parameter("experiment_id", "default_run");
        this->declare_parameter("data_directory", "gng_results");
    }

    void run() {
        // 2. Determine Input File
        std::string data_dir = this->get_parameter("data_directory").as_string();
        std::string exp_id = this->get_parameter("experiment_id").as_string();
        std::string suffix = this->get_parameter("updater_target_suffix").as_string();
        std::string input_file = data_dir + "/" + exp_id + "/" + exp_id + suffix + ".bin";

        RCLCPP_INFO(this->get_logger(), "Target GNG Map: %s", input_file.c_str());

        // 3. Load Robot Model (KinematicChain)
        std::string urdf_path_base = this->get_parameter("robot_urdf_path").as_string();
        std::string urdf_file = robot_sim::common::resolvePath(urdf_path_base);

        kinematics::KinematicChain arm;
        simulation::RobotModel *model = nullptr;

        try {
            auto model_obj = simulation::loadRobotFromUrdf(urdf_file);
            model = new simulation::RobotModel(model_obj);
            std::string leaf_link = this->get_parameter("leaf_link_name").as_string();
            arm = simulation::createKinematicChainFromModel(*model, leaf_link);
            arm.setBase(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
            RCLCPP_INFO(this->get_logger(), "Successfully loaded robot from: %s, Leaf Link: %s", urdf_file.c_str(), leaf_link.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading robot: %s", e.what());
            return;
        }

        // 4. Initialize GNG
        GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f> gng(arm.getTotalDOF(), 3, &arm);

        if (!gng.load(input_file)) {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to load GNG file: %s", input_file.c_str());
            if (model) delete model;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded GNG map with %zu active nodes", gng.getActiveIndices().size());

        // 5. Register Status Providers
        RCLCPP_INFO(this->get_logger(), "Registering Status Providers...");
        auto manip_provider = std::make_shared<GNG::ManipulabilityProvider<Eigen::VectorXf, Eigen::Vector3f>>(&arm);
        gng.registerStatusProvider(manip_provider);
        auto ee_provider = std::make_shared<GNG::EEDirectionProvider<Eigen::VectorXf, Eigen::Vector3f>>(&arm);
        gng.registerStatusProvider(ee_provider);

        // 6. Execute Batch Update (Calculate Features)
        RCLCPP_INFO(this->get_logger(), "Calculating node features...");
        gng.triggerBatchUpdates();

        // 6.5. Reset all surface flags
        RCLCPP_INFO(this->get_logger(), "Resetting all surface flags to false...");
        for (size_t i = 0; i < gng.getMaxNodeNum(); ++i) {
            auto &node = gng.nodeAt(i);
            node.status.is_surface = false;
            node.status.is_active_surface = false;
            node.status.is_boundary = false;
        }
        RCLCPP_INFO(this->get_logger(), "Reset %zu nodes.", gng.getMaxNodeNum());

        // 7. Run Classifiers (Surface & Boundary)
        RCLCPP_INFO(this->get_logger(), "Running Node Classifiers...");
        GNG::Analysis::CentroidShiftSurfaceClassifier<decltype(gng)> shift_classifier;
        shift_classifier.shift_threshold_factor = 0.3f;
        shift_classifier.classify(gng);
        RCLCPP_INFO(this->get_logger(), "[BoundaryClassifier] SKIPPED (disabled for now)");

        if (gng.save(input_file)) {
            RCLCPP_INFO(this->get_logger(), "Successfully updated and saved GNG map to: %s", input_file.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to save updated GNG map.");
            if (model) delete model;
            return;
        }

        if (model) delete model;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<OfflineGngStatusUpdaterNode>(options);
    node->run();
    rclcpp::shutdown();
    return 0;
}
