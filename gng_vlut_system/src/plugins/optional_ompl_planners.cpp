// MoveItの公開OMPLインターフェースへの追加アロケータ。探索・干渉計算は既存ライブラリ側。
#if __has_include(<moveit/ompl_interface/ompl_interface.hpp>)
#include <moveit/ompl_interface/ompl_interface.hpp>
#else
#include <moveit/ompl_interface/ompl_interface.h>
#endif
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <pluginlib/class_list_macros.hpp>
#include <memory>
#include <string>
#include <vector>

namespace gng_vlut_system {

class optional_ompl_planners final : public planning_interface::PlannerManager {
public:
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override {
    interface_ = std::make_unique<ompl_interface::OMPLInterface>(model, node, parameter_namespace);
    auto& manager = interface_->getPlanningContextManager();
    manager.registerPlannerAllocator("geometric::BITstar", allocate<ompl::geometric::BITstar>);
    manager.registerPlannerAllocator("geometric::InformedRRTstar", allocate<ompl::geometric::InformedRRTstar>);
    manager.setMinimumWaypointCount(2);
    setPlannerConfigurations(interface_->getPlannerConfigurations());
    return true;
  }

  // 外部仮想関数名はMoveItのAPIに準拠。登録名と衝突検査の共通契約。
  std::string getDescription() const override { return "OMPL"; }

  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& request) const override {
    return request.trajectory_constraints.constraints.empty();
  }

  void getPlanningAlgorithms(std::vector<std::string>& names) const override {
    names.clear();
    for (const auto& entry : interface_->getPlannerConfigurations()) names.push_back(entry.first);
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& configs) override {
    interface_->setPlannerConfigurations(configs);
    planning_interface::PlannerManager::setPlannerConfigurations(interface_->getPlannerConfigurations());
  }

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& scene,
      const planning_interface::MotionPlanRequest& request,
      moveit_msgs::msg::MoveItErrorCodes& error) const override {
    // 未登録IDの既定計画器への自動切替禁止。
    const auto& configs = interface_->getPlannerConfigurations();
    if (request.planner_id.empty() ||
        configs.find(request.group_name + "[" + request.planner_id + "]") == configs.end()) {
      error.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return {};
    }
    auto context = interface_->getPlanningContext(scene, request, error);
    if (context) context->simplifySolutions(false);
    return context;
  }

private:
  template<class planner_type>
  static ompl::base::PlannerPtr allocate(const ompl::base::SpaceInformationPtr& space,
      const std::string& name, const ompl_interface::ModelBasedPlanningContextSpecification& spec) {
    auto planner = std::make_shared<planner_type>(space);
    if (!name.empty()) planner->setName(name);
    planner->params().setParams(spec.config_, true);
    return planner;
  }

  std::unique_ptr<ompl_interface::OMPLInterface> interface_;
};

} // namespace gng_vlut_system

PLUGINLIB_EXPORT_CLASS(gng_vlut_system::optional_ompl_planners, planning_interface::PlannerManager)
