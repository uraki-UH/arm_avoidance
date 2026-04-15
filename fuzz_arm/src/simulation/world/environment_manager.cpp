#include "simulation/world/environment_manager.hpp"
#include "common/resource_utils.hpp"
#include "simulation/robot/kinematic_adapter.hpp"
#include "simulation/sensing/robot_point_cloud_sampler.hpp"
#include "simulation/robot/urdf_loader.hpp"
#include <drawstuff/drawstuff.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// Macro definitions for double precision drawstuff
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

namespace simulation {

EnvironmentManager::EnvironmentManager(dWorldID world, dSpaceID space)
    : world_(world), space_(space) {}

// ... existing code ...
// I will not replace the whole file, just the new added parts and the top of
// the file. Wait, I can only replace contiguous blocks. I'll update the top
// includes and the draw function separately if needed, or just replace the
// whole file content I know. Actually I'll use multi_replace or carefully
// targeted replace. Let's replace the top to add macros and GL includes.

EnvironmentManager::~EnvironmentManager() { clear(); }

void EnvironmentManager::clear() {
  for (dGeomID g : ode_geoms_) {
    dGeomDestroy(g);
  }
  ode_geoms_.clear();
  robot_obstacles_.clear();
  configs_.clear();
  pc_dirty_ = true;
}

bool EnvironmentManager::load(const std::string &filename) {
  clear();
  std::string resolved_path = robot_sim::common::resolvePath(filename);
  std::ifstream ifs(resolved_path);
  if (!ifs.is_open())
    return false;

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::stringstream ss(line);
    std::string type_str, name;
    ss >> type_str >> name;

    ObstacleConfig config;
    config.name = name;
    if (type_str == "STATIC_BOX") {
      config.type = ObstacleType::STATIC_BOX;
      ss >> config.pos.x() >> config.pos.y() >> config.pos.z();
      ss >> config.size.x() >> config.size.y() >> config.size.z();
      addStaticBox(config);
    } else if (type_str == "OBSTACLE_ROBOT") {
      config.type = ObstacleType::OBSTACLE_ROBOT;
      ss >> config.urdf_path;
      ss >> config.pos.x() >> config.pos.y() >> config.pos.z();
      ss >> config.leaf_link_name;
      addObstacleRobot(config);
    }
    // Others can be added here
    configs_.push_back(config);
  }
  return true;
}

void EnvironmentManager::addStaticBox(const ObstacleConfig &config) {
  dGeomID box =
      dCreateBox(space_, config.size.x(), config.size.y(), config.size.z());
  dGeomSetPosition(box, config.pos.x(), config.pos.y(), config.pos.z());
  ode_geoms_.push_back(box);

  // For static objects, we can pre-sample points once
  // But for simplicity in this manager, we'll just handle them in update if
  // needed Actually static points don't move, so we can store them.
}

void EnvironmentManager::addObstacleRobot(const ObstacleConfig &config) {
  if (config.leaf_link_name.empty()) {
    throw std::runtime_error("Obstacle robot '" + config.name +
                             "' is missing leaf_link_name in env_config.txt");
  }
  auto model_obj = loadRobotFromUrdf("urdf/" + config.urdf_path + ".urdf");
  auto obs = std::make_unique<RobotObstacle>();
  obs->name = config.name;
  obs->base_pos = config.pos;
  obs->model = std::make_unique<RobotModel>(model_obj);

  // We need a way to set the base position without ODE bodies if we only want
  // point clouds or we can build them in ODE if we want to see them. Let's
  // assume for now we don't build them in ODE to avoid collision overhead,
  // unless the user wants to see them.

  obs->chain = std::make_unique<kinematics::KinematicChain>(
      createKinematicChainFromModel(*obs->model, config.leaf_link_name,
                                    config.pos));

  for (const auto &jp : obs->model->getJoints()) {
    if (jp.second.type == kinematics::JointType::Fixed) {
      obs->fixed_links[jp.second.child_link] = {jp.second.parent_link,
                                                jp.second.origin};
    }
  }

  obs->sampler =
      std::make_unique<RobotPointCloudSampler>(*obs->model, *obs->chain);
  obs->sampler->setSamplingDensity(1000.0);

  // Set default behavior
  obs->behavior = std::make_unique<WavingRobotBehavior>();

  robot_obstacles_.push_back(std::move(obs));
}

void EnvironmentManager::update(double t) {
  // 1. Update robots
  for (auto &obs : robot_obstacles_) {
    if (obs->behavior) {
      obs->behavior->update(t, *obs->chain);
    }
    obs->sampler->update(obs->fixed_links);
  }
  pc_dirty_ = true;
}

const std::vector<Eigen::Vector3d> &
EnvironmentManager::getCombinedPointCloud() const {
  if (pc_dirty_) {
    combined_pc_.clear();
    for (const auto &obs : robot_obstacles_) {
      const auto &pts = obs->sampler->getPoints();
      combined_pc_.insert(combined_pc_.end(), pts.begin(), pts.end());
    }
    // Add static points if any (TODO)
    pc_dirty_ = false;
  }
  return combined_pc_;
}

collision::AABB EnvironmentManager::getPointCloudBounds() const {
  collision::AABB bounds;
  for (const auto &p : getCombinedPointCloud()) {
    bounds.expand(p);
  }
  return bounds;
}

std::vector<RobotObstacleState>
EnvironmentManager::getRobotObstacleStates() const {
  std::vector<RobotObstacleState> states;
  for (const auto &obs : robot_obstacles_) {
    RobotObstacleState s;
    s.name = obs->name;
    s.joint_positions = obs->chain->getJointValues();
    s.base_pos = obs->base_pos;
    states.push_back(s);
  }
  return states;
}

void EnvironmentManager::setRobotObstacleStates(
    const std::vector<RobotObstacleState>& states) {
  for (const auto& s : states) {
    for (auto& obs : robot_obstacles_) {
      if (obs->name != s.name) continue;
      if (!obs->chain) break;
      // Restore joint positions via updateKinematics
      Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
          s.joint_positions.data(), (int)s.joint_positions.size());
      int dof = obs->chain->getTotalDOF();
      obs->chain->updateKinematics(q.head(std::min((int)q.size(), dof)));
      obs->sampler->update(obs->fixed_links);
      pc_dirty_ = true;
      break;
    }
  }
}

// Helper to draw geometries manually since they are not in ODE space for
// drawing
// Helper to draw geometries manually since they are not in ODE space for
// drawing
static void drawVisualGeometry(const Geometry &geom,
                               const Eigen::Isometry3d &pose) {
  dReal pos[3] = {(dReal)pose.translation().x(), (dReal)pose.translation().y(),
                  (dReal)pose.translation().z()};
  Eigen::Matrix3d R_eigen = pose.rotation();
  dReal R[12] = {
      (dReal)R_eigen(0, 0), (dReal)R_eigen(0, 1), (dReal)R_eigen(0, 2), 0,
      (dReal)R_eigen(1, 0), (dReal)R_eigen(1, 1), (dReal)R_eigen(1, 2), 0,
      (dReal)R_eigen(2, 0), (dReal)R_eigen(2, 1), (dReal)R_eigen(2, 2), 0};

  switch (geom.type) {
  case GeometryType::BOX: {
    dReal sides[3] = {(dReal)geom.size.x(), (dReal)geom.size.y(),
                      (dReal)geom.size.z()};
    dsDrawBox(pos, R, sides);
    break;
  }
  case GeometryType::SPHERE: {
    dsDrawSphere(pos, R, (dReal)geom.size.x());
    break;
  }
  case GeometryType::CYLINDER: {
    dsDrawCylinder(pos, R, (dReal)geom.size.y(), (dReal)geom.size.x());
    break;
  }
  default:
    break;
  }
}

void EnvironmentManager::draw(bool show_structure, const float *color) {
  if (!show_structure)
    return;

  // Use OpenGL directly to force wireframe rendering
  // Note: drawstuff resets some GL state, so we set it here.
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glLineWidth(2.5f); // Make sure lines are visible

  for (const auto &obs : robot_obstacles_) {
    // Set Color
    if (color) {
      dsSetColor(color[0], color[1], color[2]);
    } else {
      // Default Red for obstacle (Danger)
      dsSetColor(1.0f, 0.2f, 0.2f);
    }

    if (!obs->model || !obs->chain)
      continue;

    // Ensure FK is up to date
    obs->chain->forwardKinematics();

    const auto &link_positions = obs->chain->getLinkPositions();
    const auto &link_orientations = obs->chain->getLinkOrientations();

    // 1. Draw Base Link (Index 0 in positions)
    if (!link_positions.empty()) {
      std::string base_name = obs->model->getRootLinkName();
      const auto *base_prop = obs->model->getLink(base_name);
      if (base_prop) {
        Eigen::Isometry3d base_pose = Eigen::Isometry3d::Identity();
        base_pose.translation() = link_positions[0];
        base_pose.linear() = link_orientations[0].toRotationMatrix();

        for (const auto &visual : base_prop->visuals) {
          Eigen::Isometry3d visual_pose = base_pose * visual.origin;
          drawVisualGeometry(visual.geometry, visual_pose);
        }
      }
    }

    // 2. Draw Chain Links (Indices 1 to N in positions, corresponding to chain
    // segments)
    int num_joints = obs->chain->getNumJoints();
    for (int i = 0; i < num_joints; ++i) {
      if ((size_t)(i + 1) >= link_positions.size())
        break; // Safety

      std::string link_name = obs->chain->getLinkName(i);
      const auto *link_prop = obs->model->getLink(link_name);

      if (!link_prop)
        continue;

      Eigen::Isometry3d link_pose = Eigen::Isometry3d::Identity();
      link_pose.translation() = link_positions[i + 1];
      link_pose.linear() = link_orientations[i + 1].toRotationMatrix();

      for (const auto &visual : link_prop->visuals) {
        Eigen::Isometry3d visual_pose = link_pose * visual.origin;
        drawVisualGeometry(visual.geometry, visual_pose);
      }
    }
  }

  // Restore to Fill mode
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

} // namespace simulation
