#include "simulation/viz/visualization_manager.hpp"
#include "gng/GrowingNeuralGas_offline2.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "simulation/world/dynamic_obstacle_manager.hpp" // Added include
#include "simulation/world/sim_obstacle_controller.hpp"

#include "simulation/core/simulation_state.hpp" // Added include
#include "simulation/safety/influence_management.hpp"
#include "simulation/safety/safety_management.hpp"
#include "simulation/sensing/voxel_grid.hpp" // Added include
#include "status/manipulability.hpp"

#include <drawstuff/drawstuff.h>
// #include <iostream>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

namespace robot_sim {
namespace simulation {

void VisualizationManager::drawWireframeBox(const Eigen::Vector3f &center,
                                            const Eigen::Vector3f &size) {
  float hx = size.x() * 0.5f;
  float hy = size.y() * 0.5f;
  float hz = size.z() * 0.5f;

  float p0[3] = {center.x() - hx, center.y() - hy, center.z() - hz};
  float p1[3] = {center.x() + hx, center.y() - hy, center.z() - hz};
  float p2[3] = {center.x() + hx, center.y() + hy, center.z() - hz};
  float p3[3] = {center.x() - hx, center.y() + hy, center.z() - hz};
  float p4[3] = {center.x() - hx, center.y() - hy, center.z() + hz};
  float p5[3] = {center.x() + hx, center.y() - hy, center.z() + hz};
  float p6[3] = {center.x() + hx, center.y() + hy, center.z() + hz};
  float p7[3] = {center.x() - hx, center.y() + hy, center.z() + hz};

  dsDrawLine(p0, p1);
  dsDrawLine(p1, p2);
  dsDrawLine(p2, p3);
  dsDrawLine(p3, p0);
  dsDrawLine(p4, p5);
  dsDrawLine(p5, p6);
  dsDrawLine(p6, p7);
  dsDrawLine(p7, p4);
  dsDrawLine(p0, p4);
  dsDrawLine(p1, p5);
  dsDrawLine(p2, p6);
  dsDrawLine(p3, p7);
}

void VisualizationManager::drawEllipsoid(
    const Eigen::Vector3d &pos, const Eigen::Vector3d &singular_values,
    const Eigen::Matrix3d &principal_directions, float r, float g, float b,
    float alpha) {
  // Save current OpenGL matrix mode
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  // 1. Translation
  glTranslatef(static_cast<float>(pos.x()), static_cast<float>(pos.y()),
               static_cast<float>(pos.z()));

  // 2. Rotation
  // Convert rotation matrix to OpenGL format (column-major)
  // We need to create a 4x4 matrix
  float rot[16];
  rot[0] = static_cast<float>(principal_directions(0, 0));
  rot[1] = static_cast<float>(principal_directions(1, 0));
  rot[2] = static_cast<float>(principal_directions(2, 0));
  rot[3] = 0.0f;

  rot[4] = static_cast<float>(principal_directions(0, 1));
  rot[5] = static_cast<float>(principal_directions(1, 1));
  rot[6] = static_cast<float>(principal_directions(2, 1));
  rot[7] = 0.0f;

  rot[8] = static_cast<float>(principal_directions(0, 2));
  rot[9] = static_cast<float>(principal_directions(1, 2));
  rot[10] = static_cast<float>(principal_directions(2, 2));
  rot[11] = 0.0f;

  rot[12] = 0.0f;
  rot[13] = 0.0f;
  rot[14] = 0.0f;
  rot[15] = 1.0f;

  glMultMatrixf(rot);

  // 3. Scaling (Singular values are expected to be the semi-axes lengths)
  // Scaling by singular values to transform the sphere into an ellipsoid
  glScalef(static_cast<float>(singular_values.x()),
           static_cast<float>(singular_values.y()),
           static_cast<float>(singular_values.z()));

  // 4. Draw Sphere
  dsSetColorAlpha(r, g, b, alpha);
  float pos_dummy[3] = {0, 0, 0};
  float R_dummy[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  dsDrawSphere(pos_dummy, R_dummy, 1.0f); // Radius 1.0, scaled by glScalef

  glPopMatrix();
}

/* Temporarily commented out for incremental refactoring
void VisualizationManager::drawManipulabilityViz(const SimulationState &state) {
  if (state.manip_viz_mode == ManipulabilityVizMode::OFF)
    return;

  if (state.fk_chain_ptr) {
    state.fk_chain_ptr->forwardKinematics();
  }

  debug_counter_++;

  // 1. Current Pose Manipulability
  if (state.manip_viz_mode == ManipulabilityVizMode::CURRENT_ONLY ||
      state.manip_viz_mode == ManipulabilityVizMode::ALL_NODES) {
    if (state.fk_chain_ptr) {
      // Calculate Jacobian at the TIP (index = num_joints + 1)
      int tip_index = state.fk_chain_ptr->getNumJoints() + 1;
      Eigen::Vector3d tip_pos = state.fk_chain_ptr->getEEFPosition();

      Eigen::MatrixXd J6 = state.fk_chain_ptr->calculateJacobian(tip_index);
      Eigen::MatrixXd J_linear = J6.topRows(3);
      auto ellipsoid =
          Manipulability::calculateManipulabilityEllipsoid(J_linear);

      if (ellipsoid.valid) {
        if (debug_counter_ % 120 == 0) {
          printf("[Manipulability] Tip=(%.3f,%.3f,%.3f) M=%.4f "
                 "SV=(%.3f,%.3f,%.3f)\n",
                 tip_pos.x(), tip_pos.y(), tip_pos.z(),
                 ellipsoid.manipulability, ellipsoid.singular_values.x(),
                 ellipsoid.singular_values.y(), ellipsoid.singular_values.z());
          fflush(stdout);
        }
        drawEllipsoid(tip_pos,
                      ellipsoid.singular_values *
                          (double)state.manip_ellipsoid_scale,
                      ellipsoid.principal_directions, 1.0f, 0.2f, 1.0f, 0.8f);
      }
    }
  }

  // 2. All Nodes Manipulability (if mode is ALL_NODES)
  if (state.manip_viz_mode == ManipulabilityVizMode::ALL_NODES) {
    if (state.fk_chain_ptr) {
      int num_joints = state.fk_chain_ptr->getNumJoints();
      for (int i = 1; i <= num_joints; ++i) {
        Eigen::Vector3d joint_pos = state.fk_chain_ptr->getJointPosition(i - 1);
        Eigen::MatrixXd J6 = state.fk_chain_ptr->calculateJacobian(i);
        if (J6.cols() < 1)
          continue;

        Eigen::MatrixXd J_linear = J6.topRows(3);
        auto ellipsoid =
            Manipulability::calculateManipulabilityEllipsoid(J_linear);

        if (ellipsoid.valid) {
          drawEllipsoid(joint_pos,
                        ellipsoid.singular_values *
                            (double)state.manip_ellipsoid_scale,
                        ellipsoid.principal_directions, 0.2f, 1.0f, 0.2f, 0.6f);
        }
      }
    }
  }
}
*/

void VisualizationManager::drawTargetSphere(const Eigen::Vector3d &pos,
                                            const Eigen::Quaterniond &quat,
                                            double radius, bool show_axes) {
  float pos_f[3] = {(float)pos.x(), (float)pos.y(), (float)pos.z()};

  Eigen::Matrix3d R = quat.toRotationMatrix();
  float R_f[12] = {(float)R(0, 0), (float)R(0, 1), (float)R(0, 2), 0.0f,
                   (float)R(1, 0), (float)R(1, 1), (float)R(1, 2), 0.0f,
                   (float)R(2, 0), (float)R(2, 1), (float)R(2, 2), 0.0f};

  dsSetColor(0.0f, 0.0f, 1.0f); // Blue
  dsDrawSphere(pos_f, R_f, (float)radius);

  if (show_axes) {
    float axis_length = 0.1f;
    // X axis (red)
    Eigen::Vector3d x_axis = R.col(0) * axis_length;
    float px[3] = {(float)(pos.x() + x_axis.x()), (float)(pos.y() + x_axis.y()),
                   (float)(pos.z() + x_axis.z())};
    dsSetColor(1.0f, 0.0f, 0.0f);
    dsDrawLine(pos_f, px);

    // Y axis (green)
    Eigen::Vector3d y_axis = R.col(1) * axis_length;
    float py[3] = {(float)(pos.x() + y_axis.x()), (float)(pos.y() + y_axis.y()),
                   (float)(pos.z() + y_axis.z())};
    dsSetColor(0.0f, 1.0f, 0.0f);
    dsDrawLine(pos_f, py);

    // Z axis (blue)
    Eigen::Vector3d z_axis = R.col(2) * axis_length;
    float pz[3] = {(float)(pos.x() + z_axis.x()), (float)(pos.y() + z_axis.y()),
                   (float)(pos.z() + z_axis.z())};
    dsSetColor(0.0f, 0.0f, 1.0f);
    dsDrawLine(pos_f, pz);
  }
}

void VisualizationManager::drawCandidatePaths(
    const std::vector<CandidatePathVisualization> &paths) {
  for (const auto &candidate : paths) {
    // Draw path as connected lines
    for (size_t i = 0; i + 1 < candidate.path.size(); ++i) {
      const auto &p1 = candidate.path[i];
      const auto &p2 = candidate.path[i + 1];

      float pos1[3] = {p1.x(), p1.y(), p1.z()};
      float pos2[3] = {p2.x(), p2.y(), p2.z()};

      dsSetColor(candidate.color.x(), candidate.color.y(), candidate.color.z());
      dsDrawLine(pos1, pos2);
    }
  }
}

void VisualizationManager::drawObstacleVoxels(const SimulationState &state) {
  if (!state.show_obstacle_voxels || !state.obstacle_manager_ptr)
    return;

  std::vector<long> debug_voxels;
  for (long vid : state.obstacle_manager_ptr->getUnifiedOccupiedVoxels()) {
    debug_voxels.push_back(vid);
  }
  float vsize = (float)state.obstacle_manager_ptr->getVoxelSize();
  drawObstacleVoxels(debug_voxels, vsize);
}

void VisualizationManager::drawObstacleVoxels(
    const std::vector<long> &occupied_voxels, float voxel_size) {
  dsSetColor(1.0f, 1.0f, 0.0f); // Yellow wireframes for voxels

  for (long vid : occupied_voxels) {
    Eigen::Vector3i idx =
        GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
    Eigen::Vector3f center = idx.cast<float>() * voxel_size +
                             Eigen::Vector3f::Constant(voxel_size * 0.5f);
    drawWireframeBox(center, Eigen::Vector3f::Constant(voxel_size));
  }
}

void VisualizationManager::drawDangerField(const SimulationState &state) {
  if (!state.show_danger_field || !state.safety_state_manager_ptr)
    return;

  const auto &debug_voxels = state.safety_state_manager_ptr->getDebugVoxels();
  dsSetColorAlpha(1.0f, 0.0f, 1.0f, 0.5f); // Base color

  for (const auto &voxel : debug_voxels) {
    // Color based on danger
    float danger = voxel.danger;
    float r = 0, g = 0, b = 0;
    if (danger > 0.5f) {
      // Interpolate from Yellow (at 0.5) to Red (at 1.0)
      r = 1.0f;
      g = 1.0f - (danger - 0.5f) * 2.0f;
      b = 0.0f;
    } else {
      // Interpolate from Black (at 0.0) to Yellow (at 0.5)
      r = danger * 2.0f;
      g = danger * 2.0f;
      b = 0.0f;
    }
    dsSetColor(r, g, b);

    // Using generic drawWireframeBox
    drawWireframeBox(voxel.center.cast<float>(), voxel.size.cast<float>());
  }
}

void VisualizationManager::drawPath(const std::vector<Eigen::Vector3d> &points,
                                    float r, float g, float b) {
  if (points.size() < 2)
    return;

  dsSetColor(r, g, b);
  for (size_t i = 0; i < points.size() - 1; ++i) {
    float p1[3] = {(float)points[i].x(), (float)points[i].y(),
                   (float)points[i].z()};
    float p2[3] = {(float)points[i + 1].x(), (float)points[i + 1].y(),
                   (float)points[i + 1].z()};
    dsDrawLine(p1, p2);
  }
}

void VisualizationManager::drawSphere(const Eigen::Vector3d &center,
                                      float radius, float r, float g, float b,
                                      float alpha) {
  dsSetColorAlpha(r, g, b, alpha);
  float pos[3] = {(float)center.x(), (float)center.y(), (float)center.z()};
  float R[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  dsDrawSphere(pos, R, radius);
}

void VisualizationManager::drawInfluenceNodes(const SimulationState &state) {
  if (!state.enable_influence_tracking || !state.influence_manager_ptr ||
      !state.gng_ptr) {
    return;
  }

  const auto &active_nodes = state.influence_manager_ptr->getActiveNodes();
  float R_id[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

  for (const auto &node_info : active_nodes) {
    // GNGノードからタスク空間位置（EEF位置）を取得
    const auto &gng_node = state.gng_ptr->nodeAt(node_info.node_id);
    Eigen::Vector3f pos_f = gng_node.weight_coord;
    float pos[3] = {pos_f.x(), pos_f.y(), pos_f.z()};

    // ラベルに応じた色設定（通常のGNG表示より鮮やかに）
    if (node_info.danger_level > 0.8f) {
      dsSetColor(1.0f, 0.0f, 0.0f); // 赤（衝突/極めて危険）
    } else if (node_info.danger_level > 0.1f) {
      dsSetColor(1.0f, 0.4f, 0.0f); // 鮮やかなオレンジ
    } else {
      dsSetColor(0.0f, 0.8f, 1.0f); // 鮮やかなスカイブルー
    }

    dsDrawSphere(
        pos, R_id,
        0.025f); // 半径を1/2(0.025f)に縮小。通常の0.02よりわずかに大きいサイズ。
  }
}

} // namespace simulation
} // namespace robot_sim
