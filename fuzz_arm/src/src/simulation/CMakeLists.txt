add_library(simulation_lib STATIC
    core/experiment_logger.cpp
    core/simulation_app.cpp
    core/simulation_state.cpp
    core/udp_command_bridge.cpp
    planning/experiment_controller.cpp
    planning/planning_coordinator.cpp
    planning/scenario_manager.cpp
    planning/ode_state_validity_checker.cpp
    robot/geometry_management.cpp
    robot/kinematic_adapter.cpp
    robot/robot_collision_model.cpp
    robot/stl_loader.cpp
    robot/urdf_loader.cpp
    robot/ode/ode_collision_manager.cpp
    robot/ode/ode_robot_builder.cpp
    robot/ode/ode_robot_sim.cpp
    robot/ode/ode_gng_visualizer.cpp
    safety/influence_management.cpp
    safety/safety_management.cpp
    sensing/robot_point_cloud_sampler.cpp
    viz/gui_manager.cpp
    viz/visualization_manager.cpp
    world/environment_manager.cpp
    world/sim_obstacle_controller.cpp
    reactive_controller.cpp
)
target_link_libraries(simulation_lib PUBLIC 
    common_lib kinematics_lib hardware_lib collision_lib control_lib experiment_lib 
    gng_lib description_lib status_lib spatial_lib 
    Eigen3::Eigen ${urdfdom_LIBRARIES} ${ODE_LIBRARY} ${DRAWSTUFF_LIBRARY} ${OpenGL_LIBRARIES} ${GLUT_LIBRARIES}
)
target_include_directories(simulation_lib PUBLIC .)
