# Simulation Core Module (Required for Offline GNG/VLUT)
set(SIM_CORE_SRCS
    robot/geometry_management.cpp
    robot/kinematic_adapter.cpp
    robot/robot_collision_model.cpp
    robot/stl_loader.cpp
    robot/urdf_loader.cpp
    sensing/robot_point_cloud_sampler.cpp
)

# ... (other srcs same as before)
set(SIM_APP_SRCS
    core/experiment_logger.cpp
    core/simulation_app.cpp
    core/simulation_state.cpp
    core/udp_command_bridge.cpp
    planning/experiment_controller.cpp
    planning/planning_coordinator.cpp
    planning/scenario_manager.cpp
    planning/ode_state_validity_checker.cpp
    safety/influence_management.cpp
    safety/safety_management.cpp
    reactive_controller.cpp
)

# set(SIM_ODE_SRCS
#     robot/ode/ode_collision_manager.cpp
#     robot/ode/ode_robot_builder.cpp
#     robot/ode/ode_robot_sim.cpp
#     robot/ode/ode_gng_visualizer.cpp
#     world/environment_manager.cpp
#     world/sim_obstacle_controller.cpp
# )

# set(SIM_VIZ_SRCS
#     viz/gui_manager.cpp
#     viz/visualization_manager.cpp
# )

# --- Core Library ---
add_library(simulation_lib STATIC ${SIM_CORE_SRCS})
target_include_directories(simulation_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(simulation_lib  
    common_lib kinematics_lib collision_lib description_lib
)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(simulation_lib 
        urdf 
        urdfdom 
        Eigen3
    )
else()
    target_link_libraries(simulation_lib  
        ${urdfdom_LIBRARIES} 
        Eigen3::Eigen
    )
endif()

# # --- Full Simulation Library (Only if ODE/Drawstuff available) ---
# if(USE_ODE AND USE_DRAWSTUFF)
#     add_library(simulation_full_lib STATIC 
#         ${SIM_APP_SRCS} 
#         ${SIM_ODE_SRCS} 
#         ${SIM_VIZ_SRCS}
#     )
#     target_link_libraries(simulation_full_lib  
#         simulation_lib hardware_lib control_lib experiment_lib 
#         gng_lib status_lib spatial_lib
#         ${ODE_LIBRARY} ${DRAWSTUFF_LIBRARY}
#     )
# endif()
