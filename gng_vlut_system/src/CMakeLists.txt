add_subdirectory(common)
add_subdirectory(core_safety)
add_subdirectory(kinematics)
add_subdirectory(description)
add_subdirectory(collision)
add_subdirectory(status)
add_subdirectory(spatial)

# Legacy / Non-essential modules (Excluded from strict safety build)

if(BUILD_WITH_ROS2)
  add_subdirectory(bridge)
  add_subdirectory(safety)
  add_subdirectory(self_recognition)
  add_subdirectory(tools)

  # --- Offline Training Tools ---
  # 1. Unified URDF Trainer (GNG + VLUT)
  add_executable(offline_urdf_trainer offline_urdf_trainer.cpp)
  target_link_libraries(offline_urdf_trainer
      core_safety
      collision_lib
      description_lib
      kinematics_lib
      common_lib
      status_lib
      spatial_lib
  )
  ament_target_dependencies(offline_urdf_trainer fcl Eigen3 urdfdom)
  target_link_libraries(offline_urdf_trainer fcl)
  install(TARGETS offline_urdf_trainer DESTINATION lib/gng_safety)

  
endif()
