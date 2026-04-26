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
  target_compile_options(offline_urdf_trainer PRIVATE
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
  )
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

  # 2. Offline GNG Main 5 Clean
  add_executable(offline_gng_main5_clean offline_gng_main5_clean.cpp)
  target_compile_options(offline_gng_main5_clean PRIVATE
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
  )
  target_link_libraries(offline_gng_main5_clean
      core_safety
      collision_lib
      description_lib
      kinematics_lib
      common_lib
      status_lib
      spatial_lib
  )
  ament_target_dependencies(offline_gng_main5_clean fcl Eigen3 urdfdom)
  target_link_libraries(offline_gng_main5_clean fcl)
  install(TARGETS offline_gng_main5_clean DESTINATION lib/gng_safety)

  # 3. Offline GNG Main 6 Island Pruning
  add_executable(offline_gng_main6_island_pruning offline_gng_main6_island_pruning.cpp)
  target_compile_options(offline_gng_main6_island_pruning PRIVATE
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
  )
  target_link_libraries(offline_gng_main6_island_pruning
      core_safety
      collision_lib
      description_lib
      kinematics_lib
      common_lib
      status_lib
      spatial_lib
  )
  ament_target_dependencies(offline_gng_main6_island_pruning fcl Eigen3 urdfdom)
  target_link_libraries(offline_gng_main6_island_pruning fcl)
  install(TARGETS offline_gng_main6_island_pruning DESTINATION lib/gng_safety)

  # 4. Offline GNG Status Updater
  add_executable(offline_gng_status_updater offline_gng_status_updater.cpp)
  target_compile_options(offline_gng_status_updater PRIVATE
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
  )
  target_link_libraries(offline_gng_status_updater
      core_safety
      collision_lib
      description_lib
      kinematics_lib
      common_lib
      status_lib
      spatial_lib
  )
  ament_target_dependencies(offline_gng_status_updater fcl Eigen3 urdfdom)
  target_link_libraries(offline_gng_status_updater fcl)
  install(TARGETS offline_gng_status_updater DESTINATION lib/gng_safety)
  
endif()
