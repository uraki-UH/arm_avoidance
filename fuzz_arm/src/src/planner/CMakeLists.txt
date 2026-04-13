# Planner is currently a header-only module
add_library(planner_lib INTERFACE)
target_link_libraries(planner_lib INTERFACE common_lib kinematics_lib collision_lib Eigen3::Eigen)
target_include_directories(planner_lib INTERFACE .)
# Add subdirectories if they exist and contain CMakeLists.txt
# add_subdirectory(RRT) # Currently RRT is also header-only
