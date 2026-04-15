add_library(hardware_lib STATIC
    twin_manager.cpp
)
target_include_directories(hardware_lib PUBLIC .)
target_link_libraries(hardware_lib PUBLIC common_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(hardware_lib Eigen3)
else()
    target_link_libraries(hardware_lib PUBLIC Eigen3::Eigen)
endif()

# UDP Controller executable
if(NOT BUILD_WITH_ROS2)
    add_executable(udp_controller udp_controller_main.cpp)
    target_link_libraries(udp_controller PRIVATE hardware_lib Threads::Threads)
endif()
