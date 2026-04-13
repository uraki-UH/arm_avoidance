add_library(hardware_lib STATIC
    twin_manager.cpp
)
target_link_libraries(hardware_lib PUBLIC common_lib Eigen3::Eigen)
target_include_directories(hardware_lib PUBLIC .)

# UDP Controller executable
add_executable(udp_controller udp_controller_main.cpp)
target_link_libraries(udp_controller PRIVATE hardware_lib Threads::Threads)
