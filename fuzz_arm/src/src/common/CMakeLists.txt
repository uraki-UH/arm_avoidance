add_library(common_lib STATIC
    config_manager.cpp
    geometry_utils.cpp
)
target_link_libraries(common_lib PUBLIC Eigen3::Eigen)
target_include_directories(common_lib PUBLIC .)
