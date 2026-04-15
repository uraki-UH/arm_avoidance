add_library(common_lib STATIC
    config_manager.cpp
    geometry_utils.cpp
)

target_include_directories(common_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(common_lib
        ament_index_cpp
    )

    target_link_libraries(common_lib
        Eigen3::Eigen
    )
else()
    target_link_libraries(common_lib
        Eigen3::Eigen
    )
endif()