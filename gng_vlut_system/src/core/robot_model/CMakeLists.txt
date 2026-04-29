# Robot Description and Geometric Loaders
set(DESCRIPTION_SRCS
    urdf_loader.cpp
    stl_loader.cpp
    geometry_management.cpp
    kinematic_adapter.cpp
)

add_library(robot_model_lib STATIC ${DESCRIPTION_SRCS})
set_target_properties(robot_model_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(robot_model_lib PUBLIC .)
target_link_libraries(robot_model_lib common_lib kinematics_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(robot_model_lib 
        urdf 
        urdfdom 
        urdfdom_headers
        Eigen3
    )
else()
    target_link_libraries(robot_model_lib 
        ${urdfdom_LIBRARIES} 
        Eigen3::Eigen
    )
endif()
