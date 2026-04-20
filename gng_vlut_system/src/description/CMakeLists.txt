# Robot Description and Geometric Loaders
set(DESCRIPTION_SRCS
    urdf_loader.cpp
    stl_loader.cpp
    geometry_management.cpp
    kinematic_adapter.cpp
)

add_library(description_lib STATIC ${DESCRIPTION_SRCS})
target_include_directories(description_lib PUBLIC .)
target_link_libraries(description_lib common_lib kinematics_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(description_lib 
        urdf 
        urdfdom 
        Eigen3
    )
else()
    target_link_libraries(description_lib 
        ${urdfdom_LIBRARIES} 
        Eigen3::Eigen
    )
endif()
