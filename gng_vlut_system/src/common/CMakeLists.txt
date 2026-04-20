# Find Eigen3
find_package(Eigen3 REQUIRED)

set(_eigen_include_dirs "${Eigen3_INCLUDE_DIRS}")
if(NOT _eigen_include_dirs AND EIGEN3_INCLUDE_DIR)
    set(_eigen_include_dirs "${EIGEN3_INCLUDE_DIR}")
endif()
if(NOT _eigen_include_dirs)
    set(_eigen_include_dirs "/usr/include/eigen3")
endif()

add_library(common_lib STATIC
    config_manager.cpp
    geometry_utils.cpp
)

target_include_directories(common_lib PUBLIC 
    .
    ${_eigen_include_dirs}
)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(common_lib Eigen3 ament_index_cpp)
else()
    target_link_libraries(common_lib Eigen3::Eigen)
endif()
