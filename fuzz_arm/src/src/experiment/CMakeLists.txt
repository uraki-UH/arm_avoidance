add_library(experiment_lib STATIC
    experiment_config.cpp
    scenario_definition.cpp
)
target_link_libraries(experiment_lib PUBLIC common_lib kinematics_lib control_lib Eigen3::Eigen)
target_include_directories(experiment_lib PUBLIC .)
