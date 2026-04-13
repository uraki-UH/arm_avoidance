add_library(gng_lib STATIC
    GrowingNeuralGas_offline2.cpp
)
target_link_libraries(gng_lib PUBLIC common_lib kinematics_lib Eigen3::Eigen)
target_include_directories(gng_lib PUBLIC .)
