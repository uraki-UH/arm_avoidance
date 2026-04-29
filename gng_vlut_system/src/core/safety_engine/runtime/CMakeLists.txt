set(OFFLINE_SAFETY_SRCS
    influence_management.cpp
    safety_management.cpp
)

add_library(safety_engine_offline STATIC ${OFFLINE_SAFETY_SRCS})
target_include_directories(safety_engine_offline PUBLIC .)
target_link_libraries(safety_engine_offline safety_engine robot_model_lib)
