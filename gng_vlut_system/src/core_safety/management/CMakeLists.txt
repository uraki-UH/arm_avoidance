set(OFFLINE_SAFETY_SRCS
    influence_management.cpp
    safety_management.cpp
)

add_library(core_safety_offline STATIC ${OFFLINE_SAFETY_SRCS})
target_include_directories(core_safety_offline PUBLIC .)
target_link_libraries(core_safety_offline core_safety description_lib)
