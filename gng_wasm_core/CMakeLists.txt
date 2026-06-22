cmake_minimum_required(VERSION 3.8)
project(gng_wasm_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(GNG_WASM_OUTPUT_DIR "" CACHE PATH "Optional output directory for Emscripten-generated JS/WASM artifacts")

add_library(gng_wasm_core
  src/gng_kernel.cpp
  src/wasm_exports.cpp
)
target_include_directories(gng_wasm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(EMSCRIPTEN)
  target_compile_options(gng_wasm_core PRIVATE -fno-exceptions -fno-rtti)
  target_link_options(gng_wasm_core PRIVATE
    "SHELL:-sALLOW_MEMORY_GROWTH=1"
    "SHELL:-sMODULARIZE=1"
    "SHELL:-sEXPORT_ES6=1"
    "SHELL:-sENVIRONMENT=web,worker,node"
    "SHELL:-sEXPORTED_FUNCTIONS=['_gng_wasm_create','_gng_wasm_destroy','_gng_wasm_reset','_gng_wasm_set_points','_gng_wasm_set_config','_gng_wasm_run','_gng_wasm_get_graph_json_size','_gng_wasm_write_graph_json']"
    "SHELL:-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
  )

  add_executable(gng_wasm_core_wasm
    src/gng_kernel.cpp
    src/wasm_exports.cpp
  )
  target_include_directories(gng_wasm_core_wasm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_options(gng_wasm_core_wasm PRIVATE -fno-exceptions -fno-rtti)
  target_link_options(gng_wasm_core_wasm PRIVATE
    "SHELL:-sALLOW_MEMORY_GROWTH=1"
    "SHELL:-sMODULARIZE=1"
    "SHELL:-sEXPORT_ES6=1"
    "SHELL:-sENVIRONMENT=web,worker,node"
    "SHELL:-sNO_EXIT_RUNTIME=1"
    "SHELL:-sNO_ENTRY=1"
    "SHELL:-sSTANDALONE_WASM=1"
    "SHELL:-sEXPORTED_FUNCTIONS=['_gng_wasm_create','_gng_wasm_destroy','_gng_wasm_reset','_gng_wasm_set_points','_gng_wasm_set_config','_gng_wasm_run','_gng_wasm_get_graph_json_size','_gng_wasm_write_graph_json','_gng_wasm_node_count','_gng_wasm_edge_count','_gng_wasm_iteration']"
    "SHELL:-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
  )
  if(GNG_WASM_OUTPUT_DIR)
    add_custom_command(TARGET gng_wasm_core_wasm POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "${GNG_WASM_OUTPUT_DIR}"
      COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE_DIR:gng_wasm_core_wasm>/gng_wasm_core_wasm.js" "${GNG_WASM_OUTPUT_DIR}/gng_wasm_core.js"
      COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE_DIR:gng_wasm_core_wasm>/gng_wasm_core_wasm.wasm" "${GNG_WASM_OUTPUT_DIR}/gng_wasm_core.wasm"
    )
  endif()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS gng_wasm_core
  EXPORT export_gng_wasm_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(gng_wasm_core)
ament_package()
