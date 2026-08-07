cmake_minimum_required(VERSION 3.10)
project(gng_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EvaluationMetrics.msg"
  "msg/GraspCandidateMetric.msg"
  "msg/GraspCandidateMetricArray.msg"
  "msg/GraspState.msg"
  "msg/JointControlClaim.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
