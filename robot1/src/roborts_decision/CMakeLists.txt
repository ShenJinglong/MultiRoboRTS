project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

add_executable(behavior_test_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  behavior_test.cpp
  executor/chassis_executor.cpp
  )

add_executable(teleop_test_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/teleop_behavior.h
  executor/chassis_executor.cpp
  teleop_test.cpp
)

add_executable(behavior_tree_test_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  blackboard/blackboard.h
  example_behavior/patrol_behavior.h
  behavior_tree/behavior_node.h
  behavior_tree/behavior_tree.h
  executor/chassis_executor.cpp
  behavior_tree_test.cpp
)

add_executable(fake_info_node
  fake_info_node.cpp
)

target_link_libraries(behavior_test_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )

target_link_libraries(teleop_test_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

target_link_libraries(behavior_tree_test_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

target_link_libraries(fake_info_node
  ${catkin_LIBRARIES}
)

add_dependencies(behavior_test_node
  roborts_msgs_generate_messages)

add_dependencies(teleop_test_node
  roborts_msgs_generate_messages)

add_dependencies(behavior_tree_test_node
  roborts_msgs_generate_messages
)