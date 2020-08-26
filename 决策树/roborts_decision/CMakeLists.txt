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
  roborts_base
  actionlib_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

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
  action_node/BackBootArea.h
  action_node/ChaseAction.h
  action_node/ChassisLimited.h
  action_node/DefendAction.h
  action_node/EscapeAction.h
  action_node/FollowAction.h
  action_node/FrozeAction.h
  action_node/GainBloodAction.h
  action_node/GainBulletAction.h
  action_node/GimbalLimited.h
  action_node/SearchAction.h
  action_node/TurnToDetectedDirection.h
  action_node/WaitBuffRefresh.h
  action_node/BackAwayAction.h
  action_node/DeckChaseAction.h
  behavior_test.cpp
  behavior_tree/behavior_node.h
  executor/chassis_executor.cpp
  executor/gimbal_executor.cpp
  )

target_link_libraries(behavior_test_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(behavior_test_node
  roborts_msgs_generate_messages)


add_executable(icra_decision_node
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    action_node/BackBootArea.h
    action_node/BackAwayAction.h
    action_node/ChaseAction.h
    action_node/ChassisLimited.h
    action_node/DeckChaseAction.h
    action_node/DefendAction.h
    action_node/EscapeAction.h
    action_node/FollowAction.h
    action_node/FrozeAction.h
    action_node/GainBloodAction.h
    action_node/GainBulletAction.h
    action_node/GimbalLimited.h
    action_node/SearchAction.h
    action_node/TurnToDetectedDirection.h
    action_node/WaitBuffRefresh.h
    icra_decision_node.cpp
    behavior_tree/behavior_node.h
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
    interact/mutualboard.h
 )

target_link_libraries(icra_decision_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )
 add_dependencies( icra_decision_node roborts_base_node
   roborts_msgs_generate_messages)


 add_executable(pos_reciever_node
   ${DecisionProtoHds}
   ${DecisionProtoSrc}
   ./interact/pos_reciever.cpp
   )
 
 target_link_libraries(pos_reciever_node
   PRIVATE
   
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   )
 
 add_dependencies(pos_reciever_node
   roborts_msgs_generate_messages)


 