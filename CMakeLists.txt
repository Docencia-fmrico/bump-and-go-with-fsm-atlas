cmake_minimum_required(VERSION 2.8.3)
project(fsm_bump_go)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behaviortree_cpp_v3
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs behaviortree_cpp_v3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/fsm_bump_go/BaseClass.cpp
  src/fsm_bump_go/BumpGo.cpp
  src/fsm_bump_go/BumpGo_Advanced.cpp
  src/fsm_bump_go/BumpGo_Advanced_Laser.cpp
  src/fsm_bump_go/CrashDetector.cpp
)

add_executable(bumpgo_node src/bumpgo_node.cpp)
target_link_libraries(bumpgo_node ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(bumpgo_nodeAdv src/bumpgo_nodeAdv.cpp)
target_link_libraries(bumpgo_nodeAdv ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(bumpgo_nodeAdvLas src/bumpgo_nodeAdvLas.cpp)
target_link_libraries(bumpgo_nodeAdvLas ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(crashdetector src/crashdetector_node.cpp)
target_link_libraries(crashdetector ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(bumgobehavior_tree 
  src/bum_go_behavior_tree.cpp
  src/fsm_bump_go/Back.cpp
  src/fsm_bump_go/Is_bumped.cpp
  src/fsm_bump_go/Forward.cpp
  src/fsm_bump_go/Turn.cpp
)
target_link_libraries(bumgobehavior_tree ${catkin_LIBRARIES})

roslint_cpp(
 src/bumpgo_node.cpp
 src/bumpgo_nodeAdv.cpp
 src/bumpgo_nodeAdvLas.cpp
 src/fsm_bump_go/BaseClass.cpp
 src/fsm_bump_go/BumpGo.cpp
 src/fsm_bump_go/BumpGo_Advanced.cpp
 src/fsm_bump_go/BumpGo_Advanced_Laser.cpp
 include/fsm_bump_go/BaseClass.h
 include/fsm_bump_go/BumpGo.h
 include/fsm_bump_go/BumpGo_Advanced.h
 include/fsm_bump_go/BumpGo_Advanced_Laser.h

)
