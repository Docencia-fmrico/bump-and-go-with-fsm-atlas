cmake_minimum_required(VERSION 2.8.3)
project(fsm_bump_go)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs
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
