cmake_minimum_required(VERSION 2.8.3)
project(sonar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sonar
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
   include ${catkin_INCLUDE_DIRS}
)

add_executable(sonar_test src/simpleHCSR04.cpp)
add_dependencies(sonar_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sonar_test
  ${catkin_LIBRARIES}
)
