cmake_minimum_required(VERSION 3.0.2)
project(gps2odom)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  message_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp
)

find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

add_executable(gps2odom src/gps2odom.cpp)
target_link_libraries(gps2odom
  ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES}
)
