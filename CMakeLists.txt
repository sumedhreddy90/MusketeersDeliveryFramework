cmake_minimum_required(VERSION 3.0.2)
project(musketeers)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(listener src/main.cpp)

# target_link_libraries(talker
#   ${catkin_LIBRARIES}
# )