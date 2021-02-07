cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cassie_interface)

cmake_policy(SET CMP0054 NEW)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp gazebo_ros cassie_estimation
)
find_package(GAZEBO REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS  cassie_estimation
   CFG_EXTRAS cassie_interface-extras.cmake
)

# Specify custom includes and libs
include_directories(include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "-Wno-deprecated-declarations -Wno-dev ${GAZEBO_CXX_FLAGS}")

# Create UDP interface library and install
add_library(cassieplugin SHARED src/udp.c src/CassiePlugin.cpp)
add_dependencies(cassieplugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(cassieplugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libagilitycassie.a)

# Build the core interface node
add_executable(cassie_interface_node src/cassie_interface_node.cpp src/udp.c)
target_link_libraries(cassie_interface_node ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libagilitycassie.a)
