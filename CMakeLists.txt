cmake_minimum_required(VERSION 2.8)
project(motion_primitive_library)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")


IF(USE_ROS)
  message(STATUS "Compile ROS utils.")
  # roslib cmake_modules
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
    DEPENDS Eigen3 
    CATKIN_DEPENDS 
    INCLUDE_DIRS include
    LIBRARIES primitive poly_solver vision_util vision_queue_util mp_map_util mp_vision_util
    )
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED )
include_directories(./include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})



add_library(primitive src/primitive/primitive.cpp
  src/primitive/primitive_util.cpp
  src/primitive/trajectory.cpp
  src/primitive/math.cpp)

add_library(poly_solver src/primitive/poly_solver.cpp
  src/primitive/poly_traj.cpp)
target_link_libraries(poly_solver primitive)

add_library(mp_map_util src/planner/mp_map_util.cpp
    src/planner/astar.cpp)
target_link_libraries(mp_map_util primitive)


add_library(vision_util src/collision_checking/vision_util.cpp)
target_link_libraries(vision_util ${OpenCV_LIBS} primitive)
add_library(vision_queue_util src/collision_checking/vision_queue_util.cpp)
target_link_libraries(vision_queue_util vision_util)

add_library(mp_vision_util src/planner/mp_vision_util.cpp
    src/planner/astar.cpp)
target_link_libraries(mp_vision_util primitive vision_queue_util)



