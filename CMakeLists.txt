cmake_minimum_required(VERSION 2.8)
project(motion_primitive_library)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)

add_library(primitive src/primitive/primitive.cpp
  src/primitive/primitive_util.cpp
  src/primitive/trajectory.cpp
  src/primitive/math.cpp)

add_library(poly_solver src/primitive/poly_solver.cpp
  src/primitive/poly_traj.cpp)
target_link_libraries(poly_solver primitive)

add_library(mrsl_mp_util src/planner/mrsl_mp_util.cpp
    src/planner/astar.cpp)
target_link_libraries(mrsl_mp_util primitive)

if(USE_ROS)
  message(STATUS "Compile ROS utils.")
endif()


IF(USE_ROS)
  # roslib cmake_modules
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
    DEPENDS Eigen3 
    CATKIN_DEPENDS 
    INCLUDE_DIRS include
    LIBRARIES #${PROJECT_NAME}
    )
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()


