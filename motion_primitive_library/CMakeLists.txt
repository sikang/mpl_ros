cmake_minimum_required(VERSION 2.8)
project(motion_primitive_library)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")
set(USE_ROS TRUE)

IF(USE_ROS)
  MESSAGE(STATUS "Compile with ROS.")
  # roslib cmake_modules
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
    DEPENDS EIGEN3
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES primitive poly_solver mp_map_util mp_cloud_util
    )
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()

FIND_PACKAGE(VTK REQUIRED)

IF(NOT VTK_FOUND)
  MESSAGE(STATUS "VTK was not found")
ELSE()
  INCLUDE(${VTK_USE_FILE})
  MESSAGE(STATUS "VTK was found")
ENDIF()

FIND_PACKAGE(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE(Eigen3 REQUIRED)
#FIND_PACKAGE(OpenCV REQUIRED )
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
#FIND_PACKAGE( Boost COMPONENTS filesystem system REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} include)
#include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} include)

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
  #MESSAGE("YAMLCPP_INCLUDE_DIRS: ${YAMLCPP_INCLUDE_DIRS}")
  #MESSAGE("YAMLCPP_LIBRARIES: ${YAMLCPP_LIBRARIES}")
  #MESSAGE("YAMLCPP_LIBRARY_DIRS: ${YAMLCPP_LIBRARY_DIRS}")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()



add_library(primitive src/primitive/primitive.cpp
  src/primitive/primitive_util.cpp
  src/primitive/trajectory.cpp
  src/primitive/math.cpp)

add_library(poly_solver src/primitive/poly_solver.cpp
  src/primitive/poly_traj.cpp)
target_link_libraries(poly_solver primitive)

add_library(mp_map_util src/planner/mp_map_util.cpp
  src/planner/mp_base_util.cpp
  src/planner/graph_search.cpp)
target_link_libraries(mp_map_util primitive)


add_library(mp_cloud_util src/planner/mp_cloud_util.cpp
  src/planner/mp_base_util.cpp
  src/collision_checking/decomp_util.cpp
  src/planner/graph_search.cpp)
target_link_libraries(mp_cloud_util primitive ${PCL_LIBRARIES})


#add_library(mp_sfc_util src/planner/mp_sfc_util.cpp
#  src/planner/mp_base_util.cpp
#  src/collision_checking/sfc_util.cpp
#  src/planner/astar.cpp)
#target_link_libraries(mp_sfc_util primitive)


add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d mp_map_util ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

