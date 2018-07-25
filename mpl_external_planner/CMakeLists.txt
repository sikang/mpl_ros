cmake_minimum_required(VERSION 2.8.3)
project(mpl_external_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(decomp_util REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(test_primitive_collide.cpp src/test_primitive_collide.cpp)

cs_install()

cs_export()
