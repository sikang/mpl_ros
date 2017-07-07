cmake_minimum_required(VERSION 2.8.3)
project(test_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(test_primitive_map src/test_primitive_map.cpp)
#cs_add_executable(test_primitive_sfc src/test_primitive_sfc.cpp)
#cs_add_executable(test_sfc src/test_sfc.cpp)
#cs_add_executable(sfc_server src/sfc_server.cpp)
#cs_add_executable(test_bfs src/test_bfs.cpp)

cs_install()

cs_export()
