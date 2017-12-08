cmake_minimum_required(VERSION 2.8.3)
project(decomp_test_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(test_path_decomp src/test_path_decomp.cpp)

cs_add_executable(test_seed_decomp src/test_seed_decomp.cpp)

cs_install()

cs_export()

