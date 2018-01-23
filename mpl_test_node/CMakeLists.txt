cmake_minimum_required(VERSION 2.8.3)
project(mpl_test_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_simple()

cs_add_executable(test_primitive_map src/test_primitive_map.cpp)
cs_add_executable(test_primitive_replan src/test_primitive_replan.cpp)
cs_add_executable(test_primitive_cloud src/test_primitive_cloud.cpp)
cs_add_executable(test_rss src/test_rss.cpp)
cs_add_executable(rss_evaluate1 src/rss_evaluate1.cpp)

cs_install()

cs_export()
