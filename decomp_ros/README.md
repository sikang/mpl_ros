MRSL Decomp Util ROS
===============
A ROS wrapper for implementing [`DecompUtil`](https://github.com/sikang/DecompUtil.git), stacks include:
  - `DecompUtil`: convex decomposition of free space in a cluttered environment
  - `decomp_ros_msgs`: ROS msgs used in storing, visualizing and communicating
  - `decomp_ros_utils`: ROS utils for interfacing with `DecompUtil`
  - `test_node`: examples

## Compilation
Prerequisite:
  - `ROS`(Indigo+)
  - [`catkin_simple`](https://github.com/catkin/catkin_simple)
  - `QT`(4+)

Using Catkin:
```sh
$ mv decomp_ros ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make -DCMAKE_BUILD_TYPE=Release
```
Using Catkin Tools:
```sh
$ mv decomp_ros ~/catkin_ws/src
$ catkin config -DCMAKE_BUILD_TYPE=Release
$ cd ~/catkin_ws & catkin build
```

## Usage
Simple test using the built-in data can be applied through following commands:
```sh
$ roscd test_node/launch
$ roslaunch rviz.launch
$ roslaunch test_decomp.launch
```

The input point cloud, path and output Safe Flight Corridor after decomposition are visualized in Rviz as:

<img src="./test_node/samples/sample1.png" width="256"> <img src="./test_node/samples/sample2.png" width="256">

Note there are several params and modes you can choose to do the decomposition, the results vary according to different set up.
