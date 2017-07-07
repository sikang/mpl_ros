MRSL Motion Primitive Library ROS
=====================
A ROS wrapper for implementing Motion Primitive Library in planning tasks. Stacks include:
  - `motion_primitive_library`: back-end for planning trajectory in various environments
  - `planning_ros_msgs`: ROS msgs used in storing, visualizing and communicating 
  - 'planning_ros_utils': ROS utils for interfacing with MPL
  - `test_node`: examples

## Compilation
Prerequisite:
  - `ROS`(Indigo+)
  - [`catkin_simple`](https://github.com/catkin/catkin_simple)
  - `QT`(4+)

Using Catkin:
```sh
$ mv jps3d ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make -DCMAKE_BUILD_TYPE=Release
```
Using Catkin Tools:
```sh
$ mv jps3d ~/catkin_ws/src
$ catkin config -DCMAKE_BUILD_TYPE=Release
$ cd ~/catkin_ws & catkin build
```
## Maps List
Simple | Levine | Skir | Office
:----- | :----- | :--- | :-----
<img src="./test_node/maps/simple/simple.png" width="128"> |<img src="./test_node/maps/levine/levine.png" width="128"> |<img src="./test_node/maps/skir/skir.png" width="128"> |<img src="./test_node/maps/office/office.png" width="128"> 

