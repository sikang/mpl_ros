# ROS Wrapper for MRSL Motion Primitive Library
=====================

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
<img src="./test_node/maps/simple/simple.png" width="96"> |<img src="./test_node/maps/simple/levine.png" width="96"> |<img src="./test_node/maps/skir/skir.png" width="96"> |<img src="./test_node/maps/office/office.png" width="96"> 

