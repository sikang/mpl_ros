# MRSL Motion Primitive Library ROS
[![wercker status](https://app.wercker.com/status/d282a628f39dac13997c792b2298bde0/s/master "wercker status")](https://app.wercker.com/project/byKey/d282a628f39dac13997c792b2298bde0)
- - -
A ROS wrapper for [Motion Primitive Library](https://sikang.github.io/motion_primitive_library/) v1.1. Video of the original paper of "Search-based Motion Planning for Quadrotors using Linear Quadratic Minimum Time Control" has been uploaded at the follwing link: [youtube](https://youtu.be/LMe72buMky8).
The package is still under maintenance, the API may change occasionally, please use `git log` to track the latest update.

Packages:
  - `motion_primitive_library`: back-end for planning trajectory in various environments
  - `planning_ros_msgs`: ROS msgs used in storing, visualizing and communicating
  - `planning_ros_utils`: ROS utils for interfacing with MPL, it also includes mapping and rviz plugins
  - `DecompROS`: convex decomposition tool
  - `mpl_external_planner`: planner that uses the
  - `mpl_test_node`: examples code for simple testing

## Installation
#### Dependancy:
  - `ROS`(Indigo+)
  - `SDL`(`sudo apt install -y libsdl1.2-dev libsdl-image1.2-dev`)
  - [`catkin_simple`](https://github.com/catkin/catkin_simple)

To initialize the submodule `motion_primitive_library` and `DecompROS`, run following commands at first:
```bash
$ cd /PATH/TO/mpl_ros
$ git submodule update --init --recursive
```

#### 1) Using Catkin:
```bash
$ mv mpl_ros ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

#### 2) Using Catkin Tools (recommended):
```bash
$ mv mpl_ros ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin config -DCMAKE_BUILD_TYPE=Release
$ catkin b
```

## Example 1 (plan in occ/voxel map)
Simple test using the built-in data in a voxel map can be run using the following commands:
```bash
$ cd ./mpl_test_node/launch/map_planner_node
$ roslaunch rviz.launch
$ roslaunch test.launch
```
It also extracts the control commands for the generated trajectory and saves as
`trajectory_commands.bag`.


The planning results are visualized in Rviz as following:

2D Occ Map | 3D Voxel Map
:--------- | :-----------
<img src="./mpl_test_node/samples/sample1.png" width="220"> | <img src="./mpl_test_node/samples/sample2.png" width="256">


## Example 2 (plan in polygonal map)
The planner can also take input polygonal map for collision checking. When the
obstacles are not static, it's able to find the trajectory that avoids future
collision:
```bash
$ cd ./mpl_test_node/launch/poly_map_planner_node
$ roslaunch rviz.launch
$ roslaunch test.launch
```
Static Obstacles | Moving Obtacles
:--------------- | :--------------
<img src="./mpl_test_node/samples/sample4.png" width="328"> | <img src="./mpl_test_node/samples/sample5.png" width="328">

<img src="./mpl_test_node/samples/sample1.gif" width="696">

Even if the trajectories of obstacles are non-linear, our planner could find the optimal maneuver for the robot with certain dynamic constraints through one plan:
```bash
$ cd ./mpl_test_node/launch/nonlinear_obstacle_node
$ roslaunch rviz.launch
$ roslaunch test.launch
```

<img src="./mpl_test_node/samples/sample2.gif" width="696">

## Example 3 (multi-robot decentralized planning)
We can build a team of robots that move simultaneously in a constrained environments without internal collision.
In the following demo, each robot replans constantly at 2Hz and it runs its own planner that tries to reach the pre-allocate goal as fast as possible.
We assume the robot knows other robots' trajectory at the time when it's planning.

Star  | Tunnel
:---- | :----
<img src="./mpl_test_node/samples/sample3.gif" width="328"> | <img src="./mpl_test_node/samples/sample4.gif" width="328">



## Example 4 (plan in SE(3) with ellispoid model)
Another example using ellipsoid model can be found in `mpl_test_node/launch/ellipsoid_planner_node`, in which a point cloud is used as obstacles, and the robot is modeled as the ellipsoid. More information can be found in the paper ["Search-based Motion Planning for Aggressive Flight in SE(3)"](http://ieeexplore.ieee.org/document/8264768/).
```bash
$ cd ./mpl_test_node/launch/ellispoid_planner_node
$ roslaunch rviz.launch
$ roslaunch test.launch
```
<img src="./mpl_test_node/samples/sample3.png" width="328">

## Example Maps
The built-in maps are listed as below:

Simple | Levine | Skir | Office
:----- | :----- | :--- | :-----
<img src="./mpl_test_node/maps/simple/simple.png" width="156"> |<img src="./mpl_test_node/maps/levine/levine.png" width="156"> |<img src="./mpl_test_node/maps/skir/skir.png" width="156"> |<img src="./mpl_test_node/maps/office/office.png" width="156">

User can form their own maps using the `mapping_utils`, a launch file example is provided in `./mpl_test_node/launch/map_generator` for converting a STL file into voxel map.
For details about the full utilities, please refer to [wiki](https://github.com/sikang/mpl_ros/wiki).

