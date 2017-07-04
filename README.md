# MRSL Motion Primitive for quadrotor

## Compilation

A) Simple cmake (set USE_ROS to OFF)
```sh
mkdir build && cd build && cmake .. && make
```


B) Using CATKIN with ROS (set USE_ROS to ON)
```sh
$ mv motion_primitive_library ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make -DCMAKE_BUILD_TYPE=Release
```


## Example Usage
The simple API are provided in the base planner class, please look up in Doxygen. Here are several functions to set up a planning thread:
```c++
std::unique_ptr<MPMapUtil> planner(new MPMapUtil(true)); // Declare a mp planner using voxel map
planner->setMapUtil(map_util); // Set collision checking function
planner->setEpsilon(1.0); // Set greedy param (default equal to 1)
planner->setVmax(2.0); // Set max velocity
planner->setAmax(1.0); // Set max acceleration (as control input)
planner->setDt(1.0); // Set dt for each primitive
planner->setMaxNum(5000); // Set maximum allowed states
planner->setMode(1, false); // 2D discretization with 1
planner->setTol(1, 1); // Tolerance for goal region

bool valid = planner->plan(start, goal); // Plan from start to goal
```

The results from ```MPMapUtil``` are plotted in [output.jpg](https://github.com/sikang/motion_primitive_library/blob/master/data/output.jpg).
![Visualization](./data/output.jpg)


Note that only control space in acceleration is available at this time!


## Doxygen
For more details, please refer to https://sikang.github.io/motion_primitive_library/
