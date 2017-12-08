MRSL Decomputil Library
======================
Fast convex decomposition utils. In the basic pipeline, it implements ellipsoid based regional inflation to model free space from a given path inside a point cloud.
Detials of the algorithm is proposed in "S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor and V. Kumar. Planning Dynamically Feasible Trajectories for Quadrotors using Safe Flight Corridors in 3-D Complex Environments. ICRA 2017".

## Compilation
A) Simple cmake
```sh
$ mkdir build && cd build && cmake .. && make
```

B) Using CATKIN
```sh
$ cd mv decomp_util ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Example
The output from `EllipseDecomp` or `IterativeDecomp` can be visualized with ROS Rviz:

<img src="./samples/sample1.png" height="256"> <img src="./samples/sample2.png" height="256">


## Doxygen
For more details, please refer to https://sikang.github.io/DecompUtil/index.html

## ROS
The ROS wrapper for easier use of this package can be found in [`DecompROS`](https://github.com/sikang/DecompROS.git).
