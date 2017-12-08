std_msgs/Header header
VoxelMapInfo info

geometry_msgs/Point origin
geometry_msgs/Point dim

# Subscript (xi, yi, zi) indexes probability[xi + num_x * yi + zi * num_x * num_y]
# represents (origin.x + xi * resolution, origin.y + yi * resolution, origin.z + zi * resolution)
int8[] data
