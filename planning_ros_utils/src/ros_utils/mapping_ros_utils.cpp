#include <ros_utils/mapping_ros_utils.h>

void setMap(MPL::VoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.info.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}


void getMap(MPL::VoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  decimal_t res = map_util->getRes();

  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.info.resolution = res;

  map.data = map_util->getMap();
}


void setMap(MPL::SubVoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.info.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

void getMap(MPL::SubVoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map) {
  Vec3i dim1 = map_util->getDimLow();
  Vec3i dim2 = map_util->getDimUp();
  Vec3i dim = dim2 - dim1;
  decimal_t res = map_util->getRes();

  Vec3f ori = map_util->intToFloat(dim1);
  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.info.resolution = res;

  map.data = map_util->getSubMap();
}

/*
void setMap(JPS::VoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.info.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}


void getMap(JPS::VoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  decimal_t res = map_util->getRes();

  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.info.resolution = res;

  map.data = map_util->getMap();
}

void setMap(JPS::SubVoxelMapUtil* map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.info.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

void getMap(JPS::SubVoxelMapUtil* map_util, planning_ros_msgs::VoxelMap& map) {
  Vec3i dim1 = map_util->getDimLow();
  Vec3i dim2 = map_util->getDimUp();
  Vec3i dim = dim2 - dim1;
  decimal_t res = map_util->getRes();

  Vec3f ori = map_util->intToFloat(dim1);
  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.info.resolution = res;

  map.data = map_util->getSubMap();
}


void toMPLMapUtil(JPS::VoxelMapUtil* jps_map_util, MPL::VoxelMapUtil* mpl_map_util) {
  mpl_map_util->setMap(jps_map_util->getOrigin(), jps_map_util->getDim(), jps_map_util->getMap(), jps_map_util->getRes());
}

void toJPSMapUtil(MPL::VoxelMapUtil* mpl_map_util, JPS::VoxelMapUtil* jps_map_util) {
  jps_map_util->setMap(mpl_map_util->getOrigin(), mpl_map_util->getDim(), mpl_map_util->getMap(), mpl_map_util->getRes());
}
*/

