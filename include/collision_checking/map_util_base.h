/**
 * @file map_util_base.h
 * @brief MapUtilBase classes
 */
#ifndef MAP_UTIL_BASE_H
#define MAP_UTIL_BASE_H

#include <stack>
#include <basic_type/data_type.h>

/**
 * @biref The base class is provided by considering both 2D and 3D maps
 * @param Ti is integer index of cell/voxel
 * @param Tf is float position of cell/voxel
 * @param Tmap is defined as a 1D array 
 */
template <class Ti, class Tf, class Tmap> class MapUtilBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  /**
   * @biref Simple constructor
   */
  MapUtilBase() {
    val_occ = 100;
    val_free = 0;
    val_unknown = -1;
  }

  ///Retrieve map as array
  Tmap getMap() { return map_; }
  ///Retrieve resolution 
  decimal_t getRes() { return res_; }
  ///Retrieve dimensions 
  Ti getDim() { return dim_; }
  ///Retrieve origin
  Tf getOrigin() { return origin_d_; }

  ///Check if the given cell is outside of the map in i-the dimension
  bool isOutSideXYZ(const Ti &n, int i) { return n(i) < 0 || n(i) >= dim_(i); }
  ///Check if the cell given index is free
  bool isFree(int idx) { return map_[idx] == val_free; }
  ///Check if the cell given index is unknown
  bool isUnKnown(int idx) { return map_[idx] == val_unknown; }
  ///Check if the cell given index is occupied
  bool isOccupied(int idx) { return map_[idx] > val_free; }
  ///Check if the given cell is free 
  bool isFree(const Ti &pn) {
    if (isOutSide(pn))
      return false;
    else
      return isFree(getIndex(pn));
  }
  ///Check if the given cell is occupied 
  bool isOccupied(const Ti &pn) {
    if (isOutSide(pn))
      return false;
    else
      return isOccupied(getIndex(pn));
  }
  ///Check if the given cell is unknown
  bool isUnKnown(const Ti &pn) {
    if (isOutSide(pn))
      return false;
    return map_[getIndex(pn)] == val_unknown;
  }
  ///Check if the ray from p1 to p2 is occluded
  bool isBlocked(const Tf &p1, const Tf &p2) {
    vec_E<Ti> pns = rayTrace(p1, p2);
    for (const auto &pn : pns) {
      if (isOccupied(pn))
        return true;
    }
    return false;
  }
  /**
   * @brief Set map 
   *
   * @param ori origin position
   * @param dim number of cells in each dimension
   * @param map array of status os cells
   * @param res map resolution
   */
  void setMap(const Tf& ori, const Ti& dim, const Tmap &map, decimal_t res) {
    map_ = map;
    dim_ = dim;
    origin_d_ = ori;
    res_ = res;
  }

  ///Print basic information about the util
  virtual void info() {};
  ///Dilate obstacles
  virtual void dilating() {};
  ///Float position to discrete cell
  virtual Ti floatToInt(const Tf &pt) = 0;
  ///Discrete cell to float position
  virtual Tf intToFloat(const Ti &pp) = 0;
  ///Check if the cell is outside
  virtual bool isOutSide(const Ti &pn) = 0;
  ///Return occupied cells as vector of points
  virtual vec_E<Tf> getCloud() = 0;
  ///Retrieve subindex of a cell
  virtual int getIndex(const Ti &pn) = 0;
  ///Raytrace from pt1 to pt2
  virtual vec_E<Ti> rayTrace(const Tf &pt1, const Tf &pt2) = 0;
  ///Set dilate information, assume robot is cylinder with radius r and height h
  virtual void dilate(decimal_t r, decimal_t h) = 0;

protected:
  decimal_t res_;
  Tf origin_d_;
  Ti dim_;

  Tmap map_;

  ///Pre-computed vector of neighboring cells for dilating
  vec_E<Ti> dilate_neighbor_;
  ///Assume occupied cell has value 100
  char val_occ = 100;
  ///Assume free cell has value 0
  char val_free = 0;
  ///Assume unknown cell has value -1
  char val_unknown = -1;
};

#endif
