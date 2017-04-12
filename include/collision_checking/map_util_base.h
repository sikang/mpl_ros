#ifndef MAP_UTIL_BASE_H
#define MAP_UTIL_BASE_H

#include <stack>
#include <basic_type/data_type.h>

template <class Ti, class Tf, class Tmap> class MapUtilBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MapUtilBase() {
    val_occ = 100;
    val_free = 0;
    val_unknown = -1;
  }

  Tmap getMap() { return map_; }
  decimal_t getRes() { return res_; }
  Ti getDim() { return dim_; }
  Tf getOrigin() { return origin_d_; }

  bool isOutSideXYZ(const Ti &n, int i) { return n(i) < 0 || n(i) >= dim_(i); }
  bool isFree(int idx) { return map_[idx] == val_free; }
  bool isUnKnown(int idx) { return map_[idx] == val_unknown; }
  bool isOccupied(int idx) { return map_[idx] > val_free; }
  bool isFree(const Ti &pn) {
    if (isOutSide(pn))
      return false;
    else
      return isFree(getIndex(pn));
    // return !isOccupied(getIndex(pn));
  }
  bool isOccupied(const Ti &pn) {
    if (isOutSide(pn))
      return false;
    else
      return isOccupied(getIndex(pn));
  }
  bool isUnKnown(const Ti &pn) {
    if (isOutSide(pn))
      return false;
    return map_[getIndex(pn)] == val_unknown;
  }

  bool isBlocked(const Tf &p1, const Tf &p2) {
    vec_E<Ti> pns = rayTrace(p1, p2);
    for (const auto &pn : pns) {
      if (isOccupied(pn))
        return true;
    }
    return false;
  }

  virtual void info() {};
  virtual void dilating() {};
  virtual Ti floatToInt(const Tf &pt) = 0;
  virtual Tf intToFloat(const Ti &pp) = 0;
  virtual bool isOutSide(const Ti &pn) = 0;
  virtual vec_E<Tf> getCloud() = 0;
  virtual void setMap(const Tf& ori, const Ti& dim, const Tmap &map, decimal_t res) = 0;
  virtual int getIndex(const Ti &pn) = 0;
  virtual vec_E<Ti> rayTrace(const Tf &pt1, const Tf &pt2) = 0;
  virtual void dilate(decimal_t r, decimal_t h) = 0;

protected:
  decimal_t res_;
  Tf origin_d_;
  Ti dim_;

  Tmap map_;

  vec_E<Ti> dilate_neighbor_;
  char val_occ = 100;
  char val_free = 0;
  char val_unknown = -1;
};

#endif
