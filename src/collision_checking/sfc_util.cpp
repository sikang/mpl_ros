#include <collision_checking/sfc_util.h>
// Optimistic: 

SFCUtil::SFCUtil() {}

SFCUtil::SFCUtil(const Polyhedra& polys) : polys_(polys) {
}

Polyhedra SFCUtil::sfc() {
  return polys_;
}

bool SFCUtil::isFree(const Primitive& pr, decimal_t dt) {
  return insideSFC(pr, polys_, 0, dt);
}

bool SFCUtil::isFree(const Vec3f& pt) {
  // Check if a point is inside free space
  // note that it can be inside unknown space as it is inside the free space of at least one polyhedron
  for(const auto& Vs: polys_) {
    if(insidePolyhedron(pt, Vs)) 
      return true;
  }
  return false;
}

bool SFCUtil::insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs) {
  for (const auto& v : Vs) {
    Vec3f a = v.n;
    decimal_t b = v.p.dot(a);
    if (a.dot(pt) - b > 1e-3)
      return false;
  }
  return true;
}


bool SFCUtil::insideSFC(const Primitive& pr, const Polyhedra& polys, 
    decimal_t t1, decimal_t t2) {
  if(polys.empty())
    return false;
  Waypoint p0 = pr.evaluate(t1);

  int id = -1;
  for(int i = 0; i < polys.size(); i++) {
    if(insidePolyhedron(p0.pos, polys[i]))  {
      id = i;
      break;
    }
  }

  if(id < 0)
    return false;

  Polyhedra polys2 = polys;
  polys2.erase(polys2.begin() + id);

  //printf("poly size: %zu, poly2 size: %zu\n", polys.size(), polys2.size());
  const vec_E<Vec6f> cs = pr.coeffs();
  for(const auto& v: polys[id]){
    Vec3f n = v.n;
    decimal_t a1 = n(0);
    decimal_t a2 = n(1);
    decimal_t a3 = n(2);

    decimal_t a = (a1*cs[0](1)+a2*cs[1](1)+a3*cs[2](1))/24;
    decimal_t b = (a1*cs[0](2)+a2*cs[1](2)+a3*cs[2](2))/6;
    decimal_t c = (a1*cs[0](3)+a2*cs[1](3)+a3*cs[2](3))/2;
    decimal_t d = (a1*cs[0](4)+a2*cs[1](4)+a3*cs[2](4));
    decimal_t e = a1*cs[0](5)+a2*cs[1](5)+a3*cs[2](5)-n.dot(v.p);

    std::vector<decimal_t> ts = solve(a, b, c, d, e);
    //printf("a, b, c, d, e: %f, %f, %f, %f, %f\n", a, b, c, d, e);
    for(const auto& it: ts) 
      if(it >= t1 && it <= t2)
      {
	Waypoint pt = pr.evaluate(it+ 1e-1);
	if(!insidePolyhedron(pt.pos, polys[id]))
	  return insideSFC(pr, polys2, it, t2);
      }
  }
  Waypoint p1 = pr.evaluate(t2);
  return insidePolyhedron(p1.pos, polys[id]);
}

