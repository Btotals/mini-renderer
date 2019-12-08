#ifndef __TRIANGLE_UTIL_H__
#define __TRIANGLE_UTIL_H__

#include "./geometry.h"
#include "./util.h"

template <class t> struct BoundaryBox {
  t top_left;
  t bottom_right;
};

template <class t>
BoundaryBox<t> get_boundary_box(const t& a, const t& b, const t& c) {
  BoundaryBox box;
  // 计算 aabb 包围盒
  box.top_left.x = min(a.x, min(b.x, c.x));
  box.bottom_right.x = max(a.x, max(b.x, c.x));
  box.top_left.y = min(a.y, min(b.y, c.y));
  box.bottom_right.x = max(a.y, max(b.y, c.y));

  return box;
}

template <class t> void get_barycentric(const t& a, const t& b, const t& c) {
  return t((a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3);
}

template <class t>
void get_barycentric_coordinate(const t& a,
                                const t& b,
                                const t& c,
                                const t& p) {
  t ap(p.x - a.x, p.y - a.y);
}

#endif
