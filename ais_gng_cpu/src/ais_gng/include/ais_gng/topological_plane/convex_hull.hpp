#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace fuzzrobo::topological_plane
{
struct plane_point
{
  double u = 0.0;
  double v = 0.0;
};

// 接平面投影の反時計回り凸包。重複点・一直線上の中間点・末尾の始点重複の除外。
inline std::vector<plane_point> convex_hull(std::vector<plane_point> points)
{
  std::sort(points.begin(), points.end(), [](const auto &a, const auto &b) {
    return a.u != b.u ? a.u < b.u : a.v < b.v;
  });
  points.erase(std::unique(points.begin(), points.end(), [](const auto &a, const auto &b) {
    return a.u == b.u && a.v == b.v;
  }), points.end());
  if (points.size() < 3U) return points;
  const auto cross = [](const plane_point &o, const plane_point &a, const plane_point &b) {
    return (a.u-o.u)*(b.v-o.v)-(a.v-o.v)*(b.u-o.u);
  };
  std::vector<plane_point> hull;
  hull.reserve(2U*points.size());
  const auto append = [&](const plane_point &point, std::size_t min_num) {
    while (hull.size() >= min_num && cross(hull[hull.size()-2U], hull.back(), point) <= 0.0)
      hull.pop_back();
    hull.push_back(point);
  };
  for (const auto &point : points) append(point, 2U);
  const auto min_upper_num = hull.size()+1U;
  for (std::size_t idx = points.size()-1U; idx > 0U; --idx) append(points[idx-1U], min_upper_num);
  hull.pop_back();
  return hull;
}

// 凸包の内側オフセットと点包含検査による外周帯。全点と全辺の総当たりなし。
inline std::vector<bool> convex_hull_boundary(
  const std::vector<plane_point> &points, double max_dist)
{
  const auto hull = convex_hull(points);
  std::vector<bool> is_boundary(points.size(), true);
  if (hull.size() < 3U) return is_boundary;
  const auto cross = [](const plane_point &a, const plane_point &b) {
    return a.u*b.v-a.v*b.u;
  };
  const auto delta = [](const plane_point &a, const plane_point &b) {
    return plane_point{a.u-b.u,a.v-b.v};
  };
  struct half_plane { plane_point point, direction; double angle; };
  std::vector<half_plane> lines;
  lines.reserve(hull.size());
  for (std::size_t idx=0; idx<hull.size(); ++idx) {
    auto direction=delta(hull[(idx+1)%hull.size()],hull[idx]);
    const double length=std::hypot(direction.u,direction.v);
    direction.u/=length; direction.v/=length;
    lines.push_back({{hull[idx].u-max_dist*direction.v,hull[idx].v+max_dist*direction.u},
      direction,std::atan2(direction.v,direction.u)});
  }
  std::sort(lines.begin(),lines.end(),[](const auto &a,const auto &b) { return a.angle<b.angle; });
  const auto intersection = [&](const half_plane &a,const half_plane &b) {
    const double ratio=cross(delta(b.point,a.point),b.direction)/cross(a.direction,b.direction);
    return plane_point{a.point.u+ratio*a.direction.u,a.point.v+ratio*a.direction.v};
  };
  const auto is_outside = [&](const half_plane &line,const plane_point &point) {
    return cross(line.direction,delta(point,line.point))<0;
  };
  // 角度順の半平面交差。各辺の追加・除去は各一回、消滅した辺の再走査なし。
  std::deque<half_plane> active;
  for (const auto &line:lines) {
    while (active.size()>1 && is_outside(line,intersection(active[active.size()-2],active.back())))
      active.pop_back();
    while (active.size()>1 && is_outside(line,intersection(active[0],active[1]))) active.pop_front();
    if (!active.empty() && cross(active.back().direction,line.direction)==0) {
      if (active.back().direction.u*line.direction.u+active.back().direction.v*line.direction.v<0)
        return is_boundary;
      if (!is_outside(line,active.back().point)) continue;
      active.pop_back();
    }
    active.push_back(line);
  }
  while (active.size()>2 && is_outside(active.front(),intersection(active[active.size()-2],active.back())))
    active.pop_back();
  while (active.size()>2 && is_outside(active.back(),intersection(active[0],active[1]))) active.pop_front();
  if (active.size()<3) return is_boundary;
  std::vector<plane_point> inner;
  for (std::size_t idx=0; idx<active.size(); ++idx) inner.push_back(intersection(active[idx],active[(idx+1)%active.size()]));
  const auto side = [&](const plane_point &a,const plane_point &b,const plane_point &point) {
    return cross(delta(b,a),delta(point,a));
  };
  // 内側凸多角形の扇形探索。外周帯との境界上も外周側の所属。
  for (std::size_t idx=0; idx<points.size(); ++idx) {
    const auto &point=points[idx];
    if (side(inner[0],inner[1],point)<=0 || side(inner[0],inner.back(),point)>=0) continue;
    std::size_t left=1,right=inner.size()-1;
    while (right-left>1) {
      const auto middle=(left+right)/2;
      if (side(inner[0],inner[middle],point)>=0) left=middle; else right=middle;
    }
    is_boundary[idx]=side(inner[left],inner[right],point)<=0;
  }
  return is_boundary;
}
}  // namespace fuzzrobo::topological_plane
