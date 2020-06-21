#include "tesselate.h"

#include <iostream>

namespace magnus {
namespace geom {

Point2d Reflect(const LineSegment2d &line, const Point2d &point) {
  std::cout << "l_p0: " << line.p0().x() << ", " << line.p0().y() << std::endl;
  std::cout << "l_p1: " << line.p1().x() << ", " << line.p1().y() << std::endl;
  std::cout << "p: " << point.x() << ", " << point.y() << std::endl;

  double den = line.p0().x() * line.p1().y() - line.p1().x() * line.p0().y();
  bool straight = std::abs(den) < 1e-9;  // TODO(bpeele) make sure eps makes sense...

  if (straight) {
    const Point2d p = line.p0();
    std::cout << "p_copy: " << p.x() << ", " << p.y() << std::endl;

    den = std::sqrt((line.p0().x() - line.p1().x()) * (line.p0().x() - line.p1().x()) +
                    (line.p0().y() - line.p1().y()) * (line.p0().y() - line.p1().y()));

    std::cout << "den: " << den << std::endl;

    const Point2d d = Point2d({(line.p1().x() - line.p0().x()) / den,
                               (line.p1().y() - line.p0().y()) / den});

    double factor = 2.0 * ((point.x() - p.x()) * d.x() + (point.y() - p.y()) * d.y());

    std::cout << "p': " << 2.0 * p.x() + factor * d.x() - point.x() << ", "
                        << 2.0 * p.y() + factor * d.y() - point.y() << std::endl;
    std::cout << "=========================================" << std::endl;
    return Point2d({2.0 * p.x() + factor * d.x() - point.x(),
                    2.0 * p.y() + factor * d.y() - point.y()});
  } else {
    const double s1 = (1.0 + line.p0().x() * line.p0().x() + line.p0().y() * line.p0().y()) / 2.0;
    const double s2 = (1.0 + line.p1().x() * line.p1().x() + line.p1().y() * line.p1().y()) / 2.0;
    const Point2d C = Point2d({(s1 * line.p1().y() - s2 * line.p0().y()) / den,
                               (line.p0().x() * s2 - line.p1().x() * s1) / den});
    const double r = std::sqrt(C.x() * C.x() + C.y() * C.y() - 1.0);

    // Reflect method
    const double factor = r * r /
        ((point.x() - C.x()) * (point.x() - C.x()) + (point.y() - C.y()) * (point.y() - C.y()));

    std::cout << "p'': " << C.x() + factor * (point.x() - C.x()) << ", "
                         << C.y() + factor * (point.y() - C.y()) << std::endl;
    std::cout << "=========================================" << std::endl;

    return Point2d({C.x() + factor * (point.x() - C.x()),
                    C.y() + factor * (point.y() - C.y())});
  }
}

Tessellate::Tessellate(int n, int k, int level) : n_(n),
                                                  k_(k),
                                                  level_(level),
                                                  num_inner_polygons_(GetNumInnerPolygons(level)) {
  GeneratePolygons();
}

Graph3d Tessellate::Graph() const {
  const auto lines = Lines();
  auto graph = Graph3d();
  std::vector<PointHandle> handles;
  for (const auto &point : lines.first) handles.emplace_back(graph.AddPoint(Point3d(point)));
  for (const auto &line : lines.second) graph.AddEdge(handles[line(0)], handles[line(1)]);
  return graph;
}

int GetMatchOrAdd(std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &new_point) {
  for (size_t i = 0; i < points.size(); i++) {
    if ((points[i] - new_point).norm() < 1e-9) return i;
  }

  points.emplace_back(new_point);
  return points.size() - 1;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector2i>> Tessellate::Lines() const {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector2i> lines;
  for (const auto &polygon : polygons_) {
    for (size_t i = 0; i < polygon.size(); i++) {
      const int p0 = GetMatchOrAdd(points, {polygon[i].x(), polygon[i].y(), 0.0});
      int next_i = (i + 1) % polygon.size();
      const int p1 = GetMatchOrAdd(points, {polygon[next_i].x(), polygon[next_i].y(), 0.0});
      lines.emplace_back(p0, p1);
    }
  }
  std::cout << "num points: " << points.size() << std::endl;
  return {points, lines};
}

int Tessellate::GetNumInnerPolygons(int num_levels) const {
  int total_num_polygons = 1;
  int num_inner_polygons = 0;

  int a = n_ * (k_ - 3);
  int b = n_;

  int next_a, next_b;
  for (int level = 1; level <= num_levels; level++) {
    num_inner_polygons = total_num_polygons;
    if (k_ == 3) {
      next_a = a + b;
      next_b = (n_ - 6) * a + (n_ - 5) * b;
    } else {
      next_a =
          ((n_ - 2) * (k_ - 3) - 1) * a +
          ((n_ - 3) * (k_ - 3) - 1) * b;
      next_b = (n_ - 2) * a + (n_ - 3) * b;
    }

    total_num_polygons += a + b;

    a = next_a;
    b = next_b;
  }

  std::cout << "Num inner polys: " << num_inner_polygons << std::endl;

  return num_inner_polygons;
}

void Tessellate::GeneratePolygons() {
  polygons_.emplace_back(ConstructCenterPolygon());
  rules_.emplace_back(0);
  std::cout << "Created initial polygon!" << std::endl;
  int j = 1;
  for (int i = 0; i < num_inner_polygons_; i++) j = ApplyRule(i, j);
}

Tessellate::Polygon2d Tessellate::ConstructCenterPolygon() {
  const double theta_a = M_PI / n_;
  const double theta_b = M_PI / k_;
  const double theta_c = M_PI / 2.0;

  const double sin_theta_a = std::sin(theta_a);
  const double sin_theta_b = std::sin(theta_b);
  const double s = std::sin(theta_c - theta_b - theta_a) /
      std::sqrt(1.0 - sin_theta_b * sin_theta_b - sin_theta_a * sin_theta_a);

  Polygon2d polygon;

  for (int i = 0; i < n_; i++) {
    polygon.emplace_back(Eigen::Vector2d(s * std::cos((3 + 2 * i) * theta_a),
                                         s * std::sin((3 + 2 * i) * theta_a)));
  }

  return polygon;
}

Tessellate::Polygon2d Tessellate::ConstructNextPolygon(const Polygon2d polygon, int side) {
  if (static_cast<int>(polygon.size()) != n_) std::cout << "shit" << std::endl;
  Point2d start = polygon[side];
  Point2d end = polygon[(side + 1) % n_];
  std::cout << "start: " << start.x() << ", " << start.y() << std::endl;
  std::cout << "end: " << end.x() << ", " << end.y() << std::endl;
  Polygon2d next_polygon;
  for (int i = 0; i < n_; i++) next_polygon.emplace_back(Eigen::Vector2d::Zero());
  for (int i = 0; i < n_; ++i) {
    // Reflect P[i] thru C to get Q[j]}
    int j = (n_ + side - i + 1) % n_;
    next_polygon[j] = Reflect({start, end}, polygon[i]);
  }

  return next_polygon;
}

int Tessellate::ApplyRule(int i, int j) {
  int rule = rules_[i];
  const bool special = (rule == 1);
  if (special) rule = 2;
  const int start = (rule == 4) ? 3 : 2;
  const int num = (k_ == 3 && rule != 0) ? n_ - rule - 1 : n_ - rule;
  for (int s = start; s < start + num; s++) {
    if (j != static_cast<int>(polygons_.size())) std::cout << "fuck" << std::endl;
    polygons_.emplace_back(ConstructNextPolygon(polygons_[i], s % n_));

    if (j != static_cast<int>(rules_.size())) std::cout << "fuck" << std::endl;
    rules_.push_back((k_ == 3 && s == start && rule != 0) ? 4 : 3);

    j++;
    int m;
    if (special) {
      m = 2;
    } else if (s == 2 && rule != 0) {
      m = 1;
    } else {
      m = 0;
    }
    for (; m < k_ - 3; m++) {
      // Create a polygon adjacent to P[j-1]

      if (j != static_cast<int>(polygons_.size())) std::cout << "fuck" << std::endl;
      polygons_.emplace_back(ConstructNextPolygon(polygons_[j - 1], 1));


      if (j != static_cast<int>(rules_.size())) std::cout << "fuck" << std::endl;
      rules_.push_back(rules_[j] = (n_ == 3 && m == k_ - 4) ? 1 : 2);

      j++;
    }
  }

  return j;
}

}  // namespace geom
}  // namespace magnus
