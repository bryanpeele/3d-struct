#pragma once

#include "graph.h"

namespace magnus {
namespace geom {

struct StructifyConfig {
  double struct_radius;
  double vertex_radius;
  int num_polygon_sides;
};

class Structify {
 public:
  Structify(const StructifyConfig &config, const std::vector<Edge3d> &edges);

  const std::vector<Point3d> &Points() const;

  // const std::vector<Triangle> &GetTriangles() const;

  const std::vector<Eigen::Vector3i> &TriangleIndices() const;

 private:
  void Build(const std::vector<Edge3d> &edges);

  void AddStruct(const Edge3d &edge);

  const StructifyConfig config_;

  std::vector<Point3d> points_;

  std::vector<Eigen::Vector3i> triangle_indices_;
};

}  // namespace geom
}  // namespace magnus
