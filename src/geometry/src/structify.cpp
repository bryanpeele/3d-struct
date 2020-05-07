#include "structify.h"

#include <cmath>
#include <iostream>

namespace magnus {
namespace geom {

Structify::Structify(const StructifyConfig &config, const std::vector<Edge3d> &edges)
    : config_(config) {
  Build(edges);
}

const std::vector<Point3d> &Structify::Points() const { return points_; }

const std::vector<Eigen::Vector3i> &Structify::TriangleIndices() const {
  return triangle_indices_;
}

void Structify::Build(const std::vector<Edge3d> &edges) {
  // Add structs.
  for (const auto &edge : edges) {
    AddStruct(edge);
  }

  // Add hubs...
}

void Structify::AddStruct(const Edge3d &edge) {

  // TODO(bpeele) Validate that edge length > 2 * vertex_radius
  const Eigen::Vector3d edge_prime = edge.vector().normalized();

  const Vector3d center0 = edge.p0().coordinate() + config_.vertex_radius * edge_prime;
  const Vector3d center1 = edge.p1().coordinate() - config_.vertex_radius * edge_prime;

  const double angle = (2.0 * M_PI)/ static_cast<double>(config_.num_polygon_sides);
  const double half_angle = 0.5 * angle;

  std::vector<Vector3d> spokes;
  const Vector3d first_spoke =
      config_.struct_radius * (edge_prime.cross(Vector3d::UnitZ())).normalized();

  std::vector<int> face_0;
  std::vector<int> face_1;
  int index = points_.size();

  for (int i = 0; i < 2 * config_.num_polygon_sides; i++) {
    const double this_angle = static_cast<double>(i) * half_angle;
    const Eigen::AngleAxisd rotation(this_angle, edge_prime);
    if (i%2 == 0) {
      points_.emplace_back(center0 + (rotation * first_spoke));
      face_0.push_back(index++);
    } else {
      points_.emplace_back(center1 + (rotation * first_spoke));
      face_1.push_back(index++);
    }
  }

  for (size_t j = 0; j < face_0.size(); j++) {
    const int next_j = (j < face_0.size() - 1) ? j + 1 : 0;
    // triangle_indices_.emplace_back(face_0[j], face_1[j], face_0[next_j]);
    triangle_indices_.emplace_back(face_0[j], face_0[next_j], face_1[j]);
    // triangle_indices_.emplace_back(face_0[next_j], face_1[j], face_1[next_j]);
    triangle_indices_.emplace_back(face_0[next_j], face_1[next_j], face_1[j]);
  }
}

}  // namespace geom
}  // namespace magnus
