#include "structify.h"

#include <cmath>
#include <iostream>

#include "Open3D/Open3D.h"

#include "math_util.h"

namespace magnus {
namespace geom {

Structify::Structify(const StructifyConfig &config, const Graph3d &graph)
    : config_(config), graph_(graph) {
  Build();
}

const std::vector<Point3d> &Structify::Points() const { return points_; }

const std::vector<Eigen::Vector3i> &Structify::TriangleIndices() const {
  return triangle_indices_;
}

void Structify::Build() {
  InitializeHubs();

  // Add structs.
  for (const auto &edge_handle : graph_.edge_handles()) { AddStruct(edge_handle); }

  // Add hubs.
  AddHubs();
}

void Structify::InitializeHubs() {
  hubs_.assign(graph_.num_points(), Hub());

  for (size_t hub_index = 0; hub_index < graph_.point_handles().size(); hub_index++) {
    hub_map_.insert({graph_.point_handles()[hub_index], hub_index});
  }
}

void Structify::AddStruct(const EdgeHandle &edge_handle) {
  // TODO(bpeele) Validate that edge length > 2 * vertex_radius
  const Eigen::Vector3d edge_prime = graph_.edge_vector(edge_handle).normalized();
  const auto endpoints = graph_.edge(edge_handle);


  const Vector3d center0 = endpoints.first.coordinate() + config_.vertex_radius * edge_prime;
  const Vector3d center1 = endpoints.second.coordinate() - config_.vertex_radius * edge_prime;

  const double angle = (2.0 * M_PI)/ static_cast<double>(config_.num_polygon_sides);
  const double half_angle = 0.5 * angle;

  std::vector<Vector3d> spokes;

  const Vector3d arbitrary =
      IsParallel(edge_prime, Vector3d::UnitZ()) ? Vector3d::UnitX() : Vector3d::UnitZ();

  const Vector3d first_spoke =
      config_.struct_radius * (edge_prime.cross(arbitrary)).normalized();

  std::vector<int> face_0;
  std::vector<int> face_1;

  for (int i = 0; i < 2 * config_.num_polygon_sides; i++) {
    const double this_angle = static_cast<double>(i) * half_angle;
    const Eigen::AngleAxisd rotation(this_angle, edge_prime);
    if (i%2 == 0) {
      points_.emplace_back(center0 + (rotation * first_spoke));
      face_0.push_back(points_.size() - 1);
    } else {
      points_.emplace_back(center1 + (rotation * first_spoke));
      face_1.push_back(points_.size() - 1);
    }
  }

  for (size_t j = 0; j < face_0.size(); j++) {
    const int next_j = (j < face_0.size() - 1) ? j + 1 : 0;
    triangle_indices_.push_back({face_0[j], face_0[next_j], face_1[j]});
    triangle_indices_.push_back({face_0[next_j], face_1[next_j], face_1[j]});
  }

  // Get hubs
  auto &hub0 = hubs_[hub_map_.at(graph_.point_handles(edge_handle).first)];
  auto &hub1 = hubs_[hub_map_.at(graph_.point_handles(edge_handle).second)];

  for (const auto &i : face_0) hub0.point_ids.push_back(i);
  for (const auto &i : face_1) hub1.point_ids.push_back(i);

  // Add centers to hubs (as needed)...
  if (!hub0.center_id) {
    points_.emplace_back(endpoints.first.coordinate());
    hub0.point_ids.push_back(points_.size() - 1);
    hub0.center_id = points_.size() - 1;
  }
  if (!hub1.center_id) {
    points_.emplace_back(endpoints.second.coordinate());
    hub1.point_ids.push_back(points_.size() - 1);
    hub1.center_id = points_.size() - 1;
  }

  hub0.normals.push_back(edge_prime);
  hub1.normals.push_back(-edge_prime);
}

void Structify::AddHubs() {
  for (const auto &hub : hubs_) {
    std::vector<Eigen::Vector3d> points;
    for (const auto &id : hub.point_ids) points.push_back(points_[id].coordinate());
    const auto cloud = open3d::geometry::PointCloud(points);
    cloud.ComputeConvexHull();
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    std::vector<size_t> stuff;
    std::tie(mesh, stuff) = cloud.ComputeConvexHull();
    mesh->OrientTriangles();
    mesh->ComputeTriangleNormals();
    mesh->ComputeVertexNormals();

    // Test for inversion?
    const auto center = mesh->GetCenter();
    const auto first_triangle = mesh->triangles_.front();
    const auto first_point = mesh->vertices_[first_triangle[0]];
    const auto center_to_first_point = first_point - center;
    const auto tri_normal = mesh->triangle_normals_.front();
    const bool need_to_invert = center_to_first_point.dot(tri_normal) < 0.0;


    std::cout << mesh->triangle_normals_[0].transpose() << std::endl;
    if (need_to_invert) {
      for (auto &normal : mesh->triangle_normals_) normal *= -1.0;
      for (auto &normal : mesh->vertex_normals_) normal *= -1.0;
      for (auto &triangle : mesh->triangles_) std::swap(triangle(0), triangle(1));
    }

    const int index_offset = points_.size();

    for (const auto &vertex : mesh->vertices_) points_.emplace_back(vertex);

    for (size_t i = 0; i < mesh->triangles_.size(); i++) {
      const auto &tri = mesh->triangles_[i];
      if (!IsNearlyEqualToAnySpoke(hub, mesh->triangle_normals_[i])) {
        triangle_indices_.emplace_back(index_offset + tri(0),
                                       index_offset + tri(1),
                                       index_offset + tri(2));
      }
    }
  }
}

bool Structify::IsParallelToAnySpoke(const Structify::Hub &hub, const Vector3d &vector) const {
  for (const auto &spoke : hub.normals) if (IsParallel(spoke, vector)) return true;
  return false;
}

bool Structify::IsNearlyEqualToAnySpoke(const Hub &hub, const Eigen::Vector3d &vector) const {
  for (const auto &spoke : hub.normals) if (IsNearlyEqual(spoke, vector)) return true;
  return false;
}

}  // namespace geom
}  // namespace magnus
