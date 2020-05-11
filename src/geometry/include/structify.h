#pragma once

#include <map>
#include <optional>

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
  Structify(const StructifyConfig &config, const Graph3d &graph);

  const std::vector<Point3d> &Points() const;
  const std::vector<Eigen::Vector3i> &TriangleIndices() const;

 private:
  struct Hub {
    std::vector<int> point_ids;
    std::optional<int> center_id;
    std::vector<Eigen::Vector3d> normals;
  };

  void Build();

  void InitializeHubs();

  void AddStruct(const EdgeHandle &edge_handle);

  void AddHubs();

  bool IsParallelToAnySpoke(const Hub &hub, const Eigen::Vector3d &vector) const;

  bool IsNearlyEqualToAnySpoke(const Hub &hub, const Eigen::Vector3d &vector) const;

  const StructifyConfig config_;

  const Graph3d graph_;

  std::vector<Hub> hubs_;
  std::map<PointHandle, int> hub_map_;

  std::vector<Point3d> points_;
  std::vector<Eigen::Vector3i> triangle_indices_;
};

}  // namespace geom
}  // namespace magnus
