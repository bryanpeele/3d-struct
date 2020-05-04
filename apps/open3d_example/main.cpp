#include "Open3D/Open3D.h"

int main() {
  auto cube_ptr = open3d::geometry::TriangleMesh::CreateBox();
  cube_ptr->ComputeVertexNormals();
  open3d::visualization::DrawGeometries({cube_ptr});
  return 0;
}
