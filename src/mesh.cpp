#include "mesh.h"

namespace ppr {

void Mesh::AddTriangles(
    const std::vector<Eigen::Vector3d> &verts,
    const std::vector<std::tuple<int, int, int>> &triangles) {
  int n = verts_.size();
  verts_.insert(verts_.end(), verts.begin(), verts.end());
  for (size_t i = 0; i < triangles.size(); i++) {
    const std::tuple<int, int, int> &t = triangles[i];
    triangles_.emplace_back(std::get<0>(t) + n, std::get<1>(t) + n,
                            std::get<2>(t) + n);
  }
}

} // namespace ppr
