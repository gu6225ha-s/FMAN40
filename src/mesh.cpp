#include "mesh.h"

namespace ppr {

void Mesh::Bounds(Eigen::Vector3d &min, Eigen::Vector3d &max) const {
  assert(verts_.size() > 0);
  min = max = verts_[0];
  for (size_t i = 1; i < verts_.size(); i++) {
    min.x() = std::min(min.x(), verts_[i].x());
    min.y() = std::min(min.y(), verts_[i].y());
    min.z() = std::min(min.z(), verts_[i].z());
    max.x() = std::max(max.x(), verts_[i].x());
    max.y() = std::max(max.y(), verts_[i].y());
    max.z() = std::max(max.z(), verts_[i].z());
  }
}

} // namespace ppr
