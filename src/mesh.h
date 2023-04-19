#ifndef MESH_H_
#define MESH_H_

#include <Eigen/Dense>

namespace ppr {

class Mesh {
public:
  Mesh() {}
  Mesh(const std::vector<Eigen::Vector3d> &verts,
       const std::vector<std::tuple<int, int, int>> &triangles)
      : verts_(verts), triangles_(triangles) {}

  void AddTriangles(const std::vector<Eigen::Vector3d> &verts,
                    const std::vector<std::tuple<int, int, int>> &triangles);

private:
  std::vector<Eigen::Vector3d> verts_;
  std::vector<std::tuple<int, int, int>> triangles_;
};

} // namespace ppr

#endif /* MESH_H_ */
