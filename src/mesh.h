#ifndef MESH_H_
#define MESH_H_

#include <Eigen/Dense>
#include <string>

namespace ppr {

class Mesh {
public:
  Mesh() {}
  Mesh(const std::vector<Eigen::Vector3d> &verts,
       const std::vector<std::tuple<int, int, int>> &triangles)
      : verts_(verts), triangles_(triangles) {}

  void AddTriangles(const std::vector<Eigen::Vector3d> &verts,
                    const std::vector<std::tuple<int, int, int>> &triangles);
  void CalcBbox(Eigen::Vector3d &min, Eigen::Vector3d &max) const;
  void WriteGltf(const std::string &path) const;

private:
  std::vector<Eigen::Vector3d> verts_;
  std::vector<std::tuple<int, int, int>> triangles_;

  void WriteGltfBuffer(const std::string &path, size_t &vert_size,
                       size_t &tri_size) const;
};

} // namespace ppr

#endif /* MESH_H_ */
