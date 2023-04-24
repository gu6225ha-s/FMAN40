#ifndef MESH_H_
#define MESH_H_

#include <Eigen/Dense>
#include <string>

namespace ppr {

class Mesh {
public:
  Mesh() {}
  Mesh(const std::vector<Eigen::Vector3d> &verts,
       const std::vector<std::tuple<int, int, int>> &triangles,
       const std::vector<Eigen::Vector2d> &texcoords,
       const std::string &image_name)
      : verts_(verts), triangles_(triangles), texcoords_(texcoords),
        image_name_(image_name) {}

  const std::vector<Eigen::Vector3d> &Verts() const { return verts_; }
  const std::vector<std::tuple<int, int, int>> &Triangles() const {
    return triangles_;
  }
  const std::vector<Eigen::Vector2d> &Texcoords() const { return texcoords_; }
  const std::string &ImageName() const { return image_name_; }

  void CalcBbox(Eigen::Vector3d &min, Eigen::Vector3d &max) const;

private:
  std::vector<Eigen::Vector3d> verts_;
  std::vector<std::tuple<int, int, int>> triangles_;
  std::vector<Eigen::Vector2d> texcoords_;
  std::string image_name_;
};

} // namespace ppr

#endif /* MESH_H_ */
