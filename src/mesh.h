#ifndef MESH_H_
#define MESH_H_

#include "image.h"
#include <Eigen/Dense>
#include <string>

namespace ppr {

class Mesh {
public:
  Mesh() {}
  Mesh(const std::vector<Eigen::Vector3d> &verts,
       const std::vector<std::tuple<int, int, int>> &triangles,
       const std::vector<Eigen::Vector2d> &texcoords, RgbImage &&image)
      : verts_(verts), triangles_(triangles), texcoords_(texcoords),
        image_(std::move(image)) {}

  const std::vector<Eigen::Vector3d> &Verts() const { return verts_; }
  const std::vector<std::tuple<int, int, int>> &Triangles() const {
    return triangles_;
  }
  const std::vector<Eigen::Vector2d> &Texcoords() const { return texcoords_; }
  const RgbImage &Image() const { return image_; }

  void Bounds(Eigen::Vector3d &min, Eigen::Vector3d &max) const;

private:
  std::vector<Eigen::Vector3d> verts_;               // Vertices
  std::vector<std::tuple<int, int, int>> triangles_; // Triangles
  std::vector<Eigen::Vector2d> texcoords_;           // Texture coordinates
  RgbImage image_;                                   // Texture image
};

} // namespace ppr

#endif /* MESH_H_ */
