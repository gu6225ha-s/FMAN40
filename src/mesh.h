#ifndef MESH_H_
#define MESH_H_

#include "image.h"
#include <Eigen/Dense>
#include <string>

namespace ppr {

/// \brief Textured triangle mesh.
class Mesh {
public:
  /// \brief Create new, empty mesh.
  Mesh() {}

  /// \brief Create new mesh from vectors of vertices, triangles, texture
  /// coordinates and a texture image.
  /// \param verts vector of vertices
  /// \param triangles vector of triangles
  /// \param texcoords vector of texture coordinates
  /// \param image texture image
  Mesh(const std::vector<Eigen::Vector3d> &verts,
       const std::vector<std::tuple<int, int, int>> &triangles,
       const std::vector<Eigen::Vector2d> &texcoords, RgbImage &&image)
      : verts_(verts), triangles_(triangles), texcoords_(texcoords),
        image_(std::move(image)) {}

  /// \brief Get the vertices of the mesh.
  /// \return The vector of vertices.
  const std::vector<Eigen::Vector3d> &Verts() const { return verts_; }

  /// \brief Get the triangles of the mesh.
  /// \return The vector of triangles.
  const std::vector<std::tuple<int, int, int>> &Triangles() const {
    return triangles_;
  }

  /// \brief Get the texture coordinates of the mesh.
  /// \return The vector of texture coordinates.
  const std::vector<Eigen::Vector2d> &Texcoords() const { return texcoords_; }

  /// \brief Get the texture image of the mesh.
  /// \return The texture image.
  const RgbImage &Image() const { return image_; }

  /// \brief Compute the bounds of the mesh.
  /// \param[out] min the minimum coordinates in xyz
  /// \param[out] max the maximum coordinates in xyz
  void Bounds(Eigen::Vector3d &min, Eigen::Vector3d &max) const;

private:
  std::vector<Eigen::Vector3d> verts_;               // Vertices
  std::vector<std::tuple<int, int, int>> triangles_; // Triangles
  std::vector<Eigen::Vector2d> texcoords_;           // Texture coordinates
  RgbImage image_;                                   // Texture image
};

} // namespace ppr

#endif /* MESH_H_ */
