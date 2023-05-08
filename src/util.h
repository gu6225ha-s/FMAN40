/// \file util.h
/// \brief Utility functions.
#ifndef UTIL_H_
#define UTIL_H_

#include "image.h"
#include "mesh.h"
#include "polygon.h"
#include <Eigen/Dense>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace ppr {

/// \brief Split string using a delimiter.
/// \param str the string to split
/// \param delim delimiter
/// \return The resulting vector of strings.
std::vector<std::string> SplitString(const std::string &str,
                                     const std::string &delim);

/// \brief Read polygon file.
/// \param path path to polygon file
/// \return A vector of <image name, 2d polygon> pairs.
std::vector<std::pair<std::string, ppr::Polygon2d>>
ReadPolygons(const std::string &path);

/// \brief Write glTF file.
///
/// See https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html
/// \param meshes vector of triangle meshes
/// \param path path to glTF file
void WriteGltf(const std::vector<Mesh> &meshes,
               const std::filesystem::path &path);

/// \brief Plane estimation using image point correspondences.
class PlaneEstimator {
public:
  /// \brief Create new plane estimator.
  PlaneEstimator() {}

  /// \brief Add a point correspondence.
  /// \param x normalized image point in the first image
  /// \param y normalized image point in the second image
  /// \param R rotation matrix of the second camera
  /// \param t translation vector of the second camera
  void AddCorrespondence(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
                         const Eigen::Matrix3d &R, const Eigen::Vector3d &t);

  /// \brief Solve for the plane normal.
  ///
  /// At least three correspondences should first be added using
  /// PlaneEstimator#AddCorrespondence.
  /// \return The plane normal vector.
  Eigen::Vector3d Solve() const;

private:
  std::vector<Eigen::Vector3d> x_, y_, Rx_, t_;
};

/// \brief RGB image cache.
class ImageCache {
public:
  /// \brief Create new image cache.
  /// \param path image directory
  ImageCache(const std::string &path) : path_(path) {}

  /// \brief Get image from the cache, reading it from disk if needed.
  /// \param name filename
  /// \return The RGB image.
  const RgbImage &operator[](const std::string &name);

private:
  std::filesystem::path path_;             // Image directory
  std::map<std::string, RgbImage> images_; // Images
};

} // namespace ppr

#endif /* UTIL_H_ */
