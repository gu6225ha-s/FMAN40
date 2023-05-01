/// \file colmap.h
/// \brief Functions to read sparse COLMAP reconstructions.
#ifndef COLMAP_H_
#define COLMAP_H_

#include "reconstruction.h"
#include <string>
#include <vector>

namespace ppr {

/// \brief Read COLMAP camera file, in text format.
///
/// The format is described at https://colmap.github.io/format.html#cameras-txt
/// \param path path to camera file.
/// \return The list of cameras.
std::vector<Reconstruction::Camera> ReadCOLMAPCameras(const std::string &path);

/// \brief Read COLMAP image file, in text format.
///
/// The format is described at https://colmap.github.io/format.html#images-txt
/// \param path path to image file.
/// \return The list of images.
std::vector<Reconstruction::Image> ReadCOLMAPImages(const std::string &path);

/// \brief Read COLMAP 3D point file, in text format.
///
/// The format is described at https://colmap.github.io/format.html#points3d-txt
/// \param path path to 3D point file.
/// \return The list of 3D points.
std::vector<Reconstruction::Point3d>
ReadCOLMAPPoints3d(const std::string &path);

/// Read sparse COLMAP reconstruction, in text format.
/// \param dir Directory containing the sparse reconstruction.
/// \return The reconstruction.
Reconstruction ReadCOLMAPReconstruction(const std::string &dir);

} // namespace ppr

#endif /* COLMAP_H_ */
