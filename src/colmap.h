#ifndef COLMAP_H_
#define COLMAP_H_

#include "reconstruction.h"
#include <string>
#include <vector>

namespace ppr {

std::vector<Reconstruction::Camera> ReadCOLMAPCameras(const std::string &path);
std::vector<Reconstruction::Image> ReadCOLMAPImages(const std::string &path);
std::vector<Reconstruction::Point3d>
ReadCOLMAPPoints3d(const std::string &path);
Reconstruction ReadCOLMAPReconstruction(const std::string &dir);

} // namespace ppr

#endif /* COLMAP_H_ */
