#ifndef COLMAP_H_
#define COLMAP_H_

#include "camera.h"
#include "image.h"
#include "point3d.h"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ppr {

std::vector<Camera> ReadCOLMAPCameras(const std::string &path);
std::vector<Image> ReadCOLMAPImages(const std::string &path);
std::map<uint64_t, Point3d> ReadCOLMAPPoints3d(const std::string &path);

} // namespace ppr

#endif /* COLMAP_H_ */
