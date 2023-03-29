#ifndef COLMAP_H_
#define COLMAP_H_

#include "camera.h"
#include <string>
#include <vector>

namespace ppr {

std::vector<Camera> ReadCOLMAPCameras(const std::string &path);

} // namespace ppr

#endif /* COLMAP_H_ */
