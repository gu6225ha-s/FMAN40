#include "colmap.h"
#include "str_util.h"
#include <fstream>

namespace ppr {

std::vector<Camera> ReadCOLMAPCameras(const std::string &path) {
  // https://colmap.github.io/format.html#cameras-txt
  std::ifstream file(path);
  std::string str;

  // Skip first three lines
  for (int i = 0; i < 3; i++) {
    std::getline(file, str);
  }

  std::vector<Camera> cameras;

  while (std::getline(file, str)) {
    std::vector<std::string> comp = split(str, " ");
    uint32_t id = static_cast<uint32_t>(std::stoul(comp[0]));
    Eigen::Vector2d f, c;
    if (comp[1] == "SIMPLE_PINHOLE") {
      f(0) = f(1) = std::stod(comp[4]);
      c(0) = std::stod(comp[5]);
      c(1) = std::stod(comp[6]);
    } else if (comp[1] == "PINHOLE") {
      f(0) = std::stod(comp[4]);
      f(1) = std::stod(comp[5]);
      c(0) = std::stod(comp[6]);
      c(1) = std::stod(comp[7]);
    } else {
      throw std::runtime_error("Unsupported camera: " + comp[1]);
    }
    cameras.push_back(Camera(id, f, c));
  }

  return cameras;
}

std::vector<Image> ReadCOLMAPImages(const std::string &path) {
  // https://colmap.github.io/format.html#images-txt
  std::ifstream file(path);
  std::string str;

  // Skip first four lines
  for (int i = 0; i < 4; i++) {
    std::getline(file, str);
  }

  std::vector<Image> images;

  while (std::getline(file, str)) {
    std::vector<std::string> comp = split(str, " ");
    uint32_t id = static_cast<uint32_t>(std::stoul(comp[0]));
    Eigen::Vector4d q({std::stod(comp[1]), std::stod(comp[2]),
                       std::stod(comp[3]), std::stod(comp[4])});
    Eigen::Vector3d t(
        {std::stod(comp[5]), std::stod(comp[6]), std::stod(comp[7])});
    uint32_t cam_id = static_cast<uint32_t>(std::stoul(comp[8]));
    const std::string &name = comp[9];
    Image image(id, q, t, cam_id, name);

    std::getline(file, str);
    comp = split(str, " ");
    for (int i = 0; i < comp.size() / 3; i++) {
      Eigen::Vector2d point(
          {std::stod(comp[3 * i]), std::stod(comp[3 * i + 1])});
      uint64_t p3d_id = std::stoull(comp[3 * i + 2]);
      image.AddPoint(point, p3d_id);
    }

    images.push_back(image);
  }

  return images;
}

} // namespace ppr
