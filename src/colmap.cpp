#include "colmap.h"
#include "str_util.h"
#include <fstream>

namespace ppr {

std::vector<CameraIntrinsics> ReadCOLMAPIntrinsics(const std::string &path) {
  // https://colmap.github.io/format.html#cameras-txt
  std::ifstream file(path);
  std::string str;

  // Skip first three lines
  for (int i = 0; i < 3; i++) {
    std::getline(file, str);
  }

  std::vector<CameraIntrinsics> intrinsics;

  while (std::getline(file, str)) {
    std::vector<std::string> comp = split(str, " ");
    if (comp[1] == "SIMPLE_PINHOLE") {
      intrinsics.push_back(CameraIntrinsics(
          std::stod(comp[4]), std::stod(comp[5]), std::stod(comp[6])));
    } else if (comp[1] == "PINHOLE") {
      intrinsics.push_back(
          CameraIntrinsics(std::stod(comp[4]), std::stod(comp[5]),
                           std::stod(comp[6]), std::stod(comp[7])));
    } else {
      throw std::runtime_error("Unsupported camera: " + comp[1]); // TESTME
    }
  }

  return intrinsics;
}

} // namespace ppr
