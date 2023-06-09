#include "colmap.h"
#include "util.h"
#include <filesystem>
#include <fstream>

namespace ppr {

std::vector<Reconstruction::Camera> ReadCOLMAPCameras(const std::string &path) {
  std::ifstream file(path);
  std::string str;

  // Skip first three lines
  for (int i = 0; i < 3; i++) {
    std::getline(file, str);
  }

  std::vector<Reconstruction::Camera> cameras;

  while (std::getline(file, str)) {
    // #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
    std::vector<std::string> comp = SplitString(str, " ");
    uint32_t id = static_cast<uint32_t>(std::stoul(comp[0]));
    size_t width = static_cast<size_t>(std::stoull(comp[2]));
    size_t height = static_cast<size_t>(std::stoull(comp[3]));
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
    cameras.push_back(Reconstruction::Camera(id, width, height, f, c));
  }

  return cameras;
}

std::vector<Reconstruction::Image> ReadCOLMAPImages(const std::string &path) {
  std::ifstream file(path);
  std::string str;

  // Skip first four lines
  for (int i = 0; i < 4; i++) {
    std::getline(file, str);
  }

  std::vector<Reconstruction::Image> images;

  while (std::getline(file, str)) {
    // #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    std::vector<std::string> comp = SplitString(str, " ");
    uint32_t id = static_cast<uint32_t>(std::stoul(comp[0]));
    Eigen::Quaterniond q({std::stod(comp[1]), std::stod(comp[2]),
                          std::stod(comp[3]), std::stod(comp[4])});
    Eigen::Vector3d t(
        {std::stod(comp[5]), std::stod(comp[6]), std::stod(comp[7])});
    uint32_t cam_id = static_cast<uint32_t>(std::stoul(comp[8]));
    const std::string &name = comp[9];
    Reconstruction::Image image(id, q, t, cam_id, name);

    // #   POINTS2D[] as (X, Y, POINT3D_ID)
    std::getline(file, str);
    comp = SplitString(str, " ");
    for (size_t i = 0; i < comp.size() / 3; i++) {
      Eigen::Vector2d point(
          {std::stod(comp[3 * i]), std::stod(comp[3 * i + 1])});
      uint64_t p3d_id = static_cast<uint64_t>(std::stoull(comp[3 * i + 2]));
      image.AddPoint(point, p3d_id);
    }

    images.push_back(image);
  }

  return images;
}

std::vector<Reconstruction::Point3d>
ReadCOLMAPPoints3d(const std::string &path) {
  std::ifstream file(path);
  std::string str;

  // Skip first three lines
  for (int i = 0; i < 3; i++) {
    std::getline(file, str);
  }

  std::vector<Reconstruction::Point3d> points;

  while (std::getline(file, str)) {
    // clang-format off
    // #   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
    // clang-format on
    std::vector<std::string> comp = SplitString(str, " ");
    uint64_t id = static_cast<uint64_t>(std::stoull(comp[0]));
    Eigen::Vector3d xyz(
        {std::stod(comp[1]), std::stod(comp[2]), std::stod(comp[3])});
    Vector3uc color({static_cast<unsigned char>(std::stoul(comp[4])),
                     static_cast<unsigned char>(std::stoul(comp[5])),
                     static_cast<unsigned char>(std::stoul(comp[6]))});

    Reconstruction::Point3d point(id, xyz, color);

    size_t offset = 8;
    assert(comp.size() >= offset);
    assert((comp.size() - offset) % 2 == 0);
    for (size_t i = 0; i < (comp.size() - offset) / 2; i++) {
      uint32_t image_id =
          static_cast<uint32_t>(std::stoul(comp[offset + 2 * i]));
      uint32_t p2d_idx =
          static_cast<uint32_t>(std::stoul(comp[offset + 2 * i + 1]));
      point.AddTrackObservation(image_id, p2d_idx);
    }

    points.push_back(point);
  }

  return points;
}

Reconstruction ReadCOLMAPReconstruction(const std::string &dir) {
  std::filesystem::path base(dir);
  std::vector<Reconstruction::Camera> cameras =
      ReadCOLMAPCameras(base / "cameras.txt");
  std::vector<Reconstruction::Image> images =
      ReadCOLMAPImages(base / "images.txt");
  std::vector<Reconstruction::Point3d> points =
      ReadCOLMAPPoints3d(base / "points3D.txt");
  return Reconstruction(cameras, images, points);
}

} // namespace ppr
