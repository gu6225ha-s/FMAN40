#include "colmap.h"
#include "mesh.h"
#include "util.h"
#include <algorithm>
#include <iostream>

static bool HasOption(char **begin, char **end, const std::string &option) {
  return std::find(begin, end, option) != end;
}

static bool HasLongShortOption(char **begin, char **end,
                               const std::string &longopt,
                               const std::string &shortopt) {
  return HasOption(begin, end, longopt) || HasOption(begin, end, shortopt);
}

static char *GetOption(char **begin, char **end, const std::string &option) {
  char **itr = std::find(begin, end, option);
  if (itr != end && ++itr != end) {
    return *itr;
  }
  return 0;
}

static char *GetLongShortOption(char **begin, char **end,
                                const std::string &longopt,
                                const std::string &shortopt) {
  char *val = GetOption(begin, end, longopt);
  if (val == nullptr) {
    val = GetOption(begin, end, shortopt);
  }
  return val;
}

int main(int argc, char *argv[]) {

  if (HasLongShortOption(argv, argv + argc, "--help", "-h")) {
    std::cout << "Piecewise Planar Reconstructions from Multiple Homographies"
              << std::endl
              << "Usage:" << std::endl
              << " ppr [-h] -ws WORKSPACE -poly POLYGONS [-out OUTPUT]"
              << std::endl
              << "Options:" << std::endl
              << " -h, --help        Show help message" << std::endl
              << " -ws, --workspace  COLMAP workspace folder" << std::endl
              << " -poly, --polygons Polygon file" << std::endl
              << " -out, --output    Output glTF file" << std::endl;
    return 0;
  }

  char *workspace = GetLongShortOption(argv, argv + argc, "--workspace", "-ws");
  if (workspace == nullptr) {
    std::cerr << "Path to COLMAP workspace folder must be specified"
              << std::endl;
    return -1;
  }

  std::cout << "Reading COLMAP reconstruction from " << workspace << std::endl;
  ppr::Reconstruction reconstruction = ppr::ReadCOLMAPReconstruction(workspace);
  if (reconstruction.Cameras().size() != 1) {
    std::cerr << "Can only handle a single camera" << std::endl;
    return -1;
  }

  char *polygons = GetLongShortOption(argv, argv + argc, "--polygons", "-poly");
  if (polygons == nullptr) {
    std::cerr << "Path to polygon file must be specified" << std::endl;
    return -1;
  }

  std::cout << "Reading polygons from " << polygons << std::endl;
  std::vector<std::pair<std::string, ppr::Polygon2d>> polys2d =
      ppr::ReadPolygons(polygons);

  std::cout << "Creating piecewise planar reconstruction" << std::endl;
  std::vector<ppr::Mesh> meshes;
  for (size_t i = 0; i < polys2d.size(); i++) {
    const std::string &image_name = polys2d[i].first;
    ppr::Polygon2d &poly2d = polys2d[i].second;
    std::cout << "Processing polygon " << i + 1 << "/" << polys2d.size()
              << " (image: " << image_name << ")" << std::endl;
    const ppr::Image *image = reconstruction.FindImage(image_name);
    assert(image);
    if (poly2d.Area() > 0) {
      poly2d.Reverse();
    }
    auto triangles = poly2d.Triangulate();
    Eigen::Vector3d color;
    auto n = reconstruction.EstimatePlane(poly2d, *image, color);
    ppr::Polygon3d poly3d = reconstruction.ProjectPolygon(poly2d, *image, n);
    meshes.emplace_back(poly3d.Points(), triangles, color);
  }

  char *output = GetLongShortOption(argv, argv + argc, "--output", "-out");
  if (output) {
    WriteGltf(meshes, output);
    std::cout << "Wrote " << output << std::endl;
  }

  return 0;
}
