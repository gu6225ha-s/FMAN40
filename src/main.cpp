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

  std::vector<std::pair<std::string, ppr::Polygon2d>> polys2d =
      ppr::ReadPolygons(polygons);
  ppr::Mesh mesh;

  for (const auto &item : polys2d) {
    const ppr::Image *image = reconstruction.FindImage(item.first);
    assert(image);
    std::vector<std::tuple<int, int, int>> triangles =
        item.second.Triangulate();
    Eigen::Vector3d n = reconstruction.EstimatePlane(item.second, *image);
    ppr::Polygon3d p3d = reconstruction.ProjectPolygon(item.second, *image, n);
    mesh.AddTriangles(p3d.Points(), triangles);
  }

  char *output = GetLongShortOption(argv, argv + argc, "--output", "-out");
  if (output) {
    mesh.WriteGltf(output);
    std::cout << "Wrote " << output << std::endl;
  }

  return 0;
}
