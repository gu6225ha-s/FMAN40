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

static ppr::Mesh CreateMesh(const ppr::Reconstruction &reconstruction,
                            const std::string &image_name,
                            const ppr::Polygon2d &poly2d) {
  const auto *image = reconstruction.FindImage(image_name);
  assert(image);

  const auto n = reconstruction.EstimatePlane(poly2d, *image);
  ppr::Polygon3d poly3d = reconstruction.ProjectPolygon(poly2d, *image, n);

  const auto triangles = poly2d.Triangulate();

  const auto &camera = reconstruction.GetCamera(image->CamId());
  const auto &points = poly2d.Points();
  std::vector<Eigen::Vector2d> texcoords;
  for (size_t j = 0; j < points.size(); j++) {
    texcoords.emplace_back((points[j].x() - 0.5) / camera.Width(),
                           (points[j].y() - 0.5) / camera.Height());
  }

  return ppr::Mesh(poly3d.Points(), triangles, texcoords, image_name);
}

int main(int argc, char *argv[]) {

  if (HasLongShortOption(argv, argv + argc, "--help", "-h")) {
    std::cout << "Piecewise Planar Reconstructions from Multiple Homographies"
              << std::endl
              << "Usage:" << std::endl
              << " ppr [-h] -sp SPARSE -poly POLYGONS -im IMAGES -out OUTPUT"
              << std::endl
              << "Options:" << std::endl
              << " -h, --help        Show help message" << std::endl
              << " -sp, --sparse     Path to COLMAP reconstruction" << std::endl
              << " -poly, --polygons Path to polygon file" << std::endl
              << " -im, --images     Path to image directory" << std::endl
              << " -out, --output    Path to output glTF file" << std::endl;
    return 0;
  }

  char *sparse = GetLongShortOption(argv, argv + argc, "--sparse", "-sp");
  if (sparse == nullptr) {
    std::cerr << "Path to COLMAP reconstruction must be specified" << std::endl;
    return -1;
  }

  std::cout << "Reading COLMAP reconstruction from " << sparse << std::endl;
  ppr::Reconstruction reconstruction = ppr::ReadCOLMAPReconstruction(sparse);
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

  char *images = GetLongShortOption(argv, argv + argc, "--images", "-im");
  if (images == nullptr) {
    std::cerr << "Path to image directory must be specified" << std::endl;
    return -1;
  }

  char *output = GetLongShortOption(argv, argv + argc, "--output", "-out");
  if (output == nullptr) {
    std::cerr << "Path to output glTF file must be specified" << std::endl;
    return -1;
  }

  std::cout << "Creating piecewise planar reconstruction" << std::endl;
  std::vector<ppr::Mesh> meshes;
  for (size_t i = 0; i < polys2d.size(); i++) {
    const std::string &image_name = polys2d[i].first;
    ppr::Polygon2d &poly2d = polys2d[i].second;
    std::cout << "Processing polygon " << i + 1 << "/" << polys2d.size()
              << " (image: " << image_name << ")" << std::endl;
    if (poly2d.Area() > 0) {
      poly2d.Reverse();
    }
    ppr::Mesh mesh = CreateMesh(reconstruction, image_name, poly2d);
    meshes.push_back(mesh);
  }

  WriteGltf(meshes, images, output);
  std::cout << "Wrote " << output << std::endl;

  return 0;
}
