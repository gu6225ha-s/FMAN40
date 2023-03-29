#include "colmap.h"
#include <algorithm>
#include <filesystem>
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
              << " ppr [-h] -ws WORKSPACE" << std::endl
              << "Options:" << std::endl
              << " -h, --help       Show help message" << std::endl
              << " -ws, --workspace COLMAP workspace folder" << std::endl;
    return 0;
  }

  char *workspace = GetLongShortOption(argv, argv + argc, "--workspace", "-ws");
  if (workspace == nullptr) {
    std::cerr << "Path to COLMAP workspace folder must be specified"
              << std::endl;
    return -1;
  }

  std::filesystem::path wspath(workspace);

  std::vector<ppr::Camera> cameras =
      ppr::ReadCOLMAPCameras(wspath / "cameras.txt");

  std::vector<ppr::Image> images = ppr::ReadCOLMAPImages(wspath / "images.txt");

  return 0;
}
