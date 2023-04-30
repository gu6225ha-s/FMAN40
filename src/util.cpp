#include "util.h"
#include <fstream>
#include <iomanip>

namespace ppr {

std::vector<std::string> SplitString(const std::string &str,
                                     const std::string &delim) {
  size_t start = 0, end;
  std::vector<std::string> ret;

  while ((end = str.find(delim, start)) != std::string::npos) {
    ret.push_back(str.substr(start, end - start));
    start = end + delim.length();
  }

  ret.push_back(str.substr(start));
  return ret;
}

std::vector<std::pair<std::string, ppr::Polygon2d>>
ReadPolygons(const std::string &path) {
  std::ifstream file(path);
  std::string str;

  // Skip first six lines
  for (int i = 0; i < 6; i++) {
    std::getline(file, str);
  }

  std::vector<std::pair<std::string, Polygon2d>> polygons;

  while (std::getline(file, str)) {
    // Image Id, Image name, Camera Id, Num polygons
    std::vector<std::string> comp = SplitString(str, " ");
    std::string name(comp[1]);

    int n_poly = std::stoi(comp[3]);
    for (int i = 0; i < n_poly; i++) {
      std::vector<Eigen::Vector2d> points;
      std::getline(file, str);
      comp = SplitString(str, ",");

      for (size_t j = 0; j < comp.size() / 2; j++) {
        // FIXME: Subtract 1 to account for MATLAB:s one-indexing?
        points.emplace_back(std::stod(comp[2 * j]), std::stod(comp[2 * j + 1]));
      }
      polygons.push_back(std::make_pair(name, Polygon2d(points)));
    }
  }

  return polygons;
}

constexpr int GLTF_TRIANGLES = 4;
constexpr int GLTF_FLOAT = 5126;
constexpr int GLTF_UNSIGNED_INT = 5125;
constexpr int GLTF_ARRAY_BUFFER = 34962;
constexpr int GLTF_ELEMENT_ARRAY_BUFFER = 34963;

static void WriteGltfBuffer(const std::vector<Mesh> &meshes,
                            const std::string &path,
                            std::vector<size_t> &vert_size,
                            std::vector<size_t> &tc_size,
                            std::vector<size_t> &tri_size) {
  size_t size = 0;

  for (const auto &mesh : meshes) {
    size += 3 * mesh.Verts().size() * sizeof(float) +
            2 * mesh.Texcoords().size() * sizeof(float) +
            3 * mesh.Triangles().size() * sizeof(unsigned int);
  }

  char *data = new char[size];
  size_t offset = 0;

  for (const auto &mesh : meshes) {
    const auto &verts = mesh.Verts();
    const auto &texcoords = mesh.Texcoords();
    const auto &triangles = mesh.Triangles();

    float *vdata = reinterpret_cast<float *>(data + offset);
    for (size_t i = 0; i < verts.size(); i++) {
      vdata[3 * i + 0] = verts[i].x();
      vdata[3 * i + 1] = verts[i].y();
      vdata[3 * i + 2] = verts[i].z();
    }
    vert_size.push_back(3 * verts.size() * sizeof(float));
    offset += vert_size.back();

    float *tcdata = reinterpret_cast<float *>(data + offset);
    for (size_t i = 0; i < texcoords.size(); i++) {
      tcdata[2 * i + 0] = texcoords[i].x();
      tcdata[2 * i + 1] = texcoords[i].y();
    }
    tc_size.push_back(2 * texcoords.size() * sizeof(float));
    offset += tc_size.back();

    unsigned int *tdata = reinterpret_cast<unsigned int *>(data + offset);
    for (size_t i = 0; i < triangles.size(); i++) {
      tdata[3 * i + 0] = std::get<0>(triangles[i]);
      tdata[3 * i + 1] = std::get<1>(triangles[i]);
      tdata[3 * i + 2] = std::get<2>(triangles[i]);
    }
    tri_size.push_back(3 * triangles.size() * sizeof(unsigned int));
    offset += tri_size.back();
  }

  std::ofstream file(path);
  file.write(data, size);

  delete[] data;
}

static void CopyImages(const std::vector<Mesh> &meshes,
                       const std::filesystem::path &src_dir,
                       const std::filesystem::path &dst_dir) {
  for (const auto &mesh : meshes) {
    std::filesystem::path src_path(src_dir / mesh.ImageName());
    std::filesystem::path dst_path(dst_dir / mesh.ImageName());
    if (!std::filesystem::exists(dst_path)) {
      std::filesystem::copy_file(src_path, dst_path);
    }
  }
}

void WriteGltf(const std::vector<Mesh> &meshes,
               const std::filesystem::path &image_dir,
               const std::filesystem::path &gltf_path) {
  // Create output directory if it doesn't exist
  const auto parent_path = gltf_path.parent_path();
  if (!parent_path.empty()) {
    std::filesystem::create_directories(gltf_path.parent_path());
  }

  // Write buffer file
  std::filesystem::path bin_path(gltf_path);
  bin_path.replace_extension(".bin");
  std::vector<size_t> vert_size, tc_size, tri_size;
  WriteGltfBuffer(meshes, bin_path, vert_size, tc_size, tri_size);

  // Copy images
  CopyImages(meshes, image_dir, gltf_path.parent_path());

  // Write glTF file
  int precision = std::numeric_limits<float>::digits10 + 1;
  std::vector<std::string> images;
  std::ofstream file(gltf_path);
  file << std::scientific << std::setprecision(precision);
  file << "{\n";
  file << "  \"asset\": {\n";
  file << "    \"version\": \"2.0\"\n";
  file << "  },\n";
  file << "  \"scene\": 0,\n";
  file << "  \"scenes\": [{\n";
  file << "    \"nodes\": [0]\n";
  file << "  }],\n";
  file << "  \"nodes\": [{\n";
  file << "    \"mesh\": 0\n";
  file << "  }],\n";
  file << "  \"meshes\": [{\n";
  file << "    \"primitives\": [\n";
  for (size_t i = 0; i < meshes.size(); i++) {
    file << "      {\n";
    file << "        \"attributes\": {\n";
    file << "          \"POSITION\": " << 3 * i << ",\n";
    file << "          \"TEXCOORD_0\": " << 3 * i + 1 << "\n";
    file << "        },\n";
    file << "        \"indices\": " << 3 * i + 2 << ",\n";
    file << "        \"mode\": " << GLTF_TRIANGLES << ",\n";
    file << "        \"material\": " << i << "\n";
    file << "      }";
    if (i < meshes.size() - 1) {
      file << ",";
    }
    file << "\n";
  }
  file << "    ]\n";
  file << "  }],\n";
  file << "  \"accessors\": [\n";
  for (size_t i = 0; i < meshes.size(); i++) {
    Eigen::Vector3d min, max;
    meshes[i].CalcBbox(min, max);
    file << "    {\n";
    file << "      \"bufferView\": " << 3 * i << ",\n";
    file << "      \"componentType\": " << GLTF_FLOAT << ",\n";
    file << "      \"count\": " << meshes[i].Verts().size() << ",\n";
    file << "      \"max\": [\n";
    file << "        " << (float)max.x() << ",\n";
    file << "        " << (float)max.y() << ",\n";
    file << "        " << (float)max.z() << "\n";
    file << "      ],\n";
    file << "      \"min\": [\n";
    file << "        " << (float)min.x() << ",\n";
    file << "        " << (float)min.y() << ",\n";
    file << "        " << (float)min.z() << "\n";
    file << "      ],\n";
    file << "      \"type\": \"VEC3\"\n";
    file << "    },\n";
    file << "    {\n";
    file << "      \"bufferView\": " << 3 * i + 1 << ",\n";
    file << "      \"componentType\": " << GLTF_FLOAT << ",\n";
    file << "      \"count\": " << meshes[i].Texcoords().size() << ",\n";
    file << "      \"type\": \"VEC2\"\n";
    file << "    },\n";
    file << "    {\n";
    file << "      \"bufferView\": " << 3 * i + 2 << ",\n";
    file << "      \"componentType\": " << GLTF_UNSIGNED_INT << ",\n";
    file << "      \"count\": " << 3 * meshes[i].Triangles().size() << ",\n";
    file << "      \"type\": \"SCALAR\"\n";
    file << "    }";
    if (i < meshes.size() - 1) {
      file << ",";
    }
    file << "\n";
  }
  file << "  ],\n";
  file << "  \"materials\": [\n";
  for (size_t i = 0; i < meshes.size(); i++) {
    size_t j;
    for (j = 0; j < images.size(); j++) {
      if (images[j] == meshes[i].ImageName()) {
        break;
      }
    }
    if (j == images.size()) {
      images.push_back(meshes[i].ImageName());
    }
    file << "    {\n";
    file << "      \"pbrMetallicRoughness\": {\n";
    file << "        \"baseColorTexture\": {\n";
    file << "          \"index\": " << j << "\n";
    file << "        },\n";
    file << "        \"metallicFactor\": 0.0\n";
    file << "      }\n";
    file << "    }";
    if (i < meshes.size() - 1) {
      file << ",";
    }
    file << "\n";
  }
  file << "  ],\n";
  file << "  \"textures\": [\n";
  for (size_t i = 0; i < images.size(); i++) {
    file << "    {\n";
    file << "      \"source\": " << i << "\n";
    file << "    }";
    if (i < images.size() - 1) {
      file << ",";
    }
    file << "\n";
  }
  file << "  ],\n";
  file << "  \"images\": [\n";
  for (size_t i = 0; i < images.size(); i++) {
    file << "    {\n";
    file << "      \"uri\": \"" << images[i] << "\"\n";
    file << "    }";
    if (i < images.size() - 1) {
      file << ",";
    }
    file << "\n";
  }
  file << "  ],\n";
  file << "  \"bufferViews\": [\n";
  size_t offset = 0;
  for (size_t i = 0; i < meshes.size(); i++) {
    file << "    {\n";
    file << "      \"buffer\": 0,\n";
    file << "      \"byteOffset\": " << offset << ",\n";
    file << "      \"byteLength\": " << vert_size[i] << ",\n";
    file << "      \"target\": " << GLTF_ARRAY_BUFFER << "\n";
    file << "    },\n";
    offset += vert_size[i];
    file << "    {\n";
    file << "      \"buffer\": 0,\n";
    file << "      \"byteOffset\": " << offset << ",\n";
    file << "      \"byteLength\": " << tc_size[i] << ",\n";
    file << "      \"target\": " << GLTF_ARRAY_BUFFER << "\n";
    file << "    },\n";
    offset += tc_size[i];
    file << "    {\n";
    file << "      \"buffer\": 0,\n";
    file << "      \"byteOffset\": " << offset << ",\n";
    file << "      \"byteLength\": " << tri_size[i] << ",\n";
    file << "      \"target\": " << GLTF_ELEMENT_ARRAY_BUFFER << "\n";
    file << "    }";
    if (i < meshes.size() - 1) {
      file << ",";
    }
    file << "\n";
    offset += tri_size[i];
  }
  file << "  ],\n";
  file << "  \"buffers\": [{\n";
  file << "    \"byteLength\": " << offset << ",\n";
  file << "    \"uri\": " << bin_path.filename() << "\n";
  file << "  }]\n";
  file << "}";
}

void PlaneEstimator::AddCorrespondence(const Eigen::Vector3d &x,
                                       const Eigen::Vector3d &y,
                                       const Eigen::Matrix3d &R,
                                       const Eigen::Vector3d &t) {
  x_.push_back(x);
  y_.push_back(y);
  Rx_.push_back(R * x);
  t_.push_back(t);
}

Eigen::Vector3d PlaneEstimator::Solve() const {
  size_t n = x_.size();
  Eigen::MatrixXd A(3 * n, 3 + n);
  Eigen::MatrixXd b(3 * n, 1);

  A.setZero();
  b.setZero();

  for (size_t i = 0; i < n; i++) {
    A.block(3 * i, 0, 3, 3) = t_[i] * x_[i].transpose();
    A.block(3 * i, 3 + i, 3, 1) = y_[i];
    b.block(3 * i, 0, 3, 1) = Rx_[i];
  }

  Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
  return x.head(3);
}

const RgbImage &ImageCache::operator[](const std::string &name) {
  const auto fn = path_ / name;
  auto it = images_.find(fn);
  if (it == images_.end()) {
    RgbImage image;
    image.ReadJpeg(fn);
    images_.emplace(fn, std::move(image));
  }
  return images_[fn];
}

} // namespace ppr
