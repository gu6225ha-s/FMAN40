#include "mesh.h"
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace ppr {

void Mesh::AddTriangles(
    const std::vector<Eigen::Vector3d> &verts,
    const std::vector<std::tuple<int, int, int>> &triangles) {
  int n = verts_.size();
  verts_.insert(verts_.end(), verts.begin(), verts.end());
  for (size_t i = 0; i < triangles.size(); i++) {
    const std::tuple<int, int, int> &t = triangles[i];
    triangles_.emplace_back(std::get<0>(t) + n, std::get<1>(t) + n,
                            std::get<2>(t) + n);
  }
}

void Mesh::CalcBbox(Eigen::Vector3d &min, Eigen::Vector3d &max) const {
  assert(verts_.size() > 0);
  min = max = verts_[0];
  for (size_t i = 1; i < verts_.size(); i++) {
    min.x() = std::min(min.x(), verts_[i].x());
    min.y() = std::min(min.y(), verts_[i].y());
    min.z() = std::min(min.z(), verts_[i].z());
    max.x() = std::max(max.x(), verts_[i].x());
    max.y() = std::max(max.y(), verts_[i].y());
    max.z() = std::max(max.z(), verts_[i].z());
  }
}

constexpr int GLTF_TRIANGLES = 4;
constexpr int GLTF_FLOAT = 5126;
constexpr int GLTF_UNSIGNED_INT = 5125;
constexpr int GLTF_ARRAY_BUFFER = 34962;
constexpr int GLTF_ELEMENT_ARRAY_BUFFER = 34963;

void Mesh::WriteGltf(const std::string &path) const {
  // Write buffer file
  std::filesystem::path bin_path(path);
  bin_path.replace_extension(".bin");
  size_t vert_size, tri_size;
  WriteGltfBuffer(bin_path, vert_size, tri_size);

  Eigen::Vector3d min, max;
  CalcBbox(min, max);
  int precision = std::numeric_limits<float>::digits10 + 1;

  // Write glTF file
  // clang-format off
  std::ofstream file(path);
  file << std::fixed << std::setprecision(precision);
  file << "{\n";
  file << "  \"asset\": { \"version\": \"2.0\" },\n";
  file << "  \"scene\": 0,\n";
  file << "  \"scenes\": [{ \"nodes\": [0] }],\n";
  file << "  \"nodes\": [{ \"mesh\": 0 }],\n";
  file << "  \"meshes\": [{\n";
  file << "    \"primitives\": [{\n";
  file << "      \"attributes\": { \"POSITION\": 0 },\n";
  file << "      \"indices\": 1,\n";
  file << "      \"mode\": " << GLTF_TRIANGLES << "\n";
  file << "    }]\n";
  file << "  }],\n";
  file << "  \"accessors\": [{\n";
  file << "    \"bufferView\": 0,\n";
  file << "    \"componentType\": " << GLTF_FLOAT << ",\n";
  file << "    \"count\": " << verts_.size() << ",\n";
  file << "    \"max\": [" << (float)max.x() << ", " << (float)max.y() << ", " << (float)max.z() << "],\n";
  file << "    \"min\": [" << (float)min.x() << ", " << (float)min.y() << ", " << (float)min.z() << "],\n";
  file << "    \"type\": \"VEC3\" },{\n";
  file << "    \"bufferView\": 1,\n";
  file << "    \"componentType\": " << GLTF_UNSIGNED_INT << ",\n";
  file << "    \"count\": " << 3 * triangles_.size() << ",\n";
  file << "    \"type\": \"SCALAR\"\n";
  file << "  }],\n";
  file << "  \"bufferViews\": [{\n";
  file << "    \"buffer\": 0,\n";
  file << "    \"byteOffset\": 0,\n";
  file << "    \"byteLength\": " << vert_size << ",\n";
  file << "    \"target\": " << GLTF_ARRAY_BUFFER << " },{\n";
  file << "    \"buffer\": 0,\n";
  file << "    \"byteOffset\": " << vert_size << ",\n";
  file << "    \"byteLength\": " << tri_size << ",\n";
  file << "    \"target\": " << GLTF_ELEMENT_ARRAY_BUFFER << "\n";
  file << "  }],\n";
  file << "  \"buffers\": [{\n";
  file << "    \"byteLength\": " << vert_size + tri_size << ",\n";
  file << "    \"uri\": " << bin_path.filename() << "\n";
  file << "  }]\n";
  file << "}\n";
  // clang-format on
}

void Mesh::WriteGltfBuffer(const std::string &path, size_t &vert_size,
                           size_t &tri_size) const {
  vert_size = 3 * verts_.size() * sizeof(float);
  tri_size = 3 * triangles_.size() * sizeof(unsigned int);
  char *data = new char[vert_size + tri_size];

  float *verts = reinterpret_cast<float *>(data);
  for (size_t i = 0; i < verts_.size(); i++) {
    verts[3 * i + 0] = verts_[i].x();
    verts[3 * i + 1] = verts_[i].y();
    verts[3 * i + 2] = verts_[i].z();
  }

  unsigned int *tris = reinterpret_cast<unsigned int *>(data + vert_size);
  for (size_t i = 0; i < triangles_.size(); i++) {
    tris[3 * i + 0] = std::get<0>(triangles_[i]);
    tris[3 * i + 1] = std::get<1>(triangles_[i]);
    tris[3 * i + 2] = std::get<2>(triangles_[i]);
  }

  std::ofstream file(path);
  file.write(data, vert_size + tri_size);

  delete[] data;
}

} // namespace ppr
