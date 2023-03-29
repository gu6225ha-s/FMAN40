#ifndef IMAGE_H_
#define IMAGE_H_

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

namespace ppr {

class Image {
public:
  Image(uint32_t id, const Eigen::Vector4d &q, const Eigen::Vector3d &t,
        uint32_t cam_id, const std::string &name)
      : id_(id), q_(q), t_(t), cam_id_(cam_id), name_(name) {}

  uint32_t Id() const { return id_; }
  const Eigen::Vector4d &Q() const { return q_; }
  const Eigen::Vector3d &T() const { return t_; }
  uint32_t CamId() const { return cam_id_; };
  const std::string &Name() const { return name_; }
  const std::vector<Eigen::Vector2d> &Points() const { return points_; }
  const std::vector<uint64_t> &P3dIds() const { return p3d_ids_; }
  void AddPoint(const Eigen::Vector2d &point, uint64_t p3d_id) {
    points_.push_back(point);
    p3d_ids_.push_back(p3d_id);
  };

private:
  uint32_t id_;                         // Unique ID
  Eigen::Vector4d q_;                   // Rotation
  Eigen::Vector3d t_;                   // Translation
  uint32_t cam_id_;                     // Camera ID
  std::string name_;                    // Filename
  std::vector<Eigen::Vector2d> points_; // Keypoints
  std::vector<uint64_t> p3d_ids_;       // 3D point IDs
};

} // namespace ppr

#endif /* IMAGE_H_ */
