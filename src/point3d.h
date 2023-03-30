#ifndef POINT3D_H_
#define POINT3D_H_

#include <Eigen/Dense>
#include <cstdint>
#include <vector>

namespace ppr {

typedef Eigen::Matrix<unsigned char, 3, 1> Vector3uc;

class Point3d {
public:
  Point3d(uint64_t id, const Eigen::Vector3d &point, const Vector3uc &color)
      : id_(id), point_(point), color_(color) {}

  uint64_t Id() const { return id_; }
  const Eigen::Vector3d &Point() const { return point_; }
  const Vector3uc &Color() const { return color_; }
  const std::vector<std::pair<uint32_t, uint32_t>> &Track() const {
    return track_;
  }
  void AddTrackObservation(uint32_t image_id, uint32_t p2d_idx) {
    track_.push_back(std::make_pair(image_id, p2d_idx));
  }

private:
  uint64_t id_;                                      // Unique ID
  Eigen::Vector3d point_;                            // Point
  Vector3uc color_;                                  // Color
  std::vector<std::pair<uint32_t, uint32_t>> track_; // Track
};

} // namespace ppr

#endif /* POINT3D_H_ */
