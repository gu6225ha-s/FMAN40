#ifndef CAMERA_H_
#define CAMERA_H_

#include <Eigen/Dense>
#include <cstdint>

namespace ppr {

class Camera {
public:
  Camera(uint32_t id, const Eigen::Vector2d &f, const Eigen::Vector2d &c)
      : id_(id), f_(f), c_(c) {}

  uint32_t Id() const { return id_; }
  Eigen::Matrix3d K() const {
    return Eigen::Matrix3d({{f_(0), 0, c_(0)}, {0, f_(1), c_(1)}, {0, 0, 1}});
  };

private:
  uint32_t id_;       // Unique ID
  Eigen::Vector2d f_; // Focal length
  Eigen::Vector2d c_; // Principal point
};

} // namespace ppr

#endif /* CAMERA_H_ */
