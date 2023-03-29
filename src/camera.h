#ifndef CAMERA_H_
#define CAMERA_H_

#include <Eigen/Dense>
#include <cstdint>

namespace ppr {

class Camera {
public:
  Camera(uint32_t id, double fx, double fy, double cx, double cy)
      : id_(id), fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}
  Camera(uint32_t id, double f, double cx, double cy)
      : Camera(id, f, f, cx, cy) {}

  uint32_t Id() const { return id_; }
  Eigen::Matrix3d K() const {
    return Eigen::Matrix3d({{fx_, 0, cx_}, {0, fy_, cy_}, {0, 0, 1}});
  };

private:
  uint32_t id_;    // Unique ID
  double fx_, fy_; // Focal length
  double cx_, cy_; // Principal point
};

} // namespace ppr

#endif /* CAMERA_H_ */
