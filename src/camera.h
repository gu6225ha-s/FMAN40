#ifndef CAMERA_H_
#define CAMERA_H_

namespace ppr {

class CameraIntrinsics {
public:
  CameraIntrinsics(double fx, double fy, double cx, double cy)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}
  CameraIntrinsics(double f, double cx, double cy)
      : CameraIntrinsics(f, f, cx, cy) {}

private:
  double fx_, fy_; // Focal length
  double cx_, cy_; // Principal point
};

} // namespace ppr

#endif /* CAMERA_H_ */
