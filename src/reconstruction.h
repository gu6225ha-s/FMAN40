#ifndef RECONSTRUCTION_H_
#define RECONSTRUCTION_H_

#include "camera.h"
#include "image.h"
#include "point3d.h"
#include "polygon.h"
#include <Eigen/Dense>
#include <cstdint>
#include <map>

namespace ppr {

class Reconstruction {
public:
  Reconstruction(const std::vector<Camera> &cameras,
                 const std::vector<Image> &images,
                 const std::vector<Point3d> points) {
    for (auto c : cameras) {
      cameras_.emplace(c.Id(), std::move(c));
    }
    for (auto i : images) {
      images_.emplace(i.Id(), std::move(i));
    }
    for (auto p : points) {
      points3d_.emplace(p.Id(), std::move(p));
    }
  }

  const std::map<uint32_t, Camera> &Cameras() const { return cameras_; }
  const std::map<uint32_t, Image> &Images() const { return images_; }
  const std::map<uint64_t, Point3d> &Points3d() { return points3d_; }

  const Camera &GetCamera(uint32_t id) const { return cameras_.at(id); }
  const Image &GetImage(uint32_t id) const { return images_.at(id); }
  const Point3d &GetPoint3d(uint64_t id) const { return points3d_.at(id); }

  const Image *FindImage(const std::string &name) const;
  Eigen::Vector3d EstimatePlane(const Polygon2d &polygon2d, const Image &image,
                                Eigen::Vector3d &color) const;
  Polygon3d ProjectPolygon(const Polygon2d &polygon2d, const Image &image,
                           const Eigen::Vector3d &plane) const;

private:
  std::map<uint32_t, Camera> cameras_;   // Cameras
  std::map<uint32_t, Image> images_;     // Images
  std::map<uint64_t, Point3d> points3d_; // 3D points
};

} // namespace ppr

#endif /* RECONSTRUCTION_H_ */
