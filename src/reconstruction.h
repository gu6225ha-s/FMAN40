#ifndef RECONSTRUCTION_H_
#define RECONSTRUCTION_H_

#include "polygon.h"
#include <Eigen/Dense>
#include <cstdint>
#include <map>

namespace ppr {

typedef Eigen::Matrix<unsigned char, 3, 1> Vector3uc;

class Reconstruction {
public:
  class Camera {
  public:
    Camera(uint32_t id, size_t width, size_t height, const Eigen::Vector2d &f,
           const Eigen::Vector2d &c)
        : id_(id), width_(width), height_(height), f_(f), c_(c) {}

    uint32_t Id() const { return id_; }
    size_t Width() const { return width_; }
    size_t Height() const { return height_; }
    Eigen::Matrix3d K() const {
      return Eigen::Matrix3d({{f_(0), 0, c_(0)}, {0, f_(1), c_(1)}, {0, 0, 1}});
    };

  private:
    uint32_t id_;           // Unique ID
    size_t width_, height_; // Size of images
    Eigen::Vector2d f_;     // Focal length
    Eigen::Vector2d c_;     // Principal point
  };

  class Image {
  public:
    Image(uint32_t id, const Eigen::Quaterniond &q, const Eigen::Vector3d &t,
          uint32_t cam_id, const std::string &name)
        : id_(id), q_(q), t_(t), cam_id_(cam_id), name_(name) {}

    uint32_t Id() const { return id_; }
    const Eigen::Quaterniond &Q() const { return q_; }
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
    Eigen::Quaterniond q_;                // Rotation
    Eigen::Vector3d t_;                   // Translation
    uint32_t cam_id_;                     // Camera ID
    std::string name_;                    // Filename
    std::vector<Eigen::Vector2d> points_; // Keypoints
    std::vector<uint64_t> p3d_ids_;       // 3D point IDs
  };

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
  Eigen::Vector3d EstimatePlane(const Polygon2d &polygon2d,
                                const Image &image) const;
  Polygon3d ProjectPolygon(const Polygon2d &polygon2d, const Image &image,
                           const Eigen::Vector3d &plane) const;
  Eigen::Matrix3d ComputeHomography(const Polygon2d &polygon,
                                    const Image &image,
                                    const Eigen::Vector3d &plane) const;

private:
  std::map<uint32_t, Camera> cameras_;   // Cameras
  std::map<uint32_t, Image> images_;     // Images
  std::map<uint64_t, Point3d> points3d_; // 3D points
};

} // namespace ppr

#endif /* RECONSTRUCTION_H_ */
