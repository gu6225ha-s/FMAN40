#ifndef RECONSTRUCTION_H_
#define RECONSTRUCTION_H_

#include "polygon.h"
#include <Eigen/Dense>
#include <cstdint>
#include <map>

namespace ppr {

typedef Eigen::Matrix<unsigned char, 3, 1> Vector3uc;

/// \brief Sparse 3D reconstruction.
class Reconstruction {
public:
  /// \brief Camera intrinsics.
  class Camera {
  public:
    /// \brief Create new camera.
    /// \param id unique id
    /// \param width image width
    /// \param height image height
    /// \param f focal length
    /// \param c principal point
    Camera(uint32_t id, size_t width, size_t height, const Eigen::Vector2d &f,
           const Eigen::Vector2d &c)
        : id_(id), width_(width), height_(height), f_(f), c_(c) {}

    /// \brief Get the unique id of the camera.
    /// \return The id of the camera.
    uint32_t Id() const { return id_; }

    /// \brief Get the width of images taken with the camera.
    /// \return The image width.
    size_t Width() const { return width_; }

    /// \brief Get the height of images taken with the camera.
    /// \return The image height.
    size_t Height() const { return height_; }

    /// \brief Get the camera intrinsic matrix.
    /// \return The intrinsic matrix.
    Eigen::Matrix3d K() const {
      return Eigen::Matrix3d({{f_(0), 0, c_(0)}, {0, f_(1), c_(1)}, {0, 0, 1}});
    };

  private:
    uint32_t id_;           // Unique ID
    size_t width_, height_; // Size of images
    Eigen::Vector2d f_;     // Focal length
    Eigen::Vector2d c_;     // Principal point
  };

  /// \brief Camera extrinsics.
  class Image {
  public:
    /// \brief Create new image.
    /// \param id unique id
    /// \param q orientation quaternion
    /// \param t translation vector
    /// \param cam_id id of the corresponding Camera
    /// \param name image name
    Image(uint32_t id, const Eigen::Quaterniond &q, const Eigen::Vector3d &t,
          uint32_t cam_id, const std::string &name)
        : id_(id), q_(q), t_(t), cam_id_(cam_id), name_(name) {}

    /// \brief Get the unique id of the image.
    /// \return The id of the image.
    uint32_t Id() const { return id_; }

    /// \brief Get the orientation of the image.
    /// \return The orientation as a quaternion.
    const Eigen::Quaterniond &Q() const { return q_; }

    /// \brief Get the translation of the image.
    /// \return The translation vector.
    const Eigen::Vector3d &T() const { return t_; }

    /// \brief Get the unique id of the Camera that took the image.
    /// \return The id of the Camera.
    uint32_t CamId() const { return cam_id_; };

    /// \brief Get the name of the image.
    /// \return The image name.
    const std::string &Name() const { return name_; }

    /// \brief Get the points of the image.
    /// \return The vector of image points.
    const std::vector<Eigen::Vector2d> &Points() const { return points_; }

    /// \brief Get the ids of the Point3d:s corresponding to the image points.
    /// \return The vector of Point3d ids.
    const std::vector<uint64_t> &P3dIds() const { return p3d_ids_; }

    /// \brief Add point to the image.
    /// \param point image point
    /// \param p3d_id id of the corresponding Point3d
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

  /// \brief 3d point.
  class Point3d {
  public:
    /// \brief Create new 3d point.
    /// \param id unique id
    /// \param point point coordinates
    /// \param color color of the point
    Point3d(uint64_t id, const Eigen::Vector3d &point, const Vector3uc &color)
        : id_(id), point_(point), color_(color) {}

    /// \brief Get the unique id of the 3d point.
    /// \return The id of the 3d point.
    uint64_t Id() const { return id_; }

    /// \brief Get the coordinates of the point.
    /// \return The point coordinates.
    const Eigen::Vector3d &Point() const { return point_; }

    /// \brief Get the color of the point.
    /// \return The point color.
    const Vector3uc &Color() const { return color_; }

    /// \brief Get the track of the point.
    ///
    /// The track is a vector of pairs where the first member is the Image id
    /// and the second is the index into the Image#Points vector.
    /// \return The point track.
    const std::vector<std::pair<uint32_t, uint32_t>> &Track() const {
      return track_;
    }

    /// \brief Add observation to the track.
    /// \param image_id image id
    /// \param p2d_idx image point index
    void AddTrackObservation(uint32_t image_id, uint32_t p2d_idx) {
      track_.push_back(std::make_pair(image_id, p2d_idx));
    }

  private:
    uint64_t id_;                                      // Unique ID
    Eigen::Vector3d point_;                            // Point
    Vector3uc color_;                                  // Color
    std::vector<std::pair<uint32_t, uint32_t>> track_; // Track
  };

  /// \brief Create new reconstruction.
  /// \param cameras vector of Camera:s
  /// \param images vector of Image:s
  /// \param points vector of Point3d:s
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

  /// \brief Get the cameras of the reconstruction.
  /// \return A map from Camera#Id to Camera.
  const std::map<uint32_t, Camera> &Cameras() const { return cameras_; }

  /// \brief Get the images of the reconstruction.
  /// \return A map from Image#Id to Image.
  const std::map<uint32_t, Image> &Images() const { return images_; }

  /// \brief Get the 3d points of the reconstruction.
  /// \return A map from Point3d#Id to Point3d.
  const std::map<uint64_t, Point3d> &Points3d() { return points3d_; }

  /// \brief Get a camera.
  /// \param id camera id
  /// \return The Camera.
  const Camera &GetCamera(uint32_t id) const { return cameras_.at(id); }

  /// \brief Get an image.
  /// \param id image id
  /// \return The Image.
  const Image &GetImage(uint32_t id) const { return images_.at(id); }

  /// \brief Get a 3d point.
  /// \param id 3d point id
  /// \return The Point3d.
  const Point3d &GetPoint3d(uint64_t id) const { return points3d_.at(id); }

  /// \brief Find image by name.
  /// \param name image name
  /// \return A pointer to the Image or a null pointer if not found.
  const Image *FindImage(const std::string &name) const;

  /// \brief Estimate 3d plane from point correspondences within a polygon.
  /// \param polygon2d polygon
  /// \param image image in which the polygon is defined
  /// \param niter number of RANSAC iterations
  /// \param thr inlier threshold (number of pixels)
  /// \return A vector \f$n\f$ such that \f$n^T X + 1 = 0\f$ for the 3d points
  /// \f$X\f$ in the plane.
  Eigen::Vector3d EstimatePlane(const Polygon2d &polygon2d, const Image &image,
                                size_t niter, double thr) const;

  /// \brief Project a 2d polygon onto a 3d plane.
  /// \param polygon2d 2d polygon
  /// \param image image in which the 2d polygon is defined
  /// \param plane 3d plane
  /// \return The 3d polygon.
  /// \see Reconstruction#EstimatePlane
  Polygon3d ProjectPolygon(const Polygon2d &polygon2d, const Image &image,
                           const Eigen::Vector3d &plane) const;

  /// \brief Compute a homography from a 3d plane to an image.
  /// \param polygon2d 2d polygon
  /// \param image image in which the 2d polygon is defined
  /// \param plane 3d plane
  /// \return The homography from 3d plane to image.
  /// \see Reconstruction#EstimatePlane
  Eigen::Matrix3d ComputeHomography(const Polygon2d &polygon2d,
                                    const Image &image,
                                    const Eigen::Vector3d &plane) const;

  /// \brief Get modified camera matrix and 2d point.
  /// \param image image in which the 2d polygon is defined
  /// \param item item contains the information of current 3d point
  /// \return the modified rotation matrix R, transfer vector t, and 2d point
  std::tuple<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Vector2d>
  ProcessInfor(const Image &image, const std::pair<uint32_t, uint32_t> &item,
               Eigen::Matrix4d H_camera) const;

private:
  std::map<uint32_t, Camera> cameras_;   // Cameras
  std::map<uint32_t, Image> images_;     // Images
  std::map<uint64_t, Point3d> points3d_; // 3D points
};

} // namespace ppr

#endif /* RECONSTRUCTION_H_ */
