#include "reconstruction.h"
#include "util.h"
#include <iostream>
#include <random>

namespace ppr {

const Reconstruction::Image *
Reconstruction::FindImage(const std::string &name) const {
  for (const auto &[id, image] : images_) {
    if (image.Name() == name) {
      return &image;
    }
  }
  return nullptr;
}

/// Estimate plane by using robust estimation method RANSAC
Eigen::Vector3d Reconstruction::EstimatePlane(const Polygon2d &polygon2d,
                                              const Image &image, size_t niter,
                                              double thr) const {

  const std::vector<Eigen::Vector2d> &points = image.Points();
  const std::vector<uint64_t> &p3d_ids = image.P3dIds();
  const Eigen::Matrix3d R1 = image.Q().toRotationMatrix();
  const Eigen::Vector3d &t1 = image.T();
  const Eigen::Matrix3d &K = GetCamera(image.CamId()).K();
  const Eigen::Matrix3d Kinv = K.inverse();
  std::vector<int> point_index;
  int max_n_inliners = 0;
  Eigen::Vector3d best_norm;
  Eigen::Matrix4d H_camera;
  H_camera.block(0, 0, 3, 3) = R1.transpose();
  H_camera.block(3, 0, 1, 3).setConstant(0);
  H_camera.block(0, 3, 3, 1) = -R1.transpose() * t1;
  H_camera.block(3, 3, 1, 1).setConstant(1);

  for (size_t i = 0; i < points.size(); i++) {
    const Eigen::Vector2d &x1 = points[i];

    if (p3d_ids[i] == (uint64_t)-1) {
      continue;
    }

    if (polygon2d.PointInside(x1)) {
      point_index.push_back(i);
    }
  }
  /// Run RANSAC iterations
  for (size_t i = 0; i < niter; i++) {
    std::vector<int> sampled_index;
    int n_inliers = 0;
    PlaneEstimator plane_estimator;
    /// Randomly select minmum set of points to do the plane estimation
    std::sample(point_index.begin(), point_index.end(),
                std::back_inserter(sampled_index), 3,
                std::mt19937{std::random_device{}()});
    /// Get corresponding points first
    for (size_t j = 0; j < sampled_index.size(); j++) {
      const Eigen::Vector2d &x1 = points[sampled_index[j]];
      for (const auto &item : GetPoint3d(p3d_ids[sampled_index[j]]).Track()) {
        if (item.first == image.Id()) {
          continue;
        }
        Eigen::Matrix3d R2;
        Eigen::Vector3d t2;
        Eigen::Vector2d x2;
        std::tie(R2, t2, x2) = ProcessInfor(image, item, H_camera);
        plane_estimator.AddCorrespondence(Kinv * x1.homogeneous(),
                                          Kinv * x2.homogeneous(), R2, t2);
      }
    }
    /// Solve for the plane norm
    Eigen::Vector3d n = plane_estimator.Solve();
    /// Compute the homography and project the points to compara the differece
    for (size_t k = 0; k < point_index.size(); k++) {
      const Eigen::Vector2d &x1 = points[point_index[k]];
      for (const auto &item : GetPoint3d(p3d_ids[point_index[k]]).Track()) {
        if (item.first == image.Id()) {
          continue;
        }
        Eigen::Matrix3d R2;
        Eigen::Vector3d t2;
        Eigen::Vector2d x2;
        std::tie(R2, t2, x2) = ProcessInfor(image, item, H_camera);
        Eigen::Matrix3d H_points = K * (R2 - t2 * n.transpose()) * Kinv;
        Eigen::Vector3d x2_tf = H_points * x1.homogeneous();
        /// Consider points with error smaller than the threshold
        if ((x2_tf.hnormalized() - x2).norm() < thr)
          n_inliers++;
      }
    }
    /// Keep tracking the best norm
    if (n_inliers > max_n_inliners) {
      max_n_inliners = n_inliers;
      best_norm = n;
    }
  }
  /// Return the best norm
  return (best_norm.homogeneous().transpose() * H_camera.inverse())
      .hnormalized();
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Vector2d>
Reconstruction::ProcessInfor(const Image &image,
                             const std::pair<uint32_t, uint32_t> &item,
                             Eigen::Matrix4d H_camera) const {
  const Reconstruction::Image &image2 = GetImage(item.first);
  assert(image2.CamId() == image.CamId());
  Eigen::MatrixXd Rt2(3, 4);
  Rt2 << image2.Q().toRotationMatrix(), image2.T();
  Rt2 = Rt2 * H_camera;
  const Eigen::Matrix3d R2 = Rt2.block(0, 0, 3, 3);
  const Eigen::Vector3d &t2 = Rt2.block(0, 3, 3, 1);
  const Eigen::Vector2d &x2 = image2.Points()[item.second];
  return {R2, t2, x2};
}

// Project an image point onto a 3d plane
static Eigen::Vector3d ProjectPoint(const Eigen::Vector2d &point,
                                    const Eigen::Matrix3d &R,
                                    const Eigen::Vector3d &t,
                                    const Eigen::Matrix3d &Kinv,
                                    const Eigen::Vector3d &plane) {
  // Camera equations:
  // λx = K(RX + t)  [1]
  //
  // Plane equation:
  // nᵗX + 1 = 0  [2]
  //
  // Solving [1] for X gives
  // X = Rᵗ(λK⁻¹x - t)  [3]
  //
  // Find λ by inserting [3] into [2]
  // λ = (nᵗRᵗt - 1)/(nᵗRᵗK⁻¹x)  [4]
  Eigen::Vector3d x = point.homogeneous();
  double lambda = (plane.transpose() * R.transpose() * t - 1) /
                  (plane.transpose() * R.transpose() * Kinv * x);
  return R.transpose() * (lambda * Kinv * x - t);
}

Polygon3d Reconstruction::ProjectPolygon(const Polygon2d &polygon2d,
                                         const Image &image,
                                         const Eigen::Vector3d &plane) const {
  const Eigen::Matrix3d R = image.Q().toRotationMatrix();
  const Eigen::Vector3d &t = image.T();
  const Eigen::Matrix3d &K = GetCamera(image.CamId()).K();
  const Eigen::Matrix3d Kinv = K.inverse();
  std::vector<Eigen::Vector3d> points;

  for (const auto &p : polygon2d.Points()) {
    points.push_back(ProjectPoint(p, R, t, Kinv, plane));
  }

  return Polygon3d(points);
}

// Compute transformation from the 2d coordinate system in the plane to the
// global coordinate system
static Eigen::MatrixXd PlaneToWorldTrans(const Eigen::Vector3d &c,
                                         const Eigen::Vector3d &x,
                                         const Eigen::Vector3d &y) {

  Eigen::MatrixXd T(4, 3);
  // clang-format off
  T << x.x(), y.x(), c.x(),
       x.y(), y.y(), c.y(),
       x.z(), y.z(), c.z(),
           0,     0,     1;
  // clang-format on
  return T;
}

Eigen::Matrix3d
Reconstruction::ComputeHomography(const Polygon2d &polygon2d,
                                  const Image &image,
                                  const Eigen::Vector3d &plane) const {
  // Get camera parameters
  const auto R = image.Q().toRotationMatrix();
  const auto &t = image.T();
  const auto &K = GetCamera(image.CamId()).K();
  const auto Kinv = K.inverse();
  Eigen::MatrixXd Rt(3, 4);
  Rt << R, t;

  // Set up a coordinate system in the plane
  // The origin is the centroid of the 2d polygon projected onto the 3d plane
  const auto c = ProjectPoint(polygon2d.Centroid(), R, t, Kinv, plane);
  // The z axis is parallel to the plane normal, pointing away from the image
  Eigen::Vector3d z = -plane; // FIXME: Choose correct sign
  // The x and y axes are chosen arbitrarily but orthogonal to z
  assert(z.x() != 0 || z.y() != 0);
  Eigen::Vector3d x = Eigen::Vector3d(z.y(), -z.x(), 0).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();
  const auto T = PlaneToWorldTrans(c, x, y);

  // Compute the homography from plane to image
  const auto H = K * Rt * T;

  // Warp polygon to plane
  const auto polygon_warped = H.inverse() * polygon2d;

  // Adjust scale so that area remains constant
  double s = sqrt(fabs(polygon2d.Area() / polygon_warped.Area()));
  return K * Rt * PlaneToWorldTrans(c, x / s, y / s);
}

} // namespace ppr
