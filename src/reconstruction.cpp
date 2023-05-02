#include "reconstruction.h"
#include "util.h"

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

Eigen::Vector3d Reconstruction::EstimatePlane(const Polygon2d &polygon2d,
                                              const Image &image) const {
  const std::vector<Eigen::Vector2d> &points = image.Points();
  const std::vector<uint64_t> &p3d_ids = image.P3dIds();
  const Eigen::Matrix3d R1 = image.Q().toRotationMatrix();
  const Eigen::Vector3d &t1 = image.T();
  const Eigen::Matrix3d &K = GetCamera(image.CamId()).K();
  const Eigen::Matrix3d Kinv = K.inverse();
  PlaneEstimator plane_estimator;

  for (size_t i = 0; i < points.size(); i++) {
    const Eigen::Vector2d &x1 = points[i];

    if (p3d_ids[i] == -1) {
      continue;
    }

    if (!polygon2d.PointInside(x1)) {
      continue;
    }

    for (const auto &item : GetPoint3d(p3d_ids[i]).Track()) {
      if (item.first == image.Id()) {
        continue;
      }

      const Image &image2 = GetImage(item.first);
      assert(image2.CamId() == image.CamId());
      const Eigen::Matrix3d R2 = image2.Q().toRotationMatrix();
      const Eigen::Vector3d &t2 = image2.T();
      const Eigen::Vector2d &x2 = image2.Points()[item.second];
      plane_estimator.AddCorrespondence(
          Kinv * Eigen::Vector3d(x1.x(), x1.y(), 1),
          Kinv * Eigen::Vector3d(x2.x(), x2.y(), 1), R2 * R1.transpose(),
          -R2 * R1.transpose() * t1 + t2);
    }
  }

  const Eigen::Vector3d n = plane_estimator.Solve();
  return n.transpose() * R1 / (n.transpose() * t1 + 1);
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
  Eigen::Vector3d x(point.x(), point.y(), 1);
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
