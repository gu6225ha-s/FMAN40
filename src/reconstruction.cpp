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

Polygon3d Reconstruction::ProjectPolygon(const Polygon2d &polygon2d,
                                         const Image &image,
                                         const Eigen::Vector3d &plane) const {
  const Eigen::Matrix3d R = image.Q().toRotationMatrix();
  const Eigen::Vector3d &t = image.T();
  const Eigen::Matrix3d &K = GetCamera(image.CamId()).K();
  const Eigen::Matrix3d Kinv = K.inverse();
  std::vector<Eigen::Vector3d> points;

  for (const auto &p : polygon2d.Points()) {
    Eigen::Vector3d x(p.x(), p.y(), 1);
    double lambda = (plane.transpose() * R.transpose() * t - 1) /
                    (plane.transpose() * R.transpose() * Kinv * x);
    points.push_back(R.transpose() * (lambda * Kinv * x - t));
  }

  return Polygon3d(points);
}

} // namespace ppr
