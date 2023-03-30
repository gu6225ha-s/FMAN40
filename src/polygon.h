#ifndef POLYGON_H_
#define POLYGON_H_

#include <Eigen/Dense>
#include <vector>

namespace ppr {

template <typename T> class Polygon {
public:
  Polygon() {}
  Polygon(const std::vector<T> &points) : points_(points) {}

  bool PointInside(const Eigen::Vector2d &point);

private:
  std::vector<T> points_; // Points
};

template <typename T>
bool Polygon<T>::PointInside(const Eigen::Vector2d &point) {
  // https://wrfranklin.org/Research/Short_Notes/pnpoly.html
  bool inside;
  for (int i = 0, j = points_.size() - 1; i < points_.size(); j = i++) {
    if ((points_[i].y() > point.y()) != (points_[j].y() > point.y()) &&
        (point.x() < (points_[j].x() - points_[i].x()) *
                             (point.y() - points_[i].y()) /
                             (points_[j].y() - points_[i].y()) +
                         points_[i].x())) {
      inside = !inside;
    }
  }
  return inside;
}

typedef Polygon<Eigen::Vector2d> Polygon2d;
typedef Polygon<Eigen::Vector3d> Polygon3d;

} // namespace ppr

#endif /* POLYGON_H_ */
