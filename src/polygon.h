#ifndef POLYGON_H_
#define POLYGON_H_

#include <Eigen/Dense>
#include <vector>

namespace ppr {

template <typename T> class Polygon {
public:
  Polygon() {}
  Polygon(const std::vector<T> &points) : points_(points) {}

  const std::vector<T> &Points() const { return points_; }
  bool PointInside(const Eigen::Vector2d &point) const;
  double Area() const;

private:
  std::vector<T> points_; // Points
};

template <typename T>
bool Polygon<T>::PointInside(const Eigen::Vector2d &point) const {
  // https://wrfranklin.org/Research/Short_Notes/pnpoly.html
  bool inside = false;
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

template <typename T> double Polygon<T>::Area() const {
  double area = 0.0;
  for (size_t i = 0; i < points_.size(); i++) {
    size_t j = (i + 1) % points_.size();
    area += points_[i].x() * points_[j].y() - points_[j].x() * points_[i].y();
  }
  return 0.5 * area;
}

typedef Polygon<Eigen::Vector2d> Polygon2d;
typedef Polygon<Eigen::Vector3d> Polygon3d;

} // namespace ppr

#endif /* POLYGON_H_ */
