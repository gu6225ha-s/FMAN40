#ifndef POLYGON_H_
#define POLYGON_H_

#include "geometry.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace ppr {

/// \brief Template polygon class.
///
/// A valid polygon should have at least three points.
template <typename T> class Polygon {
public:
  /// \brief Create new, empty polygon.
  Polygon() {}

  /// \brief Create new polygon from vector of points.
  /// \param points vector of points
  Polygon(const std::vector<T> &points) : points_(points) {}

  /// \brief Move constructor.
  ///
  /// The new polygon takes ownership of the vector of \p points.
  /// \param points vector of points
  Polygon(std::vector<T> &&points) : points_(std::move(points)) {}

  /// \brief Multiplication operator.
  /// \param H a homography / projective transformation
  /// \return A new polygon with \p H applied to each point.
  Polygon operator*(const Eigen::Matrix3d &H) const;

  /// \brief Get the points of the polygon.
  /// \return The vector of points.
  const std::vector<T> &Points() const { return points_; }

  /// \brief Check if a point is inside the polygon.
  ///
  /// The check is done in the xy plane.
  /// \param point query point
  /// \return True if the point is inside the polygon and false otherwise.
  bool PointInside(const Eigen::Vector2d &point) const;

  /// \brief Compute the signed area of the polygon.
  ///
  /// The area is computed in the xy plane.
  /// \return The signed area of the polygon.
  double Area() const;

  /// \brief Reverse the polygon.
  void Reverse();

  /// \brief Compute the centroid of the polygon.
  /// \return The centroid of the polygon.
  T Centroid() const;

  /// \brief Compute the bounds of the polygon.
  /// \param[out] min the minimum coordinates in xyz
  /// \param[out] max the maximum coordinates in xyz
  void Bounds(T &min, T &max) const;

  /// \brief Triangulate the polygon.
  ///
  /// The triangulation is done using a naive ear clipping algorithm. It assumes
  /// that the polygon is simple (no self-intersections, no holes).
  /// \return A vector of triangles.
  std::vector<std::tuple<int, int, int>> Triangulate() const;

private:
  std::vector<T> points_; // Points

  // Doubly linked vertex struct used for triangulation
  typedef struct Vertex {
    Vertex(const Eigen::VectorXd &point, int index)
        : point(point.x(), point.y()), index(index) {}

    void Delete() {
      if (prev) {
        prev->next = next;
      }
      if (next) {
        next->prev = prev;
      }
      prev = next = nullptr;
      delete this;
    }

    Vertex *GetEar(bool clockwise) {
      Vertex *vert = this;
      do {
        if (vert->IsEar(clockwise)) {
          return vert;
        }
        vert = vert->next;
      } while (vert != this);
      return nullptr;
    }

    bool IsEar(bool clockwise) const {
      double area = TriangleArea(prev->point, point, next->point);
      bool convex = (clockwise && area <= 0) || (!clockwise && area >= 0);
      if (!convex) {
        return false;
      }
      Vertex *vert = next->next;
      while (vert != prev) {
        if (PointInTriangle(prev->point, point, next->point, vert->point)) {
          return false;
        }
        vert = vert->next;
      }
      return true;
    }

    Eigen::Vector2d point;
    int index;
    Vertex *prev, *next;
  } Vertex;
};

template <>
inline Polygon<Eigen::Vector2d>
Polygon<Eigen::Vector2d>::operator*(const Eigen::Matrix3d &H) const {
  std::vector<Eigen::Vector2d> points;

  points.reserve(Points().size());

  for (const auto &p : Points()) {
    Eigen::Vector3d x = H * Eigen::Vector3d(p.x(), p.y(), 1);
    points.emplace_back(x.x() / x.z(), x.y() / x.z());
  }

  return Polygon<Eigen::Vector2d>(std::move(points));
}

inline Polygon<Eigen::Vector2d>
operator*(const Eigen::Matrix3d &H, const Polygon<Eigen::Vector2d> &polygon) {
  return polygon * H;
}

template <typename T>
bool Polygon<T>::PointInside(const Eigen::Vector2d &point) const {
  // https://wrfranklin.org/Research/Short_Notes/pnpoly.html
  bool inside = false;
  for (size_t i = 0, j = points_.size() - 1; i < points_.size(); j = i++) {
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

template <typename T> void Polygon<T>::Reverse() {
  std::reverse(points_.begin(), points_.end());
}

template <typename T> T Polygon<T>::Centroid() const {
  T c;
  c.setZero();
  for (const auto &p : points_) {
    c += p;
  }
  c /= points_.size();
  return c;
}

template <typename T> void Polygon<T>::Bounds(T &min, T &max) const {
  min = max = points_[0];
  for (const auto &p : points_) {
    min = min.cwiseMin(p);
    max = max.cwiseMax(p);
  }
}

template <typename T>
std::vector<std::tuple<int, int, int>> Polygon<T>::Triangulate() const {
  Vertex *head = new Vertex(points_[0], 0), *tail = head;
  for (size_t i = 1; i < points_.size(); i++) {
    Vertex *vert = new Vertex(points_[i], i);
    tail->next = vert;
    vert->prev = tail;
    tail = vert;
  }
  tail->next = head;
  head->prev = tail;

  std::vector<std::tuple<int, int, int>> triangles;
  size_t N = points_.size();
  Vertex *vert = head;
  bool clockwise = Area() < 0;
  while (N >= 3) {
    vert = vert->GetEar(clockwise);
    assert(vert);
    triangles.emplace_back(vert->prev->index, vert->index, vert->next->index);
    vert = vert->next;
    vert->prev->Delete();
    N--;
  }

  vert->next->Delete();
  vert->Delete();

  return triangles;
}

typedef Polygon<Eigen::Vector2d> Polygon2d;
typedef Polygon<Eigen::Vector3d> Polygon3d;

} // namespace ppr

#endif /* POLYGON_H_ */
