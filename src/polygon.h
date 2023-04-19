#ifndef POLYGON_H_
#define POLYGON_H_

#include "geometry.h"
#include <Eigen/Dense>
#include <vector>

namespace ppr {

typedef std::tuple<int, int, int> Triangle;

template <typename T> class Polygon {
public:
  Polygon() {}
  Polygon(const std::vector<T> &points) : points_(points) {}

  const std::vector<T> &Points() const { return points_; }
  bool PointInside(const Eigen::Vector2d &point) const;
  double Area() const;
  std::vector<Triangle> Triangulate() const;

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

template <typename T> std::vector<Triangle> Polygon<T>::Triangulate() const {
  Vertex *head = new Vertex(points_[0], 0), *tail = head;
  for (size_t i = 1; i < points_.size(); i++) {
    Vertex *vert = new Vertex(points_[i], i);
    tail->next = vert;
    vert->prev = tail;
    tail = vert;
  }
  tail->next = head;
  head->prev = tail;

  std::vector<Triangle> triangles;
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
