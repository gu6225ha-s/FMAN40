#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <Eigen/Dense>

namespace ppr {

inline double TriangleArea(const Eigen::VectorXd &a, const Eigen::VectorXd &b,
                           const Eigen::VectorXd &c) {
  return 0.5 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) +
                c.x() * (a.y() - b.y()));
}

inline bool PointInTriangle(const Eigen::VectorXd &a, const Eigen::VectorXd &b,
                            const Eigen::VectorXd &c,
                            const Eigen::VectorXd &p) {
  double a1 = TriangleArea(p, a, b);
  double a2 = TriangleArea(p, b, c);
  double a3 = TriangleArea(p, c, a);
  bool neg = a1 < 0 || a2 < 0 || a3 < 0;
  bool pos = a1 > 0 || a2 > 0 || a3 > 0;
  return !(neg && pos);
}

} // namespace ppr

#endif /* GEOMETRY_H_ */
