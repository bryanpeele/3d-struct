#include "math_util.h"

namespace magnus {
namespace geom {

bool IsParallel(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double eps) {
  return (std::abs(1.0 - std::abs(a.dot(b))) < eps);
}

bool IsNearlyEqual(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double eps) {
  return ((a - b).norm() < eps);
}

}  // namespace geom
}  // namespace magnus