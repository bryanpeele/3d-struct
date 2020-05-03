#pragma once

#include <vector>

#include "math.h"

namespace magnus {
namespace geom {

template <typename T, int N>
class Point {
 public:
  explicit Point(Eigen::Matrix<T, N, 1> coordinate) : coordinate_(coordinate) {}

  const Eigen::Matrix<T, N, 1>& coordinate() const { return coordinate_; }

  // TODO(bpeele) error check dimension
  T coordinate(size_t dimension) const { return coordinate_(dimension); }

  T x() const { return coordinate_(0); }
  T y() const { return coordinate_(1); }
  T z() const { return (N > 2) ? coordinate_(2) : std::numeric_limits<T>::signaling_NaN(); }

  // TODO(bpeele) options to scale axes individually
  void scale(T scale_factor) { coordinate_ *= scale_factor; }

  void translate(Eigen::Matrix<T, N, 1> translation_vector) { coordinate_ += translation_vector; }

  void rotate(T angle) {
    if (N == 2) {
      Eigen::Rotation2D<T> rotation(angle);
      coordinate_ = rotation * coordinate_;
    } else {
      // Non-1D rotations not yet implemented
      assert(1 == 2);
    }
  }

 private:
  Eigen::Matrix<T, N, 1> coordinate_;
};

using Point2d = Point<double, 2>;
using Point2f = Point<float, 2>;
using Point3d = Point<double, 3>;
using Point3f = Point<float, 3>;

}  // namespace geom
}  // namespace magnus
