#pragma once

#include <vector>

#include "math.h"

namespace magnus {
namespace geom {

bool IsParallel(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double eps = 1e-9);

bool IsNearlyEqual(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double eps = 1e-9);

}  // namespace geom
}  // namespace magnus
