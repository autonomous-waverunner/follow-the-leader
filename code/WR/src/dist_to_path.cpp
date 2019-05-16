#include "dist_to_path.h"

dist_to_path::dist_to_path(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d WR) {
  this->set(A, B, WR);
}

void dist_to_path::set(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d WR) {
  Eigen::Vector2d u = WR - A;

  v = B - A;
  b = (u.dot(v) / v.squaredNorm()) * v;
  a = b - u;

  double dist_b = b.norm();
  double dist_v = v.norm();

  dist_a_error = a.norm();
  dist_c = dist_v - dist_b;

  Eigen::Vector2d v_unit = v / v.norm();
  Eigen::Vector2d b_unit = b / b.norm();
  
  how_far = (dist_b / dist_v) * b_unit.dot(v_unit);
}
