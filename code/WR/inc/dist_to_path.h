#ifndef DIST_TO_PATH_H
#define DIST_TO_PATH_H

#include <Eigen/Dense>

/*
 * Description:    calculates the distance from one point to one line between
 *                 two other points. 
 %
 * Inputs:
 *   A, B          the points with the path between them.
 *   WR            the point which distance to the A->B path you want to calculate.
 %
 * Outputs:
 *   dist_a_error  distance to the path A->B from the WR.
 *   howFar        how far the WR is on the line A->B.
 *   norm(v)       distance between A->B, v is the vector between A->B.
 *   u             the vector between A->WR.
 *   b             the vector from A -> the projected point of WR on the A->B line.
 *   dist_c        distance between projected point of WR on the A->B line to B.
 */
struct dist_to_path {
  double dist_a_error;
  double how_far;
  double dist_c;
  Eigen::Vector2d v;
  Eigen::Vector2d a;
  Eigen::Vector2d b;

  dist_to_path(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d WR);
  void set(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d WR);
};


#endif //DIST_TO_PATH_H
