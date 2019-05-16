#ifndef WRPOSITION_H
#define WRPOSITION_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include <sbgEComLib.h>

struct wr_state {
  uint32 time;
  Eigen::Vector2d position;
  double heading;
  float velocity;
};

extern struct wr_state WRState;

void getWRPosition(std::string dev);

#endif // WRPOSITION_H
