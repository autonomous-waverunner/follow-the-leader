#ifndef PATH_ALG_H
#define PATH_ALG_H

#include <deque>
#include <limits>
#include <memory>

#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <proj_api.h>
#include <LowPassFilter.hpp>

#include "dist_to_path.h"
#include "pid.h"

#define SMALL_VELOCITY_SCALING 0.2

#define EX_COORDINATE_TRANSFORM 1

const double pi =  3.14159265359;

//#define DEV  // Uncomment to turn on debugging messages
#ifdef DEV
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  s_path_alg
#include "simstruc.h"
#endif // DEV

struct pid_v {
  double Kp, Tf, Kd, Ki, min, max, lp, scaling;
};

struct state_params {
  Eigen::Vector2d wr_position;
  double wr_heading;
  double wr_velocity;
  Eigen::Vector2d target;
  double desired_rotation;
  double distance_to_lc;
  double distance_reference;
  Eigen::Vector2d pred_loc;
  int alive;
  double dist_a_error;
};

struct result {
  double gas;
  double nozzle_angle;
};

struct config {
  pid_v throttle, nozzle;
  double deltaT, dist_ref;
};

class path_track {
 public:
  // Constructor
  path_track(double deltaT, double dist_ref, pid_v throttle, pid_v nozzle);

  // Coordinates queue related methods
  //void push_coord(std::pair<double, double> coord);
  void push_coord(Eigen::Vector2d coord);
  void clear_coords();
  Eigen::Vector2d get_last_coord();
  std::size_t coords_queue_size();

  // Functions handling the calculation of the next state
  void tick(Eigen::Vector2d wr_position, double wr_heading, double wr_velocity, double dt);
  int calculate_rotation();
  void calculate_dist_to_lc(int GPSat);

  void calculate_gas(double dt);
  void calculate_nozzle_angle(double dt);

  // Function used during development / testing
  state_params get_state();

  // Getter for output signal
  result get_results();
  result res;

 private:
  void DIE();
  std::deque<Eigen::Vector2d> coords;
  double deltaT;

  state_params state;
  pid pid_gas, pid_nozzle_angle;

  //Create a low pass filter with 1 * 2 * pi Hz cuttoff freqency. DetltaTime for each cycle is unknown and will vary.
  LowPassFilter lpf_throttle;
  LowPassFilter lpf_nozzle;
  double dist_ref;
};

#endif //PATH_ALG_H
