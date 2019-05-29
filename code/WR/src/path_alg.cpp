#include "path_alg.h"
#define pi 3.14159265359

double constrainAngle(double x){
  /*
  ssPrintf("x: %f" ,x);
   x = fmod(x + pi,2*pi);
   ssPrintf("x1: %f" ,x);
    if (x < 0){
        x += 2*pi;
    }
    ssPrintf("x2: %f",x-pi);
  return x - pi;
  */
  return remainder(x,pi);
}

path_track::path_track(double deltaT, double dist_ref, pid_v throttle, pid_v nozzle)
  : deltaT(deltaT),
    pid_gas(throttle.Kp, throttle.Tf, throttle.Kd, throttle.Ki, throttle.max, throttle.min, throttle.scaling),
    pid_nozzle_angle(nozzle.Kp, nozzle.Tf, nozzle.Kd, nozzle.Ki, nozzle.max, nozzle.min, nozzle.scaling),
    dist_ref(dist_ref)
{
  res.gas = 0;
  res.nozzle_angle = 0;
  state.alive = 1;
  lpf_throttle = LowPassFilter (throttle.lp);
  lpf_nozzle = LowPassFilter (nozzle.lp);
  // ssPrintf("GAS    Kp=%f Kd=%f Ki=%f Max=%f Min=%f\n", g_Kp, g_Kd, g_Ki, g_max, g_min);
  //ssPrintf("NOZZLE Kp=%f Kd=%f Ki=%f Max=%f Min=%f", n_Kp, n_Kd, n_Ki, n_max, n_min);
}

void path_track::push_coord(Eigen::Vector2d coord) {
  projPJ pj_utm, pj_latlong;
  double x, y;
  
  if (!(pj_utm = pj_init_plus("+proj=utm +zone=32")) )
    throw EX_COORDINATE_TRANSFORM;
  if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84")) )
    throw EX_COORDINATE_TRANSFORM;

  x = coord[0] * DEG_TO_RAD;
  y = coord[1] * DEG_TO_RAD;
  if (pj_transform(pj_latlong, pj_utm, 1, 1, &x, &y, NULL ))
    throw EX_COORDINATE_TRANSFORM;

  coords.push_back(Eigen::Vector2d(x, y));
}

void path_track::clear_coords() {
  coords.clear();
}

std::size_t path_track::coords_queue_size() {
  return coords.size();
}

Eigen::Vector2d path_track::get_last_coord() {
  return coords.back();
}

result path_track::get_results() {
  return res;
}

state_params path_track::get_state() {
  return state;
}

void path_track::tick(Eigen::Vector2d wr_position, double wr_heading, double wr_velocity, double dt) {
  state.wr_position = wr_position;
  //state.wr_heading = constrainAngle(wr_heading);
  state.wr_heading = wr_heading;
  state.wr_velocity = wr_velocity;

  if (coords.size() < 2 || state.alive == 0) {
    state.desired_rotation = state.wr_heading;
    state.target = wr_position;//Eigen::Vector2d(0,0);
    state.distance_reference = 10000000;
    //    state.distance_to_lc = 0;
  } else {
    state.distance_reference = dist_ref;  // Should be a function of velocity       TODO
    int GPSat = calculate_rotation();
    if (GPSat == -1)
      return;
    calculate_dist_to_lc(GPSat);
  }

  calculate_gas(dt);
  calculate_nozzle_angle(dt);
}

void path_track::DIE() {
  //res.gas = 0;
  //res.nozzle_angle = 0;
  state.alive = 0;
  std::cerr << "DIE" << std::endl;
}

/* 
 * Description: 
 * Calculates the rotation needed from the WR to reach the path (X) using
 * a look-ahead method.
 */
int path_track::calculate_rotation() {
  const Eigen::Vector2d dir_start(cos(state.wr_heading), sin(state.wr_heading));
  const double velocity_multiple = (state.wr_velocity != 0)
    ? (state.wr_velocity * deltaT)
    : SMALL_VELOCITY_SCALING;
  
  const Eigen::Vector2d velocity_vector = (dir_start / dir_start.norm()) * velocity_multiple;
  const Eigen::Vector2d predict_location = state.wr_position + velocity_vector;
  state.pred_loc = predict_location;
  Eigen::Vector2d A, B;

  // Loop through the X matrix (our path) to see which points the WR are
  // closest to at this moment.
  //double min_a = std::numeric_limits<double>::infinity();
  unsigned int j = 0;
  for (std::size_t i = 0; i < coords.size() - 1; i++) {
    // Handle points in the same place                                      TODO
    /*	if (coords[i] == coords[i + 1]){
		// Pop either i or i + 1, Dosn't this get hard with a queue??
		continue;
		}*/
    // Calculating how_far on current distance, distance to path.
    dist_to_path dp(coords.at(i), coords.at(i + 1), predict_location);
    dist_to_path dp2(coords.at(i), coords.at(i + 1), state.wr_position);
    //ssPrintf("\n  [dp.how_far=%f dp2.how_far=%f] ", dp.how_far, dp2.how_far);
    
    state.dist_a_error = dp2.dist_a_error;
    
    if (dp2.how_far > 1) {
      // ssPrintf("\n  [pop coords[%d] dp.how_far=%f dp2.how_far=%f] ",
      //       i, dp.how_far, dp2.how_far);
      coords.pop_front();
      i--;
      continue;
    }
    /*
    else if (dp2.how_far < 0) {// && dp2.how_far < 0) {
      state.target = coords[i];
      break;
    }
    */
    else if (dp2.how_far > dp.how_far) {
      /*
      // ssPrintf("\n  [pop coords[%d] dp.how_far=%f dp2.how_far=%f] ",
	       i, dp.how_far, dp2.how_far);
      coords.pop_front();
      i--;
      continue;
      */
      DIE();
      return -1;
    }
    
    else {
      // Save the closest points to the WR on the path.
      A = coords.at(i);
      B = coords.at(i + 1); 
      j = i;
      
      dist_to_path dp3(A, B, predict_location);
      state.target = (A + dp3.b) + (state.wr_velocity * deltaT * (dp3.v/dp3.v.norm()));
      break;
    }

    /*
    if (dp.how_far < 0) {
      if (dp2.how_far < 0) {
	state.target = coords[i];
	break;
      } else {
	// ssPrintf("[pop! dp2.how_far = %f] ", dp2.how_far);
	coords.pop_front();
	continue;
      }
    }
    else if (dp.how_far <= 1) {
      if (dp.a.norm() < min_a) {
	// Save the closest points to the WR on the path.
	min_a = dp.a.norm();
	A = coords.at(i);
	B = coords.at(i + 1);
	j = i;
	dist_to_path dp3(A, B, predict_location);
	state.target = A + dp3.b + velocity_vector.cwiseProduct(dp3.v / dp3.v.norm()) * deltaT;
	break;
      }
    } else {
      // ssPrintf("[pop! dp.how_far = %f] ", dp.how_far);
      coords.pop_front();
    }
    */
  }
   
    
  // Calculating distance to path and distance b = distance A -> predictLoc
  dist_to_path dp(A, B, predict_location);

  // First target point calculated with our current speed and time interval +
  // the current point + the "b" distance. 
  //state.target = A + dp.b + velocity_vector.cwiseProduct(dp.v / dp.v.norm()) * deltaT;

  // Calculate howFar and distance c = distance B -> target
  // While the target point is to far ahead from our current PathPoints
  // keep updating the distance left.
  // ->  Handle out of bounds exception                                     TODO
  
  /*
  for (auto it = coords.cbegin();
       (coords.size() > 1) && (dp.set(*it, *(it + 1), state.target), dp.how_far > 1);
       it++, i++) {
       state.target = *it + abs(dp.dist_c) * (dp.v / dp.v.norm());
       }*/
  if (coords.size() > 1)
    dp.set(coords.at(j), coords.at(j + 1), state.target);
  
  for (std::size_t i = j; (i + 2) < coords.size(); i++) {
    //// ssPrintf("target How_far %f", dp.how_far);
    A = coords.at(i+1);
    B = coords.at(i+2);
    if (dp.how_far > 1) {
      j++;
      double s = abs(dp.dist_c);
      dp.set(A, B, state.target);
      state.target = A + s * (dp.v / dp.v.norm());
    }
  }
  
  if (j >= coords.size() - 1) {
    state.target = B;
  }

  // Calculate the rotation difference needed to reach the target location
  // from our current direction.
  const Eigen::Vector2d wr_to_target = state.target - state.wr_position;
  //const double deg_wr = atan2(velocity_vector[1], velocity_vector[0]);
  const double deg_wr_to_target = remainder(atan2(wr_to_target[1], wr_to_target[0]),pi);

  if(state.wr_heading > deg_wr_to_target){
    //ssPrintf("körs");
    state.desired_rotation = state.wr_heading - deg_wr_to_target;
  }
  else
    state.desired_rotation = state.wr_heading - deg_wr_to_target;

	//test
//test_angle = acos(wr_to_target.dot(velocity_vector)/(wr_to_target.norm()*velocity_vector.norm()));
//ssPrintf("first angle %f, second angle %f", state.desired_rotation, acos(wr_to_target.dot(velocity_vector)/(wr_to_target.norm()*velocity_vector.norm())));

  return j;
}

void path_track::calculate_dist_to_lc(int GPSat) {
  // We need to check this in case the coords queue has shrinked in the previous
  // call to calculate_rotation. But what should we do in that case?
  if (GPSat + 1 >= coords.size()) {
    state.distance_to_lc = (coords.at(coords.size() -1) - state.wr_position).norm();
    return;
  }
  Eigen::Vector2d A = coords.at(GPSat + 1);
  double target_to_next = (A - state.target).norm();
  double dist_to_target = (state.target - state.wr_position).norm();
  state.distance_to_lc = dist_to_target + target_to_next;
  for (std::size_t i = GPSat + 1; i < coords.size() - 1; i++) {
    A = coords.at(i + 1) - coords.at(i);
    state.distance_to_lc += A.norm();
  }
}

void path_track::calculate_gas(double dt) {
  const double distance_error = state.distance_to_lc - state.distance_reference;
  //ssPrintf("\n  [g_error %f] ", distance_error);

  double _gas = pid_gas.step(distance_error, dt);
    if (_gas < pid_gas._min)
    _gas = pid_gas._min;
  else if (_gas > pid_gas._max)
    _gas = pid_gas._max;
  res.gas = lpf_throttle.update(_gas, dt);
  pid_gas.anti_windup(distance_error);
}

void path_track::calculate_nozzle_angle(double dt) {
  const double rotation_error = (std::isnan(state.desired_rotation))
    ? 0 : state.desired_rotation;// - state.wr_heading;
  //ssPrintf("\n  [r_error %f] ", rotation_error);

  double tau = pid_nozzle_angle.step(rotation_error, dt);

  double gas;
  if (res.gas == 0) {
    gas = 0.001;
  }
  else {
    gas = res.gas;
  }

  tau = tau/gas;
  if (tau < pid_nozzle_angle._min)
    tau = pid_nozzle_angle._min;
  else if (tau > pid_nozzle_angle._max)
    tau = pid_nozzle_angle._max;
  res.nozzle_angle = lpf_nozzle.update(tau,dt);
}
