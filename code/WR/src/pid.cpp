
/* Pseudo-kod for a PID-controller (taken from Balanduino project)
 * P = c0 * e
 * D = c1 * D + c2 * (e - eold)
 * u = P + I + D // Bestäm totala styrsignalen
 * daout("u", u) // Skriv styrsignalen till DA-omv.
 * I = I + c3 * e // Uppdatera integraldelen
 * eold = e
 */

#include "pid.h"

pid::pid(double Kp, double Tf, double Kd, double Ki,
	 double min, double max, double scaling)
  : _min(min), _max(max), Kp(Kp), Tf(Tf), Kd(Kd), Ki(Ki), scaling(scaling) {}

double pid::step(double e, double dt) {
  
  p = Kp * e;
  //d = Kd * (e - e_old ) / dt;
  d = (Tf * d + Kd * (e - e_old)) / (Tf + dt);
  double u = (p + i + d) * scaling;
	//SCALING FUNGERAR EJ! om man byter scaling till 1 här fungerar det
  i = i + Ki * dt * e;

  //ssPrintf("[PID p=%f, i=%f, i=%f, u=%f, ", p, i, d, u);
  e_old = e;
  /*
  if (u < _min)
    u = _min;
  else if (u > _max)
    u = _max;
*/
  //ssPrintf("u_c=%f] ", u);
  
  return u;
  
  //return e;
}

void pid::anti_windup(double e) {
  if(e < 0)
    i = 0;
  else 
    i = (i < 0) ? i : 0;
}
/*
double pid::constrain(double u) {
  if (u < _min) return _min;
  if (u > _max) return _max;
  return u;
}
*/
