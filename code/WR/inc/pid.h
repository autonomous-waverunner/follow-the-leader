#ifndef PID_H
#define PID_H

class pid {
 public:
  pid(double Kp, double Tf, double Kd, double Ki,
      double max, double min, double scaling);
  double step(double e, double dt);
  void anti_windup(double e);

  double _min, _max;
 private:
  //double constrain(double u);
  double p = 0, i = 0, d = 0, e_old = 0,
    Kp, Tf, Kd, Ki, scaling;
};

#endif //PID_H
