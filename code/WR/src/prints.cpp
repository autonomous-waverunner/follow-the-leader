#include "prints.h"

void printParams(std::ostream &s, config cfg) {
  s << "PARAMETER VALUES:" << std::endl;
  s << "  deltaT=" << cfg.deltaT;
  s << " dist_ref=" << cfg.dist_ref << std::endl;
  s << "  Gas:   ";
  s << " Kp=" << cfg.throttle.Kp << " Tf=" << cfg.throttle.Tf
    << " Kd=" << cfg.throttle.Kd << " Ki=" << cfg.throttle.Ki
    << " min=" << cfg.throttle.min << " max="<< cfg.throttle.max
    << " scaling=" << cfg.throttle.scaling
    << " lp=" << cfg.throttle.lp << std::endl;
  s << "  Nozzle:";
  s << " Kp=" << cfg.nozzle.Kp << " Tf=" << cfg.nozzle.Tf
    << " Kd=" << cfg.nozzle.Kd << " Ki=" << cfg.nozzle.Ki
    << " min=" << cfg.nozzle.min << " max="<< cfg.nozzle.max 
    << " scaling=" << cfg.nozzle.scaling
    << " lp=" << cfg.nozzle.lp<< std::endl;
}

void printState(std::ostream &s, double dt) {
  s << std::left << std::setw(10)  << dt
    << std::left << std::setw(12) << WRState.time
    << std::left << std::setw(9)  << WRState.position[0]
    << std::left << std::setw(9)  << WRState.position[1]
    << std::left << std::setw(12) << WRState.heading
    << WRState.velocity
    << "\r" << std::flush;
}

void printStateHeader(std::ostream &s) {
  s << std::endl
    << "STATE VALUES:" << std::endl
    << std::left << std::setw(10)  << "dt"
    << std::left << std::setw(12) << "UTC"
    << std::left << std::setw(9)  << "Lat"
    << std::left << std::setw(9)  << "Lon"
    << std::left << std::setw(12) << "Heading"
    << "Velocity"
    << std::endl << std::flush;
}

