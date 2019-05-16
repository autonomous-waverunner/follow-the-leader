#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <thread>

#include <libconfig.h++>

#include "path_alg.h"
#include "LCPositions.h"
#include "WRPosition.h"
#include "prints.h"
#include "can.h"

// Induced delay in the main loop (µs)
#define EXTRA_DELAY 20 * 1000

config initFromConfig(std::string fname) {
  libconfig::Config cfg;
  double deltaT;
  double dist_ref;
  pid_v pid_throttle, pid_nozzle;
  
  try {
    cfg.readFile(fname.c_str());
  } catch(const libconfig::FileIOException &fioex) {
    std::cerr << "I/O error while reading configuration file parameters.cfg" << std::endl;
    exit(EXIT_FAILURE);
  } catch(const libconfig::ParseException &pex) {
    std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
    exit(EXIT_FAILURE);
  }

  try {
    const libconfig::Setting& root = cfg.getRoot();
    const libconfig::Setting& gas = root["gas"];
    const libconfig::Setting& nozzle = root["nozzle"];
    
    deltaT = cfg.lookup("deltaT");
    dist_ref = cfg.lookup("dist_ref");
    gas.lookupValue("Kp", pid_throttle.Kp);
    gas.lookupValue("Tf", pid_throttle.Tf);
    gas.lookupValue("Kd", pid_throttle.Kd);
    gas.lookupValue("Ki", pid_throttle.Ki);
    gas.lookupValue("min", pid_throttle.min);
    gas.lookupValue("max", pid_throttle.max);
    gas.lookupValue("lp", pid_throttle.lp);
    gas.lookupValue("scaling", pid_throttle.scaling);
    nozzle.lookupValue("Kp", pid_nozzle.Kp);
    nozzle.lookupValue("Tf", pid_nozzle.Tf);
    nozzle.lookupValue("Kd", pid_nozzle.Kd);
    nozzle.lookupValue("Ki", pid_nozzle.Ki);
    nozzle.lookupValue("min", pid_nozzle.min);
    nozzle.lookupValue("max", pid_nozzle.max);
    nozzle.lookupValue("lp", pid_nozzle.lp);
    nozzle.lookupValue("scaling", pid_nozzle.scaling);
    
  } catch (const libconfig::SettingNotFoundException &nfex) {
    std::cerr << "Could not find setting." << std::endl;
    exit(1);
  }

  return {pid_throttle, pid_nozzle, deltaT, dist_ref};
}

uint64_t micros() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}


#include <time.h>
int main(int argc, char* argv[]) {
  config cfg = initFromConfig("parameters.cfg");
  //printParams(std::cerr, cfg);

  time_t now = time(0);
  struct tm tstruct;
  char log_file[80];
  tstruct = *localtime(&now);
  strftime(log_file, sizeof(log_file), "%Y-%m-%d.%X.log", &tstruct);
  std::ofstream log;
  log.open(log_file, std::ofstream::out);
  
  /*
  // BEGIN CAN TESTING
  #include <errno.h>
  for (uint8_t i = 0; i < 16; i++) {
    std::cerr << "Sending " << std::to_string(i) << "  ";
    ssize_t n = act.send(&i, 0x32, 1);
    if (n == -1)
      std::cerr << "fail, errno=" << std::to_string(errno);
    else
      std::cerr << std::to_string(n) << " bytes sent" << std::endl;
    sleep(1);
  }
  exit(0);

  for (int i = -24; i < 25; i += 2) {
    act.sendAngle(i*pi/180);
    usleep(100000);
    }

  for (int i = 0; i < 13; i++) {
    act.sendThrottle(i);
    sleep(1);
  }

  // END CAN TESTING
  */
  
  int radioFD = initRadio("/dev/tty_ftl_radio");
  std::thread threadLCPos (getLCPositions, radioFD);
  
  // UART is mapped to /dev/ttyTHS0
  std::thread threadWRPos (getWRPosition, "/dev/ttyTHS0");

  actuator act("can0");
  /*
  while (1) {
  for (int i = -24; i < 25; i += 1) {
    act.sendAngle(i*pi/180);
    usleep(100000);
  }
  for (int i = 24; i > -25; i -= 1) {
    act.sendAngle(i*pi/180);
    usleep(100000);
  }
  for (int i = 0; i <= 100; i++) {
    act.sendThrottle(i);
    usleep(100000);
  }
  for (int i = 100; i >= 0; i--) {
    act.sendThrottle(i);
    usleep(100000);
  }
  }
  */

  while (true) {
    uint64_t then = micros();
    path_track p = path_track(cfg.deltaT, cfg.dist_ref, cfg.throttle, cfg.nozzle);
    pp = &p;
    //printStateHeader(std::cerr);
    while (state == COORDINATES) {
      // Keep track of time
      uint64_t now = micros();
      double dt = ((double) now - (double) then) / 1000000.0;
      then = now;

      // Advance one time step in The Algorithm
      p.tick(WRState.position, WRState.heading, WRState.velocity, dt);

      // Get and log WR state parameters
      state_params state = p.get_state();
      log << state.dist_a_error << ", "
	  << state.desired_rotation << ", "
	  << state.distance_to_lc
	  << std::endl;

      // Actuate, please!
      result res = p.get_results();
      act.sendThrottle(res.gas);
      act.sendAngle(res.nozzle_angle);

      // Do some pretty-printing if attached terminal
      //printState(std::cerr, dt);

      // Slow everything down a little
      usleep(EXTRA_DELAY);
    }

    while (state == PARAMS) {
      pp = nullptr;
      cfg = getUpdatedConfig();
      sleep(1);
    }
    
  }

  threadLCPos.join();
  threadWRPos.join();
  
  exit(0);
}
