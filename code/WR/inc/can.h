#ifndef CAN_H
#define CAN_H

#include <iostream>

#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define pi 3.14159265359

enum msg_type {
  request_status = 0,
  command_throttle = 1,
  command_angle = 2
};

class actuator {
 public:
  actuator(std::string ifname);
  ssize_t sendThrottle(double val);
  ssize_t sendAngle(double val);
  ssize_t send(uint8_t* msg, msg_type id, uint8_t size);
  
 private:
  int sockFD;
};

#endif //CAN_H
