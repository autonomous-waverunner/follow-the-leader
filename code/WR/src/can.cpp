#include <exception>

#include "can.h"

actuator::actuator(std::string ifname) {
  struct sockaddr_can addr;
  struct ifreq ifr;
  
  if ((sockFD = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    throw std::runtime_error("Could not open CAN socket");
  }

  strcpy(ifr.ifr_name, ifname.c_str());
  ioctl(sockFD, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sockFD, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    throw std::runtime_error("Could not bind CAN socket");
  }
}

ssize_t actuator::sendThrottle(double val) {
  uint8_t data[4];
  uint16_t throttle = (uint16_t) val * 100;

  data[1] = *((uint8_t*) &throttle);
  data[0] = *(((uint8_t*) &throttle) + 1);
  data[2] = 0;
  data[3] = 0;
  
  return send(data, command_throttle, 4);
}

ssize_t actuator::sendAngle(double val) {
  uint8_t data[2];
  double val_deg = val * 180 / pi;
  int16_t nozzle = (int16_t) val_deg * 1000;

  data[1] = *((int8_t*) &nozzle);
  data[0] = *(((int8_t*) &nozzle) + 1);
  
  return send(data, command_angle, 2);
}

ssize_t actuator::send(uint8_t* msg, msg_type id, uint8_t size) {
  struct can_frame frame;

  frame.can_id = id;
  frame.can_dlc = size;
  for (uint8_t i = 0; i < size; i++)
    frame.data[i] = msg[i];
  
  return write(sockFD, &frame, sizeof(struct can_frame));
}
