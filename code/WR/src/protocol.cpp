#include <string>
#include <vector>

#include "protocol.h"

uint32_t calculate_crc(uint8_t* msg, size_t sz) {
  uint32_t sum = 0;
  
  for (unsigned int i = 0; i < sz; i++)
    sum += msg[i];

  return sum;
}

bool verify_crc(uint8_t* msg, size_t sz) {
  return calculate_crc(msg, sz - 4) == *((uint32_t*) (msg + sz - 4));
}

std::vector<std::string> split(const std::string& str, const std::string& delim) {
  std::vector<std::string> tokens;
  size_t prev = 0, pos = 0;

  do {
    pos = str.find(delim, prev);
    if (pos == std::string::npos)
      pos = str.length();
    std::string token = str.substr(prev, pos - prev);
    if (!token.empty())
      tokens.push_back(token);
    prev = pos + delim.length();
  } while (pos < str.length() && prev < str.length());
  return tokens;
}

double fixCoord(std::string s) {
  double coord;
  std::string s1 = s.substr(0, 2);
  std::string s2 = s.substr(2, std::string::npos);

  try {
    coord = std::stod(s1) + std::stod(s2) / 60.0;
  } catch (...) {
    throw;
  }

  return coord;
}

msg_coordinate parse_coordinates (uint8_t* buf) {
  const std::string str = std::string((const char*) buf);
  std::vector<std::string> tokens = split(str, ",");
  msg_coordinate msg = { 0 };

  if (tokens.size() != 3) {
    throw std::invalid_argument("Invalid coordinates message");
  }

  try {
    msg.latitude = fixCoord(tokens[1]);
    msg.longitude = fixCoord(tokens[2]);
  } catch (...) {
    throw;
  }
  
  return msg;
}

msg_parameters parse_parameters (uint8_t* buf) {
  const std::string str = std::string((const char*) buf);
  std::vector<std::string> tokens = split(str, ",");
  msg_parameters msg = { 0 };
  
  if (tokens.size() != 18) {
    throw std::invalid_argument("Invalid parameters message");
  }

  try {
    pid_v throttle = { 0 }, nozzle = { 0 };
    msg.deltaT = std::stod(tokens[1]);
    msg.dist_ref = std::stod(tokens[0]);
    throttle.Kp = std::stod(tokens[2]);
    throttle.Tf = std::stod(tokens[5]);
    throttle.Kd = std::stod(tokens[4]);
    throttle.Ki = std::stod(tokens[3]);
    throttle.min = std::stod(tokens[7]);
    throttle.max = std::stod(tokens[6]);
    throttle.lp = std::stod(tokens[8]);
    throttle.scaling = std::stod(tokens[16]);
    nozzle.Kp = std::stod(tokens[9]);
    nozzle.Tf = std::stod(tokens[12]);
    nozzle.Kd = std::stod(tokens[11]);
    nozzle.Ki = std::stod(tokens[10]);
    nozzle.min = std::stod(tokens[14]) * pi / 180;
    nozzle.max = std::stod(tokens[13]) * pi / 180;
    nozzle.lp = std::stod(tokens[15]);
    nozzle.scaling = std::stod(tokens[17]);
    msg.throttle = throttle;
    msg.nozzle = nozzle;
  } catch (...) {
    throw;
  }

  return msg;
}
