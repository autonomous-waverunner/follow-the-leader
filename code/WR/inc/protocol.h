#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdexcept>
#include "path_alg.h"

#define C_COORDINATES 'C'
#define C_WAIT 'W'
#define C_PARAMS 'P'

/*
 * PROTOCOL OVERVIEW
 *
 * Each message starts with one byte (unsigned) specifying the type of the 
 * message. Following this is the payload. This is structured differently
 * depending on the type, see the structs below. A special case is the message
 * 'Wait' which has no payload. 
 *
 * The last field is the control sum, which is calculated by simply summing
 * the data array (including the type) interpreted as unsigned 8 bit integers.
 *
 *   +---+---------------+--~   ~--+---------------+---------------+
 *   | t |    data[0]    |   ...   |    data[n]    |    uint32_t   |
 *   +---+---------------+--~   ~--+---------------+---------------+
 *     8         32                        32              32      
 *
 */


struct msg_coordinate {
  double utc, latitude, longitude;
  uint32_t crc;
};

struct msg_parameters {
  pid_v throttle, nozzle; // 8 doubles  each
  double deltaT, dist_ref;
  uint32_t crc;
};

uint32_t calculate_crc(uint8_t* msg, size_t sz);
bool verify_crc(uint8_t* msg, size_t sz);
msg_coordinate parse_coordinates (uint8_t* buf);
msg_parameters parse_parameters (uint8_t* buf);

#endif //PROTOCOL_H
