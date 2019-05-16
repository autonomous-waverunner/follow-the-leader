#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

enum msg_type {
  request_status = 0,
  command_aps_rps = 1,
  command_angle = 2
};

#endif // CAN_PROTOCOL_H
