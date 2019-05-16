#include "LCPositions.h"
#include <string.h>
bool updateParams = false;
config new_config = { 0 };
State state = PARAMS;
path_track* pp;

int initRadio(std::string dev) {
  int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1) {
    std::cerr << "Error opening " << dev << std::endl;
    exit(1);
  }

  /*---------- Setting the Attributes of the serial port using termios structure --------- */

  struct termios SerialPortSettings;
  tcgetattr(fd, &SerialPortSettings);

  /* Setting the Baud rate */
  cfsetispeed(&SerialPortSettings,B115200);
  cfsetospeed(&SerialPortSettings,B115200);

  /* 8N1 Mode */
  SerialPortSettings.c_cflag &= ~PARENB;
  SerialPortSettings.c_cflag &= ~CSTOPB;
  SerialPortSettings.c_cflag &= ~CSIZE;
  SerialPortSettings.c_cflag |=  CS8;

  SerialPortSettings.c_cflag &= ~CRTSCTS;
  SerialPortSettings.c_cflag |= CREAD | CLOCAL;

  SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
  SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  SerialPortSettings.c_oflag &= ~OPOST;

  /* Setting Time outs */
  SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
  SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

  if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) {
    std::cerr << "ERROR ! in Setting attributes" << std::endl;
    exit(1);
  }

  tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
  return fd;
}

#define N_PARAMS 18
#define FLOAT_SIZE 8
#define EXTRA 10
#define BUFSIZE 1024

void getLCPositions(int fd) {
  uint8_t buf[BUFSIZE];
  while (true) {
    ssize_t sz = read(fd, &buf, BUFSIZE);

    if (sz > 0) {
      std::cerr << "Read " << std::to_string(sz) << " bytes:" << std::endl
		<< buf << std::endl;
      
      // Should verify integrity of message here
      switch (buf[0]) {
      case C_COORDINATES:
	std::cerr << "COORDINATES STATE" << std::endl;
	state = COORDINATES;
	break;
      case C_WAIT:
	std::cerr << "WAIT STATE" << std::endl;
	state = WAIT;
	break;
      case C_PARAMS:
	std::cerr << "PARAMS STATE" << std::endl;
	state = PARAMS;
	break;
      }
      char* msg = buf + 2;
      switch (state) {
      case COORDINATES:
	std::cerr << "case COORDINATES" << std::endl;
	if (pp) {
	  try {
	    msg_coordinate m = parse_coordinates(msg);
	    std::cerr << "Utc: " << m.utc
		      << "  Lat: " << m.latitude
		      << "  Lon: " << m.longitude << std::endl;
	    Eigen::Vector2d new_coord(m.latitude, m.longitude);
	    pp->push_coord(new_coord);
	  } catch (const std::exception& e) {
	    std::cerr << "invalid coordinates msg: " << e.what() << std::endl;
	  } catch (...) {
	    std::cerr << "invalid coordinates msg" << std::endl;
	  }
	}
	break;
      case WAIT:
	break;
      case PARAMS:
	std::cerr << "case PARAMS" << std::endl;
	try {
	  msg_parameters m = parse_parameters(msg);
	  new_config.throttle = m.throttle;
	  new_config.nozzle = m.nozzle;
	  new_config.deltaT = m.deltaT;
	  new_config.dist_ref = m.dist_ref;
	} catch (const std::exception& e) {
	    std::cerr << "invalid params msg: " << e.what() << std::endl;
	} catch (...) {
	  std::cerr << "invalid params msg" << std::endl;
	}
	break;
      }
    }
    sleep(0.01);
  }
}

config getUpdatedConfig() {
  return new_config;
};
