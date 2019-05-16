#ifndef LCPOSITIONS_H
#define LCPOSITIONS_H

#include <iostream>
#include <fstream>
#include <string>
#include <sys/poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "path_alg.h"
#include "protocol.h"

// More states may be added
enum State {COORDINATES, WAIT, PARAMS};

int initRadio(std::string dev);
void getLCPositions(int fd);
config getUpdatedConfig();

extern bool updateParams;
extern State state;
extern path_track* pp;

#endif // LCPOSITIONS_H
