#ifndef PRINTS_H
#define PRINTS_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include "path_alg.h"
#include "WRPosition.h"

void printParams(std::ostream &s, config cfg);
void printState(std::ostream &s, double dt);
void printStateHeader(std::ostream &s);

#endif // PRINTS_F
