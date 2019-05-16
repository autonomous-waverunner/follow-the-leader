#ifndef COORDS_GEN_H
#define COORDS_GEN_H

#include <string>
#include <vector>
#include <fstream>
#include <utility>

#define EX_FILE_NOT_FOUND 1
#define EX_PARSE 2

class coords_gen {
 public:
  coords_gen(std::string fname);
  std::pair<double, double> get();
  
 private:
  int _coords_idx = 0;
  std::vector<std::pair<double,double>> _coords;
};

#endif //COORDS_GEN_H
