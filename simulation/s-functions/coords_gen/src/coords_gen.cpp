#include "coords_gen.h"
//#include <iostream>

coords_gen::coords_gen(std::string fname) {
  std::ifstream coords_file (fname);
  std::string line;

  if (coords_file.is_open()) {
    while (getline(coords_file, line))
	{
      try {
        std::size_t ws_pos = line.find_first_of(" ");
        std::size_t lat_pos = line.find_first_not_of(" ", ws_pos);

        std::string lon_str = line.substr(0, ws_pos);
        std::string lat_str = line.substr(lat_pos, line.length());

        double lon = std::stod(lon_str);
        double lat = std::stod(lat_str);
	//	std::cout << lon << " " << lat << std::endl;

        double utm_x, utm_y;

        _coords.push_back(std::pair<double, double>(lat, lon));
      } catch (...) {
	throw EX_PARSE;
      }
    }
    coords_file.close();
  } else throw EX_FILE_NOT_FOUND;
}

std::pair<double, double> coords_gen::get() {
  auto elem = _coords[_coords_idx];
  if (_coords_idx < _coords.size() - 1)
    _coords_idx++;
  return elem;
}
