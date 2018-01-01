#ifndef NMEA_SENTENCE_PARSER_HPP
#define NMEA_SENTENCE_PARSER_HPP

#include <cmath>
#include <string>
#include <vector>
#include <numeric>
#include <boost/algorithm/string.hpp>

namespace hdl_graph_slam {

struct GPRMC {
public:
  GPRMC() {
    status = 'V';
  }

  GPRMC(const std::vector<std::string>& tokens) {
    if(tokens[0] != "$GPRMC" || tokens.size() < 12) {
      status = 'V';
      return;
    }

    long time = std::stol(tokens[1]);
    hour = time / 10000;
    minute = (time % 10000) / 100;
    second = time % 100;

    status = tokens[2][0];

    latitude = degmin2deg(std::stod(tokens[3]));
    latitude = tokens[4] == "N" ? latitude : -latitude;

    longitude = degmin2deg(std::stod(tokens[5]));
    longitude = tokens[6] == "E" ? longitude : -longitude;

    speed_knots = std::stod(tokens[7]);
    track_angle_degree = std::stod(tokens[8]);

    long date = std::stol(tokens[9]);
    year = date % 100;
    month = (date / 100) % 100;
    day = (date / 10000) % 100;

    magnetic_variation = std::stod(tokens[10]);
    magnetic_variation = tokens[11][0] == 'E' ? magnetic_variation : -magnetic_variation;
  }

  double degmin2deg(double degmin) {
    double d = std::floor(degmin / 100.0);
    double m = (degmin - d * 100.0) / 60.0;
    return d + m;
  }

public:
  char status;                  // Status A=active or V=Void.

  int hour;                     // Fix taken at 12:35:19 UTC
  int minute;
  int second;

  double latitude;              //
  double longitude;

  double speed_knots;           // Speed over the ground in knots
  double track_angle_degree;    // Track angle in degrees True

  int year;
  int month;
  int day;

  double magnetic_variation;
};

class NmeaSentenceParser {
public:
  NmeaSentenceParser() {}
  ~NmeaSentenceParser() {}

  GPRMC parse(const std::string& sentence) const {
    int checksum_loc = sentence.find('*');
    if(checksum_loc == std::string::npos) {
      return GPRMC();
    }

    int checksum = std::stoul(sentence.substr(checksum_loc + 1), nullptr, 16);

    std::string substr = sentence.substr(1, checksum_loc - 1);
    int sum = std::accumulate(substr.begin(), substr.end(), static_cast<unsigned char>(0), [=](unsigned char n, unsigned char c) { return n ^ c; });

    if(checksum != (sum & 0xf)) {
      std::cerr << "checksum doesn't match!!" << std::endl;
      std::cerr << sentence << " " << sum << std::endl;
      return GPRMC();
    }

    std::vector<std::string> tokens;
    boost::split(tokens, sentence, boost::is_any_of(","));

    return GPRMC(tokens);
  }
};

}

#endif // NMEA_SENTENCE_PARSER_HPP

