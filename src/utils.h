#ifndef UTILS_H_
#define UTILS_H_

#include <vector>

const double DELTA_TIME = 0.02;
const double MAX_REF_VEL = 46.0;
const double FRONT_TOO_CLOSE = 30.0;

// reference velocity to targer
// mph - miles per hour
extern double ref_vel; 

enum class Lane {
  LEFT, CENTER, RIGHT, NONE, UNKOWN
};

enum class BehaviorLaneType {
	KEEP_LANE, LANE_CHANGE_RIGHT, LANE_CHANGE_LEFT
};

enum class BehaviorAccType {
  NONE, UP, DOWN
};

enum class Direction {
	FRONT, BACK
};

struct MapPoints {
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y; 
  std::vector<double> map_waypoints_x; 
  std::vector<double> map_waypoints_y; 
  std::vector<double> map_waypoints_s;
};

#endif /* UTILS_H_ */