#ifndef UTILS_H_
#define UTILS_H_

#include <vector>

const double DELTA_TIME = 0.02;

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


#endif /* UTILS_H_ */