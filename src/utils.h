#ifndef UTILS
#define UTILS

#include <vector>

const double DELTA_TIME = 0.02;

enum class Lane {
  LEFT, CENTER, RIGHT, NONE, UNKOWN
};

enum class BehaviorType {
	KEEP_LANE, LANE_CHANGE_RIGHT, LANE_CHANGE_LEFT
};


#endif /* UTILS */