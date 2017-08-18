#ifndef VEHICLE
#define VEHICLE

#include <vector>

const double DELTA_TIME = 0.02;

enum class Lane {
  LEFT, CENTER, RIGHT, NONE, UNKOWN
};

enum class BehaviorType {
	KEEP_LANE, TURN_RIGHT, TURN_LEFT
}


#endif /* VEHICLE */