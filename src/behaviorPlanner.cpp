#include "behaviorPlanner.h"

using namespace std;

BehaviorPlanner::BehaviorPlanner(){}

Behavior BehaviorPlanner::updateState(Vehicle& car, std::vector<Vehicle>& otherVehicles) {

	State currentLaneState;
	State leftLaneState;
	State rightLaneState;

	currentLaneState.update(car, otherVehicles, car.lane);
	leftLaneState.update(car, otherVehicles, car.laneOnLeft);
	rightLaneState.update(car, otherVehicles, car.laneOnRight);

  Behavior behavior;
  
  bool too_close = false;
  double too_close_speed = 0.0;
  // check there is a car in front of us and closer than 30 meters
  if (currentLaneState.front_distance < 30) {
    too_close = true;
    too_close_speed = currentLaneState.front_v;
    
    if (car.lane == Lane::CENTER) {
      behavior.laneType = BehaviorLaneType::LANE_CHANGE_LEFT;
    } else if (car.lane == Lane::LEFT) {
      behavior.laneType = BehaviorLaneType::LANE_CHANGE_RIGHT;
    } else {
      behavior.laneType = BehaviorLaneType::KEEP_LANE;
    }
  } else {
    behavior.laneType = BehaviorLaneType::KEEP_LANE;
  }

  if (too_close && car.v > too_close_speed) {
    behavior.accType = BehaviorAccType::DOWN;
  } else if (car.v < 49.5) {
    behavior.accType = BehaviorAccType::UP;
  } else {
    behavior.accType = BehaviorAccType::NONE;
  }
  
  return behavior;
}