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
  if (currentLaneState.front_distance < FRONT_TOO_CLOSE) {
    too_close = true;
    too_close_speed = currentLaneState.front_v;

    const double costKeepLane    = 0.0;
    const double costChangeLeft  = 0.0;
    const double costChangeRight = 0.0;

    if (car.lane == Lane::CENTER) {
      costKeepLane    = 200 - 2 * this->getFrontCost(currentLaneState);
      costChangeLeft  = 200 - (this->getFrontCost(leftLaneState)  + this->getBackCost(leftLaneState));
      costChangeRight = 200 - (this->getFrontCost(rightLaneState) + this->getBackCost(rightLaneState));

      costKeepLane    = this->logisticFunction(costKeepLane);
      costChangeLeft  = this->logisticFunction(costChangeLeft);
      costChangeRight = this->logisticFunction(costChangeRight);

    } else if (car.lane == Lane::LEFT) {
      costKeepLane    = 200 - 2 * this->getFrontCost(currentLaneState);
      costChangeRight = 200 - (this->getFrontCost(rightLaneState) + this->getBackCost(rightLaneState));

      costKeepLane    = this->logisticFunction(costKeepLane);
      costChangeLeft = 1.0;
      costChangeRight = this->logisticFunction(costChangeRight);

    } else if (car.lane == Lane::RIGHT) {
      costKeepLane    = 200 - 2 * this->getFrontCost(currentLaneState);
      costChangeLeft  = 200 - (this->getFrontCost(leftLaneState)  + this->getBackCost(leftLaneState));

      costKeepLane    = this->logisticFunction(costKeepLane);
      costChangeLeft  = this->logisticFunction(costChangeLeft);
      costChangeRight = 1.0;

    }

    if (costKeepLane <= costChangeLeft && costKeepLane <= costChangeRight) {
      behavior.laneType = BehaviorLaneType::KEEP_LANE;
    } else if (costChangeLeft <= costKeepLane && costChangeLeft <= costChangeRight) {
      behavior.laneType = BehaviorLaneType::LANE_CHANGE_LEFT;
    } else if (costChangeRight <= costKeepLane && costChangeRight <= costChangeLeft) {
      behavior.laneType = BehaviorLaneType::LANE_CHANGE_RIGHT;
    }

  } else {
    behavior.laneType = BehaviorLaneType::KEEP_LANE;
  }

  if (too_close && car.v > too_close_speed) {
    behavior.accType = BehaviorAccType::DOWN;
  } else if (car.v < MAX_REF_VEL) {
    behavior.accType = BehaviorAccType::UP;
  } else {
    behavior.accType = BehaviorAccType::NONE;
  }
  
  return behavior;
}