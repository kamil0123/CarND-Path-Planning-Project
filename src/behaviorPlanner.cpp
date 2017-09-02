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

    double costKeepLane    = 1.0;
    double costChangeLeft  = 1.0;
    double costChangeRight = 1.0;

    if (car.lane == Lane::CENTER) {
      costKeepLane    = 200 - 1.8 * this->getFrontCost(car, currentLaneState);
      costChangeLeft  = 200 - (this->getFrontCost(car, leftLaneState)  + this->getBackCost(car, leftLaneState));
      costChangeRight = 200 - (this->getFrontCost(car, rightLaneState) + this->getBackCost(car, rightLaneState));

      cout << costChangeLeft << " | " << costKeepLane << " | " << costChangeRight << endl;

      costKeepLane    = this->logisticFunction(costKeepLane);
      costChangeLeft  = this->logisticFunction(costChangeLeft);
      costChangeRight = this->logisticFunction(costChangeRight);

    } else if (car.lane == Lane::LEFT) {
      costKeepLane    = 200 - 1.5 * this->getFrontCost(car, currentLaneState);
      costChangeRight = 200 - (this->getFrontCost(car, rightLaneState) + this->getBackCost(car, rightLaneState));

      cout << costChangeLeft << " | " << costKeepLane << " | " << costChangeRight << endl;

      costKeepLane    = this->logisticFunction(costKeepLane);
      costChangeRight = this->logisticFunction(costChangeRight);

    } else if (car.lane == Lane::RIGHT) {
      costKeepLane    = 200 - 1.5 * this->getFrontCost(car, currentLaneState);
      costChangeLeft  = 200 - (this->getFrontCost(car, leftLaneState)  + this->getBackCost(car, leftLaneState));

      cout << costChangeLeft << " | " << costKeepLane << " | " << costChangeRight << endl;

      costKeepLane    = this->logisticFunction(costKeepLane);
      costChangeLeft  = this->logisticFunction(costChangeLeft);

    }

    cout << costChangeLeft << " | " << costKeepLane << " | " << costChangeRight << endl;

    if (costKeepLane <= costChangeLeft && costKeepLane <= costChangeRight) {
      behavior.laneType = BehaviorLaneType::KEEP_LANE;
      cout << "Behavior: KeepLane"  << endl;;
    } else if (costChangeLeft <= costKeepLane && costChangeLeft <= costChangeRight) {
      behavior.laneType = BehaviorLaneType::LANE_CHANGE_LEFT;
      cout << "Behavior: ChangeLeft" << endl;;
    } else if (costChangeRight <= costKeepLane && costChangeRight <= costChangeLeft) {
      behavior.laneType = BehaviorLaneType::LANE_CHANGE_RIGHT;
      cout << "Behavior: ChangeRight" << endl;;
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

double BehaviorPlanner::getFrontCost(Vehicle& car, State& state) {

  double cost = -200.0;

  if (state.front_distance > 40) {
    cost = 3 * state.front_distance - state.front_v;
  } else if (state.front_distance > 30 && (car.v + 10) < state.front_v) {
    cost = 3 * state.front_distance - 2 * state.front_v;
  }

  return cost;
}

double BehaviorPlanner::getBackCost(Vehicle& car, State& state) {

  double cost = -200.0;

  if (state.back_distance > 40) {
    cost = 2 * state.front_distance - state.front_v;
  } else if (state.back_distance > 20 && car.v > (state.back_v + 10)) {
    cost = 2 * state.front_distance - 2 * state.front_v;
  }

  return cost;
}

double BehaviorPlanner::logisticFunction(double& cost) {
  return ( 1 / ( 1 + exp(-0.02 * (cost - 50)) ) );


}

