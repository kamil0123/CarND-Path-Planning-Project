#ifndef BEHAVIORPLANNER
#define BEHAVIORPLANNER

#include <vector>
#include <iostream>
#include <math.h>

#include "behavior.h"
#include "vehicle.h"
#include "state.h"
#include "utils.h"

class BehaviorPlanner {

  public:
    BehaviorPlanner();
    
	Behavior updateState(Vehicle& car, std::vector<Vehicle>& otherVehicles);
  double getFrontCost(Vehicle& car, State& state);
  double getBackCost(Vehicle& car, State& state);
  double logisticFunction(double& cost);
};

#endif /* BEHAVIORPLANNER */