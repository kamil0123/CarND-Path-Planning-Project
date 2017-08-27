#ifndef BEHAVIORPLANNER
#define BEHAVIORPLANNER

#include <vector>
#include <iostream>

#include "behavior.h"
#include "vehicle.h"
#include "state.h"
#include "utils.h"

class BehaviorPlanner {

  public:
    BehaviorPlanner();
    
	Behavior updateState(Vehicle& car, std::vector<Vehicle>& otherVehicles);
};

#endif /* BEHAVIORPLANNER */