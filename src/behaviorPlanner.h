#ifndef BEHAVIORPLANNER
#define BEHAVIORPLANNER

#include <vector>
#include "behavior.h"
#include "vehicle.h"
#include "state.h"

class BehaviorPlanner {

  public:
    BehaviorPlanner();
    
	Behavior updateState(Vehicle& car, std::vector<Vehicle>& otherVehicles);
};

#endif /* BEHAVIORPLANNER */