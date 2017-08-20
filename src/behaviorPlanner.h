#ifndef BEHAVIORPLANNER
#define BEHAVIORPLANNER

#include <vector>
#include "vehicle.h"
#include "state.h"

class BehaviorPlanner {

  public:
    BehaviorPlanner();
    
	BehaviorType updateState(Vehicle& car, std::vector<Vehicle>& otherVehicles);
};

#endif /* BEHAVIORPLANNER */