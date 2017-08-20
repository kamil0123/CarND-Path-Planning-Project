#ifndef BEHAVIORPLANNER
#define BEHAVIORPLANNER

#include <vector>
#include "helper.h"
#include "vehicle.h"

class BehaviorPlanner {

  public:
    BehaviorPlanner();
    
	BehaviorType updateState(Vehicle& car, std::vector<Vehicle>& otherVehicles);
};

#endif /* BEHAVIORPLANNER */