#ifndef BEHAVIORPLANNER
#define BEHAVIORPLANNER

#include <vector>

class BehaviorPlanner {

  public:
    BehaviorPlanner();
    
	BehaviorType update(Vehicle& car, std::vector<Vehicle>& otherVehicles);
};

#endif /* BEHAVIORPLANNER */