#ifndef STATE_H_
#define STATE_H_

#include <vector>

#include "utils.h"
#include "vehicle.h"

class State {

  public:
    
    bool onRoad;

	double front_distance;
	double front_s;
	double front_v;

	double back_distance;
	double back_s;
	double back_v;

	State();
	void update(const Vehicle &car, const std::vector<Vehicle>& otherVehicles, const Lane lane);
};

#endif /* STATE_H_ */