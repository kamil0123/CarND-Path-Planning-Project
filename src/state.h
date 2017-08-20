#ifndef STATE_H_
#define STATE_H_

#include <vector>

#include "utils.h"
#include "vehicle.h"

class State {

  public:
    
	double front_delta_s;
	double front_s;
	double front_v;

	double back_delta_s;
	double back_s;
	double back_v;

	State();
	void update(const Vehicle &car, const std::vector<Vehicle>& otherVehicles, const Lane lane, const int direction);
};

#endif /* STATE_H_ */