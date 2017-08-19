#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <vector>
#include <math.h>
#include "utils.h"

class Vehicle {

  public:
    
	int id;
    double s;
    double d;
    double v;
	
	Lane lane;
	Lane laneOnLeft;
	Lane laneOnRight;

    Vehicle(const int i);
	
	void updateParameters(const double s, const double d, const double v);
	void updateSideLanes();

	Lane calculateLane(const double d);
	Lane calculateLane();
};

#endif /* VEHICLE_H_ */