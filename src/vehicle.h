#ifndef VEHICLE
#define VEHICLE

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
	void update_lines();

	Lane calculateLane(const double d);
	void specifySideLanes();

};

#endif /* VEHICLE */