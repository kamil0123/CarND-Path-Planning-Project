#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <vector>
#include <iostream>
#include <math.h>

#include "spline.h"

#include "utils.h"
#include "behavior.h"
#include "vehicle.h"
#include "state.h"

using namespace std;

class TrajectoryGenerator {

  public:

    int lane;
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
    TrajectoryGenerator();

    double deg2rad(double x);
    double rad2deg(double x);
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

    void updateTrajectory(Vehicle& car, double& ref_vel, Behavior& behavior, MapPoints& mapPoints);
};

#endif /* BEHAVIORPLANNER */