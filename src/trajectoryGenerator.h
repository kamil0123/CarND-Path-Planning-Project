#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <vector>
#include <iostream>

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

    TrajectoryGenerator updateTrajectory(Vehicle& car, int& ref_vel, Behavior& behavior, MapPoints& mapPoints);
};

#endif /* BEHAVIORPLANNER */