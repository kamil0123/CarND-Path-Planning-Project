#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <vector>
#include <iostream>

#include "behavior.h"
#include "vehicle.h"
#include "state.h"

class TrajectoryGenerator {

  public:

    int lane;
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
    TrajectoryGenerator();

    TrajectoryGenerator updateTrajectory(Vehicle& car, Behavior& behavior);
};

#endif /* BEHAVIORPLANNER */