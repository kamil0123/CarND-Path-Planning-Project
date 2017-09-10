## Write up

### The goal of this project

In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

Path planner must meet the following criteria: 
* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit, which is 50 MPH.
* The car isn't driving much slower than speed limit unless obstructed by traffic.
* The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
* The car does not have collisions.
* The highway has 6 lanes total - 3 heading in each direction.The car should only ever be in one of the 3 lanes on the right-hand side.
* The car stays in its lane, except for the time between changing lanes.
* The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes.
* The car is able to change lanes.

### Overview

Pipeline steps:
1. Get sensor data about my car.
2. Get sensor data about other vehicles.
3. Get data about map and previous trajectory of my car.
4. Path planning - decide what to do - keep or change lane, how to change velocity.
5. Calculate new trajectory.

### 1. Get sensor data about my car.
The data contains:
* car's location in global map coordiante sytem (x, y and yaw)
* car's location in Frenet coordinate sytem (s, d)
* speed (m/s)
* previous path data (x, y) of the car
* Previous path's end s and d values








