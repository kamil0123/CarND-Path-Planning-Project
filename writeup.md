## Model documentation

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

We have given a sparse map list of waypoints around the highway.

Pipeline steps to calculate path trajectory:
1. Get sensor data about my car.
2. Get sensor data about other vehicles.
3. Path planning - decide what to do - keep or change lane and how to change velocity.
4. Generate new trajectory.

Trajectory of the car is a list of 50 points in global map coordinate system. 
If new circle of pipeline is calculated before car passed all poins generated in previous circle, not passed points are addad at the beginning of the new list of points and just remaining points (to finally get 50 points) are calculated.

### 1. Get sensor data about my car.

The data contains:
* car's location in global map coordiante sytem (x, y and yaw)
* car's location in Frenet coordinate sytem (s, d)
* speed in MPH (miles per hour)
* previous path data points (x, y), which were not used (the car haven't passed them yet)
* previous path's end s and d values

If list of previous data points is not empty, then we will add this points to new trajectory. Now, calculation of new trajectory will start not at current position of the car (s, d), but at end s and d of previous path. To take that into account we change given position s to end s from previous path (line 216 of main.cpp).

Object Vehcicle of our car is created at lines 224-226 of main.cpp file. 

### 2. Get sensor data about other vehicles

The data contains:
* car's id - unique identifier of the vehicle
* car's velocity in global map coordinate system (vx, vy) in m/s (meters per second)
* car's location in Frenet coordinate sytem (s, d)

If we have data points from previous path and start calculation of new trajectory at the end of previous path, then we also need to calculate where other vehicles will be at that point.
Total velocity of the car is calculated from vx and vy (sqrt(vx*vx + vy*vy)).
Then this value is used to predict where the car will be in the future. If it will be assumed that car is moving along the road, then its future s (Frenet coordinate system) will be equal to current s value plus its total velocity multiplied by number of previous, not driven yet, points and delta time (time elapsed between going two points).

Objects Vehcicle, for every other detected vehicle, are created at lines 245-229 of main.cpp file. 

### 3. Path planning

In path planning part we must decide how our car will behave. To do this I created simple finite state machine with two separte categories, 3 states each:
* lane (3 states): 
 * keep lane
 * lane change right
 * lane change left 
* velocity (3 states): 
 * don't change
 * increase
 * reduce

Calculations of path planning are made in methods of class BehaviorPlanner. Input of its main method (updateState) is data of our and other cars.

#### 3.1 Path planning - change lane

To calculate new state of lane changing I have projected simple cost functions. State with lowest cost is the best one.

The cost of every operation is calculated as logistic function with equation:
cost = L / (1 + e^(-k*(x-x0)))

I used constat values of paramters:
* L = 1
* k = 0.02
* x0 = 50
I tried find values, which will give final cost in wide range, not just close to zero and one.

Value of cost depends on x value and is in range of (0, 1). The lower x value, the lower the cost.

For keeping lane x is calculated as:
x = 200 - k * frontCost
where k is equal:
* 1.8, if our car is on center lane
* 1.5, if our car is on left or right lane
For changing lane x is equal:
x = 200 - (frontCost + backCost)

If car is on left or right lane we can only keep lane or change to center. So cost of changing in opposite direction is equal 1.0.

Parameter k is added in keeping lane equation because of lack of backCost (if we are not changing lane, we are not not interested in what happens behind us). Is smaller than 2 because front cost usually gives a little bigger value than back cost. If our car is on one of side lanes (left or right) is smaller than on center lane because I wanted to increase probability of going back on center lane to have more maneuver options in the future (turn left or right on center lane, or just one of it on side lane).

FrontCost and backCost are equal -200.0 at the beginning, which gives really big final cost. Then, depending of our cars velocity, distance to other cars and other cars velocity it can increase and gives smaller final cost.

#### 3.2 Path planning - velocity

Velocity of our car is:
* reduced - if car in front of our car is closer than 30 meters and it's velocity is lower than velociy of our car,
* increased - if velocity of our car is smaller than max reference velocity (46.0 MPH) and distance to nearest vehicle in front of our car is bigger than 10 meters,

otherwise it is not changed.

Velocity is changed by adding or subtracting 0.224 MPH (0.1 mps) to reference velocity, which is used in trajectory generator to calcluate new trajecoty points.

Velocity state is calculated in method updateState of class BehaviorPlanner (lines 78-89 of behaviorPlannes.cpp file). New reference velocity is calculated in lines 260-264 of main.cpp file.

#### 4. Generate new trajectory.

New trajectory is generated in class TrajectoryGenerator in method updateTrajecory.
Trajectory of the car is a list of 50 points in global map coordinate system. 
If new circle of pipeline is calculated before car passed all poins generated in previous circle, not passed points are addad at the beginning of the new list of points and just remaining points (to finally get 50 points) are calculated.
In TrajectoryGenerator each axe of coordiante system has it's own vector (file trajectoryGenerator.h, lines 22-23).

Previous, not used points, are added to new trajectory in lines 129-132 of file trajectoryGenerator.cpp.

To generate new path we need to know on which lane of road we want to be. We are on road with three lanes in our direction:
* left lane, nearest of center of road, marked as 0
* center lane, marked as 1,
* right lane, marked as 2,
in next steps we will use this mark to calculate distance of our car from center of road (d in Frenet coordinate system).

We want to generate five reference points of a car and then use spline library to get smooth trajectory. 

To determine first two reference points we will use information about current car position and previous trajectory. If list of previous not used points is less than two we use car current position to calculate reference points (file TrajectoryGenerator, lines 71-81). Otherwise we we will use previous path's end points (lines 85-97).

Now we calculate in Frenet coordiante system, where we want to be in next 30, 60 and 90 miles (lines 101-103), and then we shift car reference angle to 0 degrees (lines 113-119).

Next, by using spline we calculate where we will be, after moving 30 miles upward on map, and what distance we will travel (lines 135-137). With that informations we can calculate missing points of our 50 points of new trajectory (lines 143-162).

### Conclusions
I believe that my path planner is good and meets the requirements of the project. However, it can make mistakes. I tested it on 3 runs, each taked about 20 minutes. 2 runs goes correct, without any incidents. On one run, the car has one incident during lane change, about 2 minuts after start. It's difficult to determine what has not worked and how to improve it due to unique and changing environmental conditions (number and distance of other cars, their velocities, position and velocity of our car).





 
 








