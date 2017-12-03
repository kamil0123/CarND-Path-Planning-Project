# include "trajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator(){}

// For converting back and forth between radians and degrees.
double TrajectoryGenerator::deg2rad(double x) { return x * M_PI / 180; }
double TrajectoryGenerator::rad2deg(double x) { return x * 180 / M_PI; }

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> TrajectoryGenerator::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

void TrajectoryGenerator::updateTrajectory(Vehicle& car, double& ref_vel, Behavior& behavior, MapPoints& mapPoints) {
  
  int lane;
  
  if (car.lane == Lane::CENTER && behavior.laneType == BehaviorLaneType::KEEP_LANE) {
    lane = 1;
  } else if (car.lane == Lane::CENTER && behavior.laneType == BehaviorLaneType::LANE_CHANGE_LEFT) {
    lane = 0;
  } else if (car.lane == Lane::CENTER && behavior.laneType == BehaviorLaneType::LANE_CHANGE_RIGHT) {
    lane = 2;
  } else if (car.lane == Lane::LEFT && behavior.laneType == BehaviorLaneType::KEEP_LANE) {
    lane = 0;
  } else if (car.lane == Lane::LEFT && behavior.laneType == BehaviorLaneType::LANE_CHANGE_RIGHT) {
    lane = 1;
  } else if (car.lane == Lane::RIGHT && behavior.laneType == BehaviorLaneType::KEEP_LANE) {
    lane = 2;
  } else if (car.lane == Lane::RIGHT && behavior.laneType == BehaviorLaneType::LANE_CHANGE_LEFT) {
    lane = 1;
  }

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // Later we will iterplate these waypoints with a spline and fill it in with more points that control spedd.

  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states
  // either we will reference the starting point as where the car is or at the presious paths end point
  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = this->deg2rad(car.yaw);

  int prev_size = mapPoints.previous_path_x.size();

  // if previous size is almost empty, use the car as starting reference
  if (prev_size < 2) {
    // Use two points that make the path tangent to the car
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car.y);
  } 
  // use the previous path's end point as starting reference
  else {
    // redefine reference state as previous path end point
    ref_x = mapPoints.previous_path_x[prev_size - 1];
    ref_y = mapPoints.previous_path_y[prev_size - 1];

    double ref_x_prev = mapPoints.previous_path_x[prev_size-2];
    double ref_y_prev = mapPoints.previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30miles spaced points ahead of the starting reference
  vector<double> next_wp0 = this->getXY(car.s + 30, (2+4*lane), mapPoints.map_waypoints_s, mapPoints.map_waypoints_x, mapPoints.map_waypoints_y);
  vector<double> next_wp1 = this->getXY(car.s + 60, (2+4*lane), mapPoints.map_waypoints_s, mapPoints.map_waypoints_x, mapPoints.map_waypoints_y);
  vector<double> next_wp2 = this->getXY(car.s + 90, (2+4*lane), mapPoints.map_waypoints_s, mapPoints.map_waypoints_x, mapPoints.map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // start with all of the previous path points from last time
  for (int i = 0; i < mapPoints.previous_path_x.size(); i++) {
    this->next_x_vals.push_back(mapPoints.previous_path_x[i]);
    this->next_y_vals.push_back(mapPoints.previous_path_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 35;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

  double x_add_on = 0;

  // fill up the rest of paths planner after filling it with previous points, here we will always output 50 points

  for (int i = 1; i <= 50 - mapPoints.previous_path_x.size(); i++) {

    double N = (target_dist/(DELTA_TIME * ref_vel / 2.24)); // divide by 2.24 to get mps (meters per second) from mph (miles per hour)
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
    
    x_point += ref_x;
    y_point += ref_y;

    this->next_x_vals.push_back(x_point);
    this->next_y_vals.push_back(y_point);
     
  }
}