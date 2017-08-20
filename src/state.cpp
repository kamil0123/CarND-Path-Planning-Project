#include "state.h"

State::State(){}

void State::update(const Vehicle &car, const std::vector<Vehicle>& otherVehicles, const Lane lane) {

	if (lane == Lane::NONE || lane == Lane::UNKOWN) {
	  onRoad = false;
	} else {

    onRoad = true;

	  this->front_distance = 1000;
	  this-> back_distance = 1000;
	
	  for (auto &observedVehicle : otherVehicles) {

      double distance = car.s - observedVehicle.s;

      if (distance > 0.0) {
        // observed car is over our car 
        if (observedVehicle.lane == lane && distance < this->back_distance) {
          this->back_distance = distance;
          this->back_s = observedVehicle.s;
          this->back_v = observedVehicle.v;
        } 

      } else {
        // observed car is in front of our car 
        distance = distance * (-1.0);
        if (observedVehicle.lane == lane && distance < this->front_distance) {
          this->front_distance = distance;
          this->front_s = observedVehicle.s;
          this->front_v = observedVehicle.v;
        } 
      }
	  }
  }
} 