#include "vehicle.h"

Vehicle::Vehicle(const int i){
  this->id = i;
}

void Vehicle::updateParameters(const double s, const double d, const double x, const double y, const double yaw, const double v) {
	this->s = s;
	this->d = d;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
	this->v = v;
	this->lane = this->calculateLane(this->d);
}

Lane Vehicle::calculateLane(const double d) {
	
	Lane lane = Lane::NONE;
	
	if (d > 0.0 && d <= 4.0) {
		lane = Lane::LEFT;
	} else if (d > 4.0 && d <= 8.0) {
		lane = Lane::CENTER;
	} else if (d > 8.0 && d <= 12.0) {
		lane = Lane::RIGHT;
	}
	
	return lane;
}

Lane Vehicle::calculateLane() {
	return this->calculateLane(this->d);
}
	
void Vehicle::updateSideLanes() {
	
	if (this->lane == Lane::LEFT) {
		this->laneOnLeft  = Lane::NONE;
		this->laneOnRight = Lane::CENTER;
		
	} else if (this->lane == Lane::CENTER) {
		this->laneOnLeft  = Lane::LEFT;
		this->laneOnRight = Lane::RIGHT;
		
	} else if (this->lane == Lane::RIGHT) {
		this->laneOnLeft  = Lane::CENTER;
		this->laneOnRight = Lane::NONE;
	
	} else {
		this->lane        = Lane::UNKOWN;
		this->laneOnLeft  = Lane::UNKOWN;
		this->laneOnRight = Lane::UNKOWN;
	}
}