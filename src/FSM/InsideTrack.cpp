/**  @file: InsideTrack.cpp
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include "InsideTrack.h"

InsideTrack::InsideTrack(int _sg, int _lgl, int _lrpm, int _arpm,
                         int _hrpm, float _bs, float _sf): DrivingState(){
    setParameters(_sg, _lgl, _lrpm, _arpm, _hrpm, _bs, _sf);
}

void
InsideTrack::setParameters( int _sg, int _lgl, int _lrpm, int _arpm,
                    int _hrpm, float _bs, float _sf) {
    start_gear = _sg;
    low_gear_limit = _lgl;
    low_rpm = _lrpm;
    average_rpm = _arpm;
    high_rpm = _hrpm;
    base_speed = _bs;
    speed_factor = _sf;
    current_gear = start_gear;
}

bool
InsideTrack::shouldDecreaseGear(int current_gear, int rpm) {
    if(isLowGear(current_gear) && runningOnLow(rpm)) return true;
    if(isHighGear(current_gear) && runningUnderAverage(rpm)) return true;
    return false;
}

inline bool
InsideTrack::runningOnLow(int rpm) {
    return (rpm < low_rpm);
}

inline bool
InsideTrack::runningUnderAverage(int rpm) {
    return (rpm <= average_rpm);
}

inline bool
InsideTrack::runningOnHigh(int rpm) {
    return (rpm > high_rpm);
}

inline bool
InsideTrack::isLowGear(int gear) {
    return (gear > start_gear && gear < low_gear_limit);
}

inline bool
InsideTrack::isHighGear(int gear) {
    return (gear >= low_gear_limit);
}

inline bool
InsideTrack::shouldIncreaseGear(int current_gear, int rpm) {
    return runningOnHigh(rpm);
}

bool
InsideTrack::isFacingWrongWay(CarState &cs) {
    return cs.getAngle() < -M_PI/2 || cs.getAngle() > M_PI/2;
}

void
InsideTrack::setTargetSpeed(CarState &cs) {
    this->target_speed = base_speed + speed_factor*this->distance;
}

float
InsideTrack::findFarthestDirection(CarState &cs) {
    float farthestSensor = -INFINITY;
    float farthestDirection = 0;
       for (int i = 0; i < 19; i++) { /*Percorre os sensores de pista e guarda o de maior valor*/
          if (farthestSensor < cs.getTrack(i)) {
            farthestSensor = cs.getTrack(i);
            farthestDirection = i;
        }
    }
    this->distance = farthestSensor;
    farthestDirection = -M_PI/2 + farthestDirection*M_PI/18;
    return normalizeSteer(-farthestDirection);
}

float
InsideTrack::normalizeSteer(float angle) {
    const float maxsteer = 0.785398;
    return angle/maxsteer;
}

InsideTrack::~InsideTrack() {
    /* Nothing. */
}



/**************************************************************************
 * Modularization*/
float
InsideTrack::get_steer(CarState &cs) {
    return isFacingWrongWay(cs) ? cs.getAngle() : findFarthestDirection(cs);
}

int
InsideTrack::get_gear(CarState &cs){
    int gear = cs.getGear();
    if(gear <= 0) return start_gear;

    int rpm = cs.getRpm();

    if(shouldIncreaseGear(gear, rpm)) ++gear;
    else if(shouldDecreaseGear(gear, rpm)) --gear;

    return gear;
}

float
InsideTrack::get_accel(CarState &cs){
    setTargetSpeed(cs);
    return cs.getSpeedX() > target_speed ? 0 : 1;
}

float
InsideTrack::get_brake(CarState &cs){
    return cs.getSpeedX() > target_speed ? 0.3 : 0;
}

float
InsideTrack::get_clutch(CarState &cs){
    return 0;
}


CarControl
InsideTrack::drive(CarState &cs) {
    float steer = get_steer(cs);
    setTargetSpeed(cs);
    int gear = get_gear(cs);
    float accel  = get_accel(cs);
    float brake = get_brake(cs);
    float clutch = get_clutch(cs);

    return CarControl(accel, brake, gear, steer, clutch);
    
}
