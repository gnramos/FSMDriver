/**  @file: InsideTrack.cpp
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include "InsideTrackA.h"

InsideTrackA::InsideTrackA(int _sg, int _lgl, int _lrpm, int _arpm,
                         int _hrpm, float _bs, float _sf) {

    setParameters(_sg, _lgl, _lrpm, _arpm, _hrpm, _bs, _sf);
}

void
InsideTrackA::setParameters( int _sg, int _lgl, int _lrpm, int _arpm,
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
InsideTrackA::shouldDecreaseGear(int current_gear, int rpm) {
    if(isLowGear(current_gear) && runningOnLow(rpm)) return true;
    if(isHighGear(current_gear) && runningUnderAverage(rpm)) return true;
    return false;
}

inline bool
InsideTrackA::runningOnLow(int rpm) {
    return (rpm < low_rpm);
}

inline bool
InsideTrackA::runningUnderAverage(int rpm) {
    return (rpm <= average_rpm);
}

inline bool
InsideTrackA::runningOnHigh(int rpm) {
    return (rpm > high_rpm);
}

inline bool
InsideTrackA::isLowGear(int gear) {
    return (gear > start_gear && gear < low_gear_limit);
}

inline bool
InsideTrackA::isHighGear(int gear) {
    return (gear >= low_gear_limit);
}

inline bool
InsideTrackA::shouldIncreaseGear(int current_gear, int rpm) {
    return runningOnHigh(rpm);
}

bool
InsideTrackA::isFacingWrongWay(CarState &cs) {
    return cs.getAngle() < -M_PI/2 || cs.getAngle() > M_PI/2;
}

void
InsideTrackA::setTargetSpeed(CarState &cs) {
    this->target_speed = base_speed + speed_factor*this->distance;
}

float
InsideTrackA::findFarthestDirection(CarState &cs) {
    float farthestSensor = -INFINITY;
    float farthestDirection = 0;
       for (int i = 0; i < 19; i++) { 
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
InsideTrackA::normalizeSteer(float angle) {
    const float maxsteer = 0.785398;
    return angle/maxsteer;
}

InsideTrackA::~InsideTrackA() {
    /* Nothing. */
}



/**************************************************************************
 * Modularization*/
float
InsideTrackA::get_steer(CarState &cs) {
    return isFacingWrongWay(cs) ? cs.getAngle() : findFarthestDirection(cs);
}

int
InsideTrackA::get_gear(CarState &cs){
    int gear = cs.getGear();
    if(gear <= 0) return start_gear;

    int rpm = cs.getRpm();

    if(shouldIncreaseGear(gear, rpm)) ++gear;
    else if(shouldDecreaseGear(gear, rpm)) --gear;

    return gear;
}

float
InsideTrackA::get_accel(CarState &cs){
    setTargetSpeed(cs);
    float Front, max10, max20;

    Front = cs.getTrack(10);
    max10 = max(cs.getTrack(9), cs.getTrack(11));
    max20 = max(cs.getTrack(8), cs.getTrack(12));

    float accel = (cs.getSpeedX() > target_speed ? 0 : (Front+max10+max20)/(3*200));

    if(Front >= 70) accel = 1;

    if(Front <= 20 && cs.getSpeedX() <= 30) accel = 1;  /*Resolve o caso em que o carro está preso com a frente voltada para a borda da pista*/

    //printf("%.0f, %.0f, %.0f --> accel: %.2f\n", Front, max10, max20, accel);

    return accel ;
}

float
InsideTrackA::get_brake(CarState &cs){
    return cs.getSpeedX() > target_speed ? 0.3 : 0;
}

float
InsideTrackA::get_clutch(CarState &cs){
    return 0;
}
