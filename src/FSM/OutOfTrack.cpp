/**  @file: OutOfTrack.cpp
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include "OutOfTrack.h"

OutOfTrack::OutOfTrack(float _ms, float _nap, int _vg4, int _vg3, int _vg2, float _maxra, float _minra) : DrivingState(){
    setParameters(_ms, _nap, _vg4, _vg3, _vg2, _maxra, _minra);
}

void
OutOfTrack::setParameters(float _ms, float _nap, int _vg4, int _vg3, int _vg2, float _maxra, float _minra) {
    max_skidding = _ms;
    negative_accel_percent = _nap;
    velocity_gear_4 = _vg4;
    velocity_gear_3 = _vg3;
    velocity_gear_2 = _vg2;
    max_return_angle = _maxra;
    min_return_angle = _minra;
}

float
OutOfTrack::get_steer(CarState &cs) {
    float angle = cs.getAngle();
    if(cs.getTrackPos() > 0){
        if(angle > max_return_angle) return 1;
        if(angle < min_return_angle) return -1;
    } else {
        if(angle < -(max_return_angle)) return -1;
        if(angle > -(min_return_angle)) return 1;
    }

    return 0;
}

int
OutOfTrack::get_gear(CarState &cs) {
    if(cs.getSpeedX() > velocity_gear_4) return cs.getGear(); 
                                                            /* @todo need reverse behavior */
    if(cs.getSpeedX() > velocity_gear_3) return 3;
    if(cs.getSpeedX() > velocity_gear_2) return 2;

    return 1;
}

float
OutOfTrack::get_accel(CarState &cs) {
    return(1-abs(cs.getSpeedY())*negative_accel_percent); /* @todo can be negative, need some fix */
}

float
OutOfTrack::get_brake(CarState &cs) {
    if(cs.getSpeedX() < 0) return 1;
    if(abs(cs.getSpeedY()) > max_skidding) return 0.1;

    return 0;
}

float
OutOfTrack::get_clutch(CarState &cs){
    return 0;
}


OutOfTrack::~OutOfTrack() {
    /* Nothing. */
}

