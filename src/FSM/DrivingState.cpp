/**  @file: DrivingState.cpp
 *
 * https://github.com/bruno147/fsmdriver
 * 
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version. 
 */

#include "DrivingState.h"
#include "FSMDriver.h"

DrivingState::DrivingState() {
    /* Nothing. */
}

DrivingState::~DrivingState() {
	/* Nothing. */
}

void
DrivingState::enter() {
	/* Nothing. */
}

void
DrivingState::exit() {
	/* Nothing. */
}


/**************************************************************************
 * Modularização*/

CarControl 
drive(CarState &cs) {
    float steer = get_steer(cs);
    int gear = get_gear(cs);     
    float accel  = get_accel(cs);
    float brake = get_brake(cs);
    float clutch = get_clutch(cs);

    return CarControl(accel, brake, gear, steer, clutch);
}



