/**  @file: InsideTrack.cpp
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include "InsideTrackPID.h"
#include <cmath>

const float InsideTrackPID::WHEEL_RAD = 0.4836;
const float InsideTrackPID::REFERENCE_SLIP = 0.2;
const float InsideTrackPID::dt = 0.02;
const float InsideTrackPID::Kp = -0.60178; /*best = -0.60178*/
const float InsideTrackPID::Ti = 0.4098; /*best = 0.4015*/
const float InsideTrackPID::Td = 0.1; /*best = 0.05*/

float InsideTrackPID::ABSfilter(CarState &carState){ 
    float brake;
    get_error(carState);

    brake = Kp * (current_error + Ti * integral_of_error() + Td * derivative_of_error()); 

    last_error = current_error;

    return brake;

}

float
InsideTrackPID::get_brake(CarState &cs){
    float brake = InsideTrack::get_brake(cs);
    return brake == 0 ? reset_ABS() : ABSfilter(cs);
}

float InsideTrackPID::reset_ABS(){
    last_integral = 0;

    return 0;
}

float InsideTrackPID::derivative_of_error(){
    return (current_error - last_error)/dt;
}

float InsideTrackPID::integral_of_error(){
    float integral = last_integral + (current_error * dt);
    last_integral = integral;
    return integral;
}

void InsideTrackPID::get_error(CarState &cs){
    int i;
    float total_slip = 0;
    float slip;

    for (i = 0; i < 4; i++) {
        total_slip += (cs.getSpeedX() - (cs.getWheelSpinVel(i) * WHEEL_RAD))/cs.getSpeedX();
    }

    slip = total_slip/4.0;

    current_error = REFERENCE_SLIP-slip;
}