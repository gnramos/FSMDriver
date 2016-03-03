/**  @file: InsideTrack.cpp
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include "InsideTrackNPID.h"
#include <cmath>

const float InsideTrackNPID::WHEEL_RAD = 0.4836;
const float InsideTrackNPID::REFERENCE_SLIP = 0.2;
const float InsideTrackNPID::dt = 0.02;
const float InsideTrackNPID::Knp = -0.2995;
const float InsideTrackNPID::Tni = 0.279;
const float InsideTrackNPID::Tnd = 0.05; 
const float InsideTrackNPID::alpha_p = 0.3;
const float InsideTrackNPID::alpha_i = 0.27;
const float InsideTrackNPID::alpha_d = 0.15;
const float InsideTrackNPID::delta_p = 0.1;
const float InsideTrackNPID::delta_i = 0.1;
const float InsideTrackNPID::delta_d = 0.1;

float InsideTrackNPID::ABSfilter(CarState &carState){ 
    float brake;
    get_error(carState);

    brake = Knp * (nonlinear_function(current_error, alpha_p, delta_p) + 
        Tni * nonlinear_function(integral_of_error(), alpha_i, delta_i) + 
        Tnd * nonlinear_function(derivative_of_error(), alpha_d, delta_d));

    last_error = current_error;

    return brake;

}

float
InsideTrackNPID::get_brake(CarState &cs){
    float brake = InsideTrack::get_brake(cs);
    return brake == 0 ? reset_ABS() : ABSfilter(cs);
}

float InsideTrackNPID::reset_ABS(){
    last_integral = 0;

    return 0;
}

float InsideTrackNPID::derivative_of_error(){
    return (current_error - last_error)/dt;
}

float InsideTrackNPID::integral_of_error(){
    float integral = last_integral + (current_error * dt);
    last_integral = integral;
    return integral;
}

void InsideTrackNPID::get_error(CarState &cs){
    int i;
    float total_slip = 0;
    float slip;

    for (i = 0; i < 4; i++) {
        total_slip += (cs.getSpeedX() - (cs.getWheelSpinVel(i) * WHEEL_RAD))/cs.getSpeedX();
    }

    slip = total_slip/4.0;

    current_error = REFERENCE_SLIP-slip;
}

float InsideTrackNPID::nonlinear_function(float x, float alpha, float delta){
    if(abs(x) > delta){
        return copysign(1, x) * (pow(abs(x), alpha));
    }
    else{
        return pow(delta, alpha - 1) * x;
    }
}