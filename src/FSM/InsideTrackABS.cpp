/**  @file: InsideTrack.cpp
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include "InsideTrackABS.h"


const float InsideTrackABS::ABS_SLIP = 0.8;
const float InsideTrackABS::ABS_RNG = 0.8;
const float InsideTrackABS::ABS_MINSPEED = 3.0;    /* [m/s] */
const float InsideTrackABS::WHEEL_RAD = 0.4836;


float InsideTrackABS::ABSfilter(float brake, CarState carState){
        int i;
        float speedWheels = 0.0;
        float slip;

        if (carState.getSpeedX() < ABS_MINSPEED) return brake;

        for (i = 0; i < 4; i++) {
           	speedWheels += carState.getWheelSpinVel(i) * WHEEL_RAD/carState.getSpeedX();
       	}

        slip = speedWheels/4.0;
        //slip = carState.getSpeedX() - speedWheels;

        if (slip < ABS_SLIP) 
            brake = brake*slip*ABS_RNG;
    
    return brake;
}

float
InsideTrackABS::get_brake(CarState &cs){
    float brake = InsideTrack::get_brake(cs);
    return ABSfilter(brake == 0 ? 0.0: 1.0, cs);
}