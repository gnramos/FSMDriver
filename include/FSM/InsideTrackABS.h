/**  @file: InsideTrackABS.h
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#ifndef UNB_FSMDRIVER_STATE_INSIDE_TRACK_ABS_H
#define UNB_FSMDRIVER_STATE_INSIDE_TRACK_ABS_H

#include "InsideTrack.h"
#include <fstream>

/**
 * @brief InsideTrackABS state
 * @details  Uses the methods of FSM3's InsideTrack state with the addition of an ABS filter.
 */

class InsideTrackABS : public InsideTrack{
public:
	/** Multiply the the received value of brake,the value of 
    * car slip ratio and a constant that diminish the braking value, returning an smaller value of braking 
    * in an attempt to reduce the slip of wheels.
    *@param carState A CarState data structure that describes the car's perception of the environment by it's sensors information.
    *@param brake The standard braking value obtained from the controller.
    *@return A value for braking (interval 0-1);
    */
    float ABSfilter(float brake, CarState carState);

    /** override the get_brake method, with de addition of the ABS_filter method.
    *@param cs a data structure cointaining information from the car's sensors.
    *@return A value for braking, with an ABS filter for a decreased slip of wheels.
    */
    virtual float get_brake(CarState &cs);
    

    /** 
    * the minimum slip ratio that is needed in order to activate the ABS filter.
    */
	static const float ABS_SLIP;
    /**
    * the minimum speed of the car in order to activate de ABS filter.
    */
    static const float ABS_MINSPEED;
    /**
    * A constant that is used to diminish braking value.
    */
    static const float ABS_AT;
    /**
    * The radius of the car's wheels.
    */
    static const float WHEEL_RAD;
};


#endif // FSMDRIVER_STATE_INSIDETRACK-ABS_H