/**  @file: InsideTrack.h
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

class InsideTrackABS : public InsideTrack{
public:
	/**
    * @brief Filters the braking value based on the cars data about the four wheels spins.
    * 
    */
    float ABSfilter(float brake, CarState carState);

    /** override the get_brake method, with de addition of the ABS.
    *@param cs a data structure cointaining information from the car's sensors.
    *@return 0 if the current speedX is lower the target speed and 0.3 if it higher the target speed
    */
    virtual float get_brake(CarState &cs);
    
	static const float ABS_SLIP;
    static const float ABS_MINSPEED;
    static const float ABS_RNG;
    static const float WHEEL_RAD;
};


#endif // FSMDRIVER_STATE_INSIDETRACK-ABS_H