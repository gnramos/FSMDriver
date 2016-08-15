/**  @file: InsideTrack.h
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#ifndef UNB_FSMDRIVER_STATE_INSIDE_TRACK_PID_H
#define UNB_FSMDRIVER_STATE_INSIDE_TRACK_PID_H

#include "InsideTrack.h"
#include <fstream>

class InsideTrackPID : public InsideTrack{
public:
    static const float WHEEL_RAD;
    static const float REFERENCE_SLIP;
    static const float dt;
    static const float Kp;
    static const float Ti;
    static const float Td;

	/**
    * Uses a PID controller in order to maintain the car's slip of wheels in a certain reference slip.
    * @param carSate A data structure cointaining information from the car's sensors.
    * @return A value for braking with the ABS PID controller.
    */
    float ABSfilter(CarState &carState);


    /**
    * Sets the atributes of the states to a initial condition (before a new braking action).
    * @return 0, indicating that the car is not braking at this moment and the state's atributes were reset.
    */
    float reset_ABS();


    /** override the get_brake method, with de addition of the ABS.
    *@param cs a data structure cointaining information from the car's sensors.
    *@return A value for braking, with an ABS PID controller for a decreased slip of wheels.
    */
    virtual float get_brake(CarState &cs);

private:
    float current_error;
    float last_error;
    float last_integral;

    /** Calculates the integral of the error between the reference and the currently slip ratio, needed in the ABS controller.
    *@return A value that represents the integral of the error for the PID controller.
    */
    float integral_of_error();
    /** Calculates the derivative of the error between the reference and the currently slip ratio, needed in the ABS controller.
    *@return A value that represents the derivative of the error for the PID controller.
    */
    float derivative_of_error();
    /** Calculates the error between the reference and the currently slip ratio, needed in the ABS controller.
    *@return A value that represents the error for the PID controller.
    */
    void get_error(CarState &cs);
};


#endif // FSMDRIVER_STATE_INSIDETRACK-ABS_H