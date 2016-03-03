/**  @file: InsideTrack.h
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#ifndef UNB_FSMDRIVER_STATE_INSIDE_TRACK_NPID_H
#define UNB_FSMDRIVER_STATE_INSIDE_TRACK_NPID_H

#include "InsideTrack.h"
#include <fstream>
#include <cmath>

class InsideTrackNPID : public InsideTrack{
public:
    /**
    * The radius of the car's wheels.
    */
    static const float WHEEL_RAD;
    /**
    * The slip ratio that we want the car to maintain.
    */
    static const float REFERENCE_SLIP;
    /**
    * the simulation step.
    */
    static const float dt;
    /**
    * The proportional gain of the NPID controller.
    */
    static const float Knp;
    /**
    * The integral time constant of the NPID controller.
    */
    static const float Tni;
    /**
    * The derivative time constant of the NPID controller.
    */
    static const float Tnd;
    /**
    * Parameter of the nonlinear function for the proportional part of the NPID controller.
    */
    static const float alpha_p;
    /**
    * Parameter of the nonlinear function for the integral part of the NPID controller.
    */
    static const float alpha_i;
    /**
    * Parameter of the nonlinear function for the derivative part of the NPID controller.
    */
    static const float alpha_d;
    /**
    * Parameter of the nonlinear function for the proportional part of the NPID controller.
    */
    static const float delta_p;
    /**
    * Parameter of the nonlinear function for the integral part of the NPID controller.
    */
    static const float delta_i;
    /**
    * Parameter of the nonlinear function for the derivative part of the NPID controller.
    */
    static const float delta_d;

	/**
    * Uses a non-linear PID controller(NPID) in order to maintain the car's slip of wheels in a certain reference slip.
    * @param carSate A data structure cointaining information from the car's sensors.
    * @return A value for braking with the ABS NPID controller.
    */
    float ABSfilter(CarState &carState);


    /**
    * Sets the atributes of the states to a initial condition (before a new braking action).
    * @return 0, indicating that the car is not braking at this moment and the state's atributes were reset.
    */
    float reset_ABS();


    /** override the get_brake method, with de addition of the ABS filter.
    *@param cs a data structure cointaining information from the car's sensors.
    *@return A value for braking, with an ABS NPID controller for a decreased slip of wheels.
    */
    virtual float get_brake(CarState &cs);

private:
    float current_error;
    float last_error;
    float last_integral;

    /** Calculates the integral of the error between the reference and the currently slip ratio, needed in the ABS controller.
    *@return A value that represents the integral of the error for the NPID controller.
    */
    float integral_of_error();
    /** Calculates the derivative of the error between the reference and the currently slip ratio, needed in the ABS controller.
    *@return A value that represents the derivative of the error for the NPID controller.
    */
    float derivative_of_error();
    /** Calculates the error between the reference and the currently slip ratio, needed in the ABS controller.
    *@return A value that represents the error for the NPID controller.
    */
    void get_error(CarState &cs);

    /** A nonlinear function used in the non-linear PID controller. 
    * @return if |x| > delta, returns sign(x) * |x|^alpha, else, returns delta^(alpha - 1) * x.
    */
    float nonlinear_function(float x, float alpha, float delta);
};


#endif // FSMDRIVER_STATE_INSIDETRACK-ABS_H