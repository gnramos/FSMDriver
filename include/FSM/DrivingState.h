/**  @file: DrivingState.h
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#ifndef UNB_FSMDRIVER_DRIVING_STATE_H
#define UNB_FSMDRIVER_DRIVING_STATE_H

#include "CarControl.h"
#include "CarState.h"

class FSMDriver;

/** Abstract class defining a state for a Finite State Machine controller. */
class DrivingState {
public:
	/** Construtor.
     */
	DrivingState();

	/** Virtual destructor. */
    virtual ~DrivingState();

    /** Defines the driving policy of the state.
     *
     * @param cs the driver's perception of the environment. */
    virtual CarControl drive(CarState &cs);

    /** Called when entering the state. */
    virtual void enter();

    /** Called when exiting the state. */
    virtual void exit();


    int get_ticks_in_state();

    void add_ticks_in_state();


/**************************************************************************
 * Modularization*/

/* Modular virtual methods, should be implemented in each state.*/
protected:
    int ticks_in_state;

    /** Defines the steering angle value output according to the car's perception of the environment.
    * 
    * @param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
    * @return The steering angle value output [-1, 1] full right and full left respectively.*/
    virtual float get_steer(CarState &cs) = 0;

    /** Determines the gear value according to the car's perception of the environment.
    *
    * @param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
    * @return The gear value [-1, 0, 1, ..., 6], which -1 is reverse and 0 is neutral.*/
    virtual int get_gear(CarState &cs) = 0;

    /** Defines the intensity of the virtual gas pedal  according to the car's perception of the environment.
     *
     * @param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
     * @return The acceleration intensity [0, 1], 0 means no gas, 1 full gas. */
    virtual float get_accel(CarState &cs) = 0;

    /** Defines the intensity of the virtual break pedal  according to the car's perception of the environment.
     *
     * @param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
     * @return The break intensity [0, 1], 0 means no break, 1 full break. */
    virtual float get_brake(CarState &cs) = 0;

    /** Defines the intensity of the virtual clutch pedal  according to the car's perception of the environment.
     *
     * @param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
     * @return The clutch intensity [0, 1], 0 means no clutch, 1 full clutch. */
    virtual float get_clutch(CarState &cs) = 0;
};

#endif // UNB_FSMDRIVER_DRIVING_STATE_H
