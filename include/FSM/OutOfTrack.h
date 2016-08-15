/**  @file: OutOfTrack.h
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */


#ifndef UNB_FSMDRIVER_STATE_OUT_OF_TRACK_H
#define UNB_FSMDRIVER_STATE_OUT_OF_TRACK_H

#include <cmath>
#include "DrivingState.h"

/**
 * @brief OutOfTrack state.
 * @details Handles the driving when the car is outside track limits. Since the outside the track the track's type change, like dirt road,
 * the driver losses performance that state manage to take the drive out of the track.
 *
 * @param max_skidding Defines the threshold to start to break to avoid skidding.
 * @param negative_accel_percent Dictate how much to release the acceleration pedal to avoid to skidding,
 *                               it is defined with the axis speed of car.
 * @param velocity_gear_4 Threshold to change the gear to 4.
 * @param velocity_gear_3 Threshold to change the gear to 3.
 * @param velocity_gear_2 Threshold to change the gear to 2.
 * @param max_return_angle Upper boundary to angle of return to track.
 * @param min_return_angle Lower boundary to angle of return to track.
 */
class OutOfTrack : public DrivingState {
public:
    /** Constructor.
     *
     * Call setParameters
     * @param _ms (max_skidding)
     * @param _nap (negative_accel_percent)
     * @param _vg4 (velocity_gear_4)
     * @param _vg3 (velocity_gear_3)
     * @param _vg2 (velocity_gear_2)
     * @param maxra (max_return_angle)
     * @param minra (min_return_angle)
     *
     * Description    
     * can be found bellow.
     *
     * @see setParameters(float, float, int, int, int, float, float);
     */
    OutOfTrack(float _ms = 3, float _nap = 0.1, int _vg4 = 90,
               int _vg3 = 70, int _vg2 = 40, float _maxra = 0.7,
               float _minra = 0.5);

    /** Defines the driving policy of the state. To work propertly for FSM3, the methos must be called in the order they're shown here.
    @param cs the driver's perception of the environment. 
    @return A CarControl with correct values for the actuators.
    */
    CarControl drive(CarState &);

    /**************************************************************************/

    /** Obtains the steering value based on the trackPos sensor from cs, that allow to know which track 
    * border the car is, and angle sensor that allow to know which side to turn the steer.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return -1 or 1 to Steer value, full right and full left respectively, according to the track border. */
    virtual float get_steer(CarState &cs);

    /** It receives cs and calculates the gear based on the current speed and the speed limits to each gear, that way
    * high speed need high gear. Please note that this state does not use rpm to obtain gear.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return The gear value accordingthe car's current speed.*/
    virtual int get_gear(CarState &cs);

    /** It calculates the acceleration based on the speedY from cs and the proportinal factor negative_accel_percent.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return Virtual gas pedal intensity proportional to the speedY.*/
    virtual float get_accel(CarState &cs);

    /** It calculates the brake based on the speedX, speedY from cs and max_skidding.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return 0.1 if the car's speedY is higher than max_skidding,
    * 1 when the car's speedX is negative, and 0 for other cases.*/
    virtual float get_brake(CarState &cs);

    /** It receives the cs and calculates the clutch, always returning 0.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return Always 0. */
    virtual float get_clutch(CarState &cs);

    /**************************************************************************/

    /** Auxiliar funcion to set class attributes*/
    void setParameters(float, float, int, int, int, float, float);
    //! Empty destructor
    ~OutOfTrack();

private:
    /** A constant to indicated how much the drive must brake, measured by speedY.*/
    float max_skidding;

    /** Proportinal factor to calculate acceleration.*/
    float negative_accel_percent;

    /** A constant that determines a speed which above it the gear must remain the same.*/
    int velocity_gear_4;

    /** A constant that determines a speed which above it and bellow velocity_gear_3 the gear must be set 3.*/
    int velocity_gear_3;

    /** A constant that determines a speed which bellow it must be set 1.*/
    int velocity_gear_2;

    /** The a angle limit that turning steer to -1 or 1 depending the TrackPos sensor.*/
    float max_return_angle;

    /** The a angle limit that turning steer to -1 or 1 depending the TrackPos sensor.*/
    float min_return_angle;
    
};

#endif // UNB_FSMDRIVER_STATE_OUT_OF_TRACK_H
