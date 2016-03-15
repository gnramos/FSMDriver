/**  @file: InsideTrack.h
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#ifndef UNB_FSMDRIVER_STATE_INSIDE_TRACK_H
#define UNB_FSMDRIVER_STATE_INSIDE_TRACK_H

#include <cmath>
#include "DrivingState.h"

/**
 * @brief InsideTrack state
 * @details  Handles the driving when the car is within track limits, which mean that the the
 *           sensors track wil return values > 0 inside the track. This state is responsible
 *           for any event that occur inside the track including curves and straightline tracks.
 * 
 * @param start_gear the gear used at the begining of the race
 * @param low_gear_limit Threshlod to bound low gears.
 * @param low_rpm Threshlod of rpm to delimit the change of low gears.
 * @param average_rpm Threshlod to decrease high gears.
 * @param high_rpm Threshlod to increase high gears.
 * @param base_speed Proportionality between highest value read by range finders and TARGET SPEED.
 * @param speed_factor Lowest speed allowed.
 *
 */
class InsideTrack : public DrivingState {
public:
    /** Constructor.
     *
     * @param _sg (start_gear).
     * @param _lgl (low_gear_limit).
     * @param _lrpm (low_rpm).
     * @param _arpm (average_rpm).
     * @param _hrpm (high_rpm).
     * @param _bs (base_speed).
     * @param _sf (speed_factor).
     *
     * Description    
     * can be found bellow.
     *
     * @see setParameters(int, int, int, int, int, float, float);
     */
    InsideTrack(int _sg = 1, int _lgl = 4, int _lrpm = 1500,
                int _arpm = 4000, int _hrpm = 9000, float _bs = 83,
                float _sf = 1.4);

    /** Defines the driving policy of the state. To work propertly for FSM3, the methos must be called in the order they're shown here.
    @param cs the driver's perception of the environment. 
    @return A CarControl with correct values for the actuators.
    */
    CarControl drive(CarState &);
    
    /**************************************************************************
    * Modularization*/

    /** Obtains the steering value by checking first if the driven is at the right direction, if not the steer is
    *calculated using angle value from cs, case not using the distance (highest track sensor value).
    *@param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    *@return A normalized steer value.*/
    virtual float get_steer(CarState &cs);

    /** It receives the gear from cs, based on it and rpm it decreases or increases the gear.
    *@param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return The gear value accordingthe car's rpm.*/
    virtual int get_gear(CarState &cs);

    /** It sets the target_speed based on cs, and decides if should applies full gas or no gas.
    *@param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    *@return 0 if the current speed is higher than the target_speed and 1 if it is lower than the target_speed.*/
    virtual float get_accel(CarState &cs);

    /** It sets the target_speed based on cs, and decides if should applies 0.3 break intesity or no break.
    *@param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    *@return 0.3 if the current speed is higher than the target_speed and 0 if it is lower than the target_speed.*/
    virtual float get_brake(CarState &cs);

    /** It recives the cs and calculates the clutch, always returning 0.
    *@param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    *@return Always 0. */
    virtual float get_clutch(CarState &cs);

    /**************************************************************************/

    /** Auxiliar funcion to set class attributes*/
    void setParameters(int, int, int, int, int, float, float);
    //! Empty destructor
    ~InsideTrack();

private:
    /** Gear used at the begining of the race*/
    int start_gear;

    /** Gear value that determine if the driver must decreace or increace gear based on gear.*/
    int low_gear_limit;

    /** Engine rotation value that determine if the driver must decreace or increace gear based on the gear.*/
    int low_rpm;

    /** Engine Rotation value that determine if the driver must decreace or increace gear based on the rpm.*/
    int average_rpm;

    /** Engine rotation value that determine if the driver must decreace or increace gear based on the rpm.*/
    int high_rpm;

    /** Parameter to determine if the driver's speed. */
    float base_speed;

    /** A proportional parameter to determine driver's speed.*/
    float speed_factor;

    /** The driver's speed at the present code execution.*/
    int current_gear;

    /** The highest value of the 19 track sensors.*/
    float distance;

    /** The speed the car must reach, it is calculated based on distance, base_speed and speed_factor.*/
    float target_speed;

    /** Checks the current_gear and rpm, if the gear and rpm is above a certain value the function authorizes to decrease gear.
    * @param current_gear the gear of the car at the moment of execution.
    * @param rpm the value of the engine rotation read by the sensor.
    * @return True if the driver must decrease gear and false if it must not.*/
    bool shouldDecreaseGear(int current_gear, int rpm);

    /** Checks if rpm is bellow a certain value (low_rpm).
    * @param rpm the value of rpm read by the sensor.
    * @return True if rpm is bellow low_rpm and false if it is not.*/
    bool runningOnLow(int rpm);

    /** Checks if rpm is bellow a certain value (average_rpm).
    * @param rpm the value of the engine rotation read by the sensor.
    * @return True if rpm is bellow average_rpm and false if it is not.*/
    bool runningUnderAverage(int rpm);

    /** Checks if rpm is above a certain value (high_rpm).
    * @param rpm the value of the engine rotation read by the sensor.
    * @return True if rpm is above high_rpm and false if it is not.*/
    bool runningOnHigh(int rpm);

    /** Checks if gear is between two values (start_gear and low_gear_limit).
    * @param rpm the value of the engine rotation read by the sensor.
    * @return True if gear is between start_gear and low_gear_limit, false if it is not.*/
    bool isLowGear(int gear);

    /** Checks if gear is above a certain value (low_gear_limit).
    * @param rpm the value of the engine rotation read by the sensor.
    * @return True if gear is above low_gear_limit, false if it is not.*/
    bool isHighGear(int gear);

    /** Checks the current_gear and rpm, if they are bellow a certain value the function authorizes to increase.
    * @param current_gear the gear of the car at the moment of execution.
    * @param rpm the value of the engine rotation read by the sensor.
    * @return True if the driver must increase gear and false if it must not.*/
    bool shouldIncreaseGear(int current_gear, int rpm);

    /** Changes the target_speed based on base_speed, speed_factor and distance.
    * @param cs a data structure cointaining information from the car's sensors.*/
    void setTargetSpeed(CarState &cs);

    /** Verifies if the car is driving the right path, once it is possible.
    * that the car collide and turn to the opposite way.
    * @param cs a data structure cointaining information from the car's sensors.*/
    bool isFacingWrongWay(CarState &cs);

    /** Find the highest value of the 19 track sensors.
    * @param cs a data structure cointaining information from the car's sensors.
    * @return the index of the track sensor with highest value.*/
    float findFarthestDirection(CarState &cs);

    /** It receive angle at radians (0.785398, -0.785398) and normalize it to -1 and 1.
    * @param angle a data from the car's sensor angle.
    * @return a normalized value.*/
    float normalizeSteer(float angle);

};

#endif // FSMDRIVER_STATE_INSIDETRACK_H
