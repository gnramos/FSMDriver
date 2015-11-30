/**  @file: InsideTrack.h
 *
 * https://github.com/bruno147/fsmdriver
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#ifndef UNB_FSMDRIVER_STATE_INSIDE_TRACKA_H
#define UNB_FSMDRIVER_STATE_INSIDE_TRACKA_H

#include <cmath>
#include "DrivingState.h"

/**
 * Handles the driving when the car is within track limits.
 *
 * This means that the sensors track will return values > 0 inside the track. This
 * state is responsible for any event that occurs inside the track including
 * curves and straightline tracks.
 */
class InsideTrackA : public DrivingState {
public:
    /** Constructor.
     *
     * @param _sg The gear to be used at the begining of the race.
     * @param _lgl The rpm threshold value for changing low gears.
     * @param _lrpm The low rpm value that determine if the driver must change the gear.
     * @param _arpm The average rpm value that determine if the driver must change the gear.
     * @param _hrpm The high rpm value that determine if the driver must change the gear.
     * @param _bs A base speed factor for the driver.
     * @param _sf A proportional parameter to determine driver's speed
     *
     * @see setParameters(int, int, int, int, int, float, float);
     */
    InsideTrackA(int _sg = 1, int _lgl = 4, int _lrpm = 1500,
                int _arpm = 4000, int _hrpm = 9000, float _bs = 83,
                float _sf = 1.4);

    /** Defines a steering angle according to the car's perception of the environment.
    * Obtains the steering value by checking first if the driven is at the right direction, if not the steer is
    *calculated using angle value, case not using the distance (highest track sensor value)
    *@param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
    *@return A value for steering (between -ANGULO e ANGULO)*/
    virtual float get_steer(CarState &cs);

    /** Defines the gear value according to the car's perception of the environment.
    * It receives the gear from cs, based on it and rpm, it changes the gear
    * @param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
    * @return the a gear value accordingthe car's rpm */
    virtual int get_gear(CarState &cs);

    virtual float get_accel(CarState &cs);

    /** Defines the brake intensity according to the car's perception of the environment.
    * Calculates the brake based on the targetSpeed factor
    *@param cs A CarState data structure that describes the car's perception of the environment by it's sensors information.
    *@return 0 if the current speedX is lower the target speed and 0.3 if it higher the target speed*/
    virtual float get_brake(CarState &cs);

    virtual float get_clutch(CarState &cs);

    /** Faz alguma coisa * */
    void setParameters(int, int, int, int, int, float, float);
    //! Empty destructor
    ~InsideTrackA();

private:
    /** The gear used at the beginning of the race */
    int start_gear;

    /** The gear value that determine if the driver must decrease or increase gear based on gear*/
    int low_gear_limit;

    /**low_rpm is the rpm value that determine if the driver must decrease or increase gear based on the gear*/
    int low_rpm;

    /**average_rpm is the rpm value that determine if the driver must decrease or increase gear based on the rpm*/
    int average_rpm;

    /**high_rpm is the rpm value that determine if the driver must decrease or increase gear based on the rpm*/
    int high_rpm;

    /**base_speed is a parameter to determine the driver's speed*/
    float base_speed;

    /**speed_factor is a proportional parameter to determine driver's speed*/
    float speed_factor;

    /**current_speed is the driver's speed at the present code execution*/
    int current_gear;

    /** distance is the highest value of the 19 track sensors*/
    float distance;

    /** target_speed is the speed the car must reach, it is calculated based on distance, base_speed and speed_factor*/
    float target_speed;

    /** Check the current_gear and rpm, if the gear and rpm is above a certain value the function authorize decrease gear at one gear
    * @param current_gear the gear of the car at the moment of execution
    * @param rpm the value of rpm read by the sensor
    * @return true if the driver must decrease gear and false if it must not*/
    bool shouldDecreaseGear(int current_gear, int rpm);

    /** Check if rpm is bellow a certain value(low_rpm)
    * @param rpm the value of rpm read by the sensor
    * @return true if rpm is bellow low_rpm and false if it is not*/
    bool runningOnLow(int rpm);

    /** Check if rpm is bellow a certain value(average_rpm)
    * @param rpm the value of rpm read by the sensor
    * @return true if rpm is bellow average_rpm and false if it is not*/
    bool runningUnderAverage(int rpm);

    /** Check if rpm is above a certain value(high_rpm)
    * @param rpm the value of rpm read by the sensor
    * @return true if rpm is above high_rpm and false if it is not*/
    bool runningOnHigh(int rpm);

    /** Check if gear is between two value(start_gear and low_gear_limit)
    * @param rpm the value of rpm read by the sensor
    * @return true if gear is between start_gear and low_gear_limit, false if it is not*/
    bool isLowGear(int gear);

    /** Check if gear is above a certain value(low_gear_limit)
    * @param rpm the value of rpm read by the sensor
    * @return true if gear is above low_gear_limit, false if it is not*/
    bool isHighGear(int gear);

    /** Check the current_gear and rpm, if the gear and rpm is bellow a certain value the function authorize increase gear at one gear
    * @param current_gear the gear of the car at the moment of execution
    * @param rpm the value of rpm read by the sensor
    * @return true if the driver must increase gear and false if it must not*/
    bool shouldIncreaseGear(int current_gear, int rpm);

    /** Change the target_speed based on base_speed, speed_factor and distance
    *@param cs a data structure cointaining information from the car's sensors.*/
    void setTargetSpeed(CarState &cs);

    /** isFacingWrongWay verify if the car is driving the right path, once it is possible
    * that the car collide and turn to the opposite way.
    * @param cs a data structure cointaining information from the car's sensors.*/
    bool isFacingWrongWay(CarState &cs);

    /** Find the highest value of the 19 track sensors.
    *@param cs a data structure cointaining information from the car's sensors.
    *@return the index of the track sensor with highest value*/
    float findFarthestDirection(CarState &cs);

    /** It receive angle at radians (0.785398 to -0.785398) and normalize it turning -1 to 1
    *@param angle a data from the car's sensor angle.
    *@return a normalized value*/
    float normalizeSteer(float angle);
};

#endif // FSMDRIVER_STATE_INSIDETRACKA_H
