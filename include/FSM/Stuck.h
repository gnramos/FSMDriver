/**  @file: Stuck.h
 *
 * https://github.com/bruno147/fsmdriver
 * 
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version. 
 */


#ifndef UNB_FSMDRIVER_STATE_STUCK_H
#define UNB_FSMDRIVER_STATE_STUCK_H

#include <cmath>

#include "DrivingState.h"

/**
 * @brief Stuck state.
 * @details Handles the driving when the car is stuck. This usually means it is stopped
 * or has been driving at a very low speed for a while, mesuared by slow_speed_ticks.
 * Entering this state mean that the driver may be hitting a wall or a corner. The 
 * stuck has  a limit time to work without going to another state. it is highly desireble
 * at a actual race that the drive do not enter at this state, since the pilot can complete 
 * the race faster if it do not happend..
 * 
 * @param stuck_speed Defines the threshold to monitorate if its in stuck.
 * @param minimum_distance_raced Just to avoid to enter in stuck at the beginning of race when the car is stopped.
 * @param maximum_number_of_ticks_stuck Maximum number of ticks in which reverse gear is allowed.
 * @param maximum_number_of_ticks_in_slow_speed Maximum number of ticks in low speed before stuck is triggered.
 */
class Stuck : public DrivingState {
public:
    /** Constructor.
     *
     * Call setParameters
     *
     * @param ss (stuck speed).
     * @param mrd (minimum distance raced).
     * @param mst (maximum number of_ticks stuck).
     * msst (maximum number of ticks in slow speed).
     *
     * Description    
     * can be found bellow.
     *
     * @see setParameters(float, int, int, int);
     */
    Stuck(float ss = 5.0, int mrd = 100, int mst = 300, int msst = 50);

    /** Defines the driving policy of the state. To work propertly for FSM3, the methos must be called in the order they're shown here.
    @param cs the driver's perception of the environment. 
    @return A CarControl with correct values for the actuators.
    */
    CarControl drive(CarState &);

    /**************************************************************************/

    /** Calculates the steering value based on the angle of the controller
    * in relation to the track and the track's initial position.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return -1 or 1 at stuck in order to fast turn to right way. */
    virtual float get_steer(CarState &cs);
    /** get_gear receives cs and returns the back gear
    * @param cs the driver's perception of the environment.
    * @return -1
    */
    virtual int get_gear(CarState &cs);
    /** get_accel recives cs and returns full accel
    *@param cs a data structure cointaining information from the car's sensors.
    *@return 1
    */
    virtual float get_accel(CarState &cs);
    /** get_brake recives cs and return no brake
    *@param cs a data structure cointaining information from the car's sensors.
    *@return 0 
    */
    virtual float get_brake(CarState &cs);


    /** Receives cs and returns the reverse gear.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return Always -1.*/
    virtual int get_gear(CarState &cs);

    /** Recives cs and returns full acceleration.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return Always 1.*/
    virtual float get_accel(CarState &cs);

    /** Recives cs and return no brake.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return Always 0.*/
    virtual float get_brake(CarState &cs);

    /** It recives the cs and calculates the clutch, always returning 0.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return Always 0. */
    virtual float get_clutch(CarState &cs);

    /**************************************************************************/

    /** Destructor. */
    ~Stuck();

    /** Indicates if the controller is stuck.
     *
     * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
     * @return True if the controller is stuck, false otherwise. */
    bool isStuck(CarState &cs);

    /** Auxiliar funcion to set class attributes*/
    void setParameters(float ss, int mrd, int mst, int msst);

private:
    /** Value which bellow it a car may be stuck.*/
    float stuck_speed;

    /** Distance raced that determine if the car is at begining of the race or not.*/
    unsigned int minimum_distance_raced;

    /** Determines the total time (by ticks) that stuck state can work without change to another state.*/
    unsigned int maximum_number_of_ticks_stuck;

    /** Value of ticks that the driver can be bellow 
    *the stuck_speed to determine if is stuck or not.*/
    unsigned int maximum_number_of_ticks_in_slow_speed;

    /** Checks if the stuck state is appropriate to the event at the race.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return True if the car has been at bellow the stuck_speed for long enough and false if not.*/
    bool seemsStuck(CarState &cs);

    /** Based on the distance raced it verifies if the driver has just started the race.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return True if the pilot is at begining of the race or false if not.*/
    bool justStartedRace(CarState &cs);

    /** Checks if the car is driving according the race direction.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return True if the car is facing the right race direction or face on the other way.*/
    bool onRightWay(float trackPos, float angle);

    /**  Based on the car's speed and stuck_speed this function determine if the car is stuck or not.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * is the sensors value of the angle between the car direction and race direction.
    * @return True if the car is not stuck anymore or false if it is.*/
    bool notStuckAnymore(CarState &cs);

    /** Since the driver can not be stuck for a long time without goint back the track, a time limit is used,
    * this checks if the stuck state surpass that limit.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return True if the driver pass the time limit or false if not.*/
    bool hasBeenStuckLongEnough();

    /**function to determine the steer.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return -1 or 1 at stuck in order to fast turn to right way.*/
    float auxSteer(float trackInitialPos, CarState &cs);

    /**Function to determine the track_pos at the begin of stuck state.
    * @param cs A data structure cointaining information from the car's sensors, the driver's perception of the environment.
    * @return The track_initial_pos or the current track position.*/
    float getInitialPos(CarState &cs);

    /**  Ticks'counter incremented when the car has been bellow the stuck_speed.*/
    unsigned int elapsed_ticks;

    /** Speed lowerlimit considered to increment the elapsed_ticks.*/
    unsigned int slow_speed_ticks;

    /** track_pos (sensor) value when the car enter the stuck state.*/
    float track_initial_pos;

};

#endif // UNB_FSMDRIVER_STATE_STUCK_H
