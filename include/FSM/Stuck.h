#ifndef FSMDriver_STATE_STUCK_H
#define FSMDRIVER_STATE_STUCK_H

#include <cmath>
#include "Constants.h"
#include "FSM.h"


class FSMDriver;
/*! \class Stuck
 * \brief Stuck State Class.
 *
 * Class to treat stuck state when the car is stop or with low speed, it usually happen at track section with high curvature.
 */
class Stuck : public DrivingState<FSMDriver> {
public:
    /** Create a pointer to the state to accomplish the singleton.
    */
    static Stuck *instance();
    static inline bool isStuck(CarState &cs) {
	return (seemsStuck(cs) && !justStartedRace(cs));
    }
    /** Function to indicate that the drive started at Stuck State. 
    * \param driver is a pointer of the object of the driver itself. 
    */
    void enter(FSMDriver *driver, CarState &cs);
    /** Function to indicate that the drive leave the Stuck State. 
    * \param driver is a pointer of the object of the driver itself. 
    */
    void exit(FSMDriver *driver);
    /** Main function at state to drive the car.
    * /param driver is a pointer of the object of the driver itself,  
    * cs a data structure cointaining information from the car's sensors.
    */
    virtual CarControl drive(FSMDriver *FSMDriver, CarState &cs);
    ~Stuck();

private:
    unsigned int elapsedTicks;
    static unsigned int slowSpeedTicks;
    static float trackInitialPos;

    Stuck();
    Stuck(Stuck const &);
    void operator=(Stuck const&);
    

	float getSteer(float trackInitialPos, CarState &cs);
	float getInitialPos(CarState &cs);

	static inline bool seemsStuck(CarState &cs) {
		cs.getSpeedX()<STUCK_SPEED?slowSpeedTicks++:slowSpeedTicks = 0;
		if(notStuckAnymore(cs.getTrackPos(), cs.getAngle())){
			slowSpeedTicks=0;
		}
		return (slowSpeedTicks>MAX_SLOW_SPEED_TICKS?1:0);
	    //return (abs(cs.getSpeedX()) <= STUCK_SPEED);
	}

	static inline bool justStartedRace(CarState &cs) {
	    return (cs.getDistRaced() <= MIN_RACED_DISTANCE); 
	}
	
	static inline bool onRightWay(float trackPos, float angle) {
	    return (((trackPos < 0) && (angle > -1.57) && (angle < 0)) ||
	            ((trackPos > 0) && (angle < 1.57 ) && (angle > 0)) ||
	            ((trackPos > 1) && (angle > 0))||
	            ((trackPos < -1) && (angle < 0)));
	}
	/* @todo give this test (and the previous ones) with a meaningful name... */
	static inline bool notStuckAnymore(float trackPos, float angle) {
		return (onRightWay(trackPos, angle));
	}
	inline bool hasBeenStuckLongEnough() {
		return (elapsedTicks >= MAX_STUCK_TICKS);
	}
};

#endif // FSMDriver_STATE_STUCK_H
