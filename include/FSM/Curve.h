#ifndef FSMDriver_STATE_CURVE_H
#define FSMDriver_STATE_CURVE_H

#include <cmath>
#include "FSM.h"
#include "StraightLine.h"
#include "Constants.h"


class FSMDriver;
/**Class to treat curve state where the track has a minimal curvature. Note that this state is important at the race considering oponents with equal speed the way the driver complete the curve can achieve the lead at the race.*/
class Curve : public DrivingState<FSMDriver> {
public:
    static Curve *instance();
    void enter(FSMDriver *driver);
    void exit(FSMDriver *driver);
    virtual CarControl drive(FSMDriver *FSMDriver, CarState &cs);

    ~Curve();

private:
    int currentGear;

    Curve();
    Curve(Curve const &);
    void operator=(Curve const&);

    float getAccel(CarState &cs);
    bool isFacingWrongWay(CarState &cs);
    float getBrake(CarState cs);
    float findFarthestDirection(CarState &cs);
    float normalizeSteer(float angle);/**Since the TORCS receive a value between -1 and 1 to the steer it used this function to normalize */
    float getSteer(CarState &cs);
};

#endif // FSMDriver_STATE_CURVE_H
