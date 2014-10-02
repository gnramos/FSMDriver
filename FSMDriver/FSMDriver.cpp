#include "FSMDriver.h"

#include "StraightLine.h"
typedef StraightLine StateStraightLine;
#include "Curve.h"
typedef Curve StateCurve;
#include "OutOfTrack.h"
typedef OutOfTrack StateOutOfTrack;

//-------------------------------------------------------------------------------------------------------------------
//FsmDriver Class

FsmDriver::FsmDriver() {
    this->accel = 0;
    this->brake = 0;
    this->steer = 0;
    this->gear = 0;
}

CarState& FsmDriver::getCarState() {
    return this->_cs;
}

void FsmDriver::setCarState(CarState cs) {
    this->_cs = cs;
}

State* FsmDriver::getState() {
    return this->_state;
}

void FsmDriver::SetState(State *_newState) {
    delete this->_state;
    this->_state = _newState;
}

float FsmDriver::getAccel() {
    return this->accel;
}

void FsmDriver::setAccel(float accel) {
    this->accel = accel;
}

float FsmDriver::getBrake() {
    return this->brake;
}

void FsmDriver::setBrake(float brake) {
    this->brake = brake;
}

float FsmDriver::getSteer() {
    return this->steer;
}

void FsmDriver::setSteer(float steer) {
    this->steer = steer;
}

int FsmDriver::getGear() {
    return this->gear;
}

void FsmDriver::setGear(int gear) {
    this->gear = gear;
}

CarControl FsmDriver::wDrive(CarState cs) {
    this->setCarState(cs);
    this->SetState(transition(this->getCarState()));
    return (this->getState())->execute(this);
}

void FsmDriver::onRestart() {
    cout << "Restarting the race!" << endl;
}

void FsmDriver::onShutdown() {
    cout << "End of race!" << endl;
}

void FsmDriver::init(float *angles){
    for (int i = 0; i < 19; ++i)
        angles[i]=-90+i*10;
}

State* FsmDriver::transition(CarState &cs) {
    State* p;
    if(cs.getTrackPos() > -1 && cs.getTrackPos() < 1) {
        // Getting track information from the sensor at +5 degrees towards the car axis
        float rSensor = cs.getTrack(10);
        // Getting track information from the sensor parallel to the car axis
        float cSensor = cs.getTrack(9);
        // Getting track information from the sensor at -5 degrees towards the car axis
        float lSensor = cs.getTrack(8);
        // Characteristics of a 'straight' to the FSM:
        // 		- If the central sensor is beyond the distance of maximum speed or if it
        // 		the biggest of {central, right (+5 degrees), left (-5 degrees)} sensors
        if (cSensor > 70 || (cSensor >= rSensor && cSensor >= lSensor)) {
            p = new StraightLine;
            cout << "Straight" << endl;
            return p;
        }
        // Characteristics of a 'curve' to the FSM
        else {
            p = new Curve;
            cout << "Curve" << endl;
            return p;
        }
    }
    // Characteristics of 'outside of track' states
    else {
        // Returns
        cout << "OutOfTrack" << endl;
        p = new OutOfTrack;
        return p;
    }
}