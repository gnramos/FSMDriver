#include "InsideTrack.h"

// float SPEED_FACTOR = 1.4;

InsideTrack::InsideTrack(int _sg = 1, int _lgl = 4, int _lrpm = 1500,
                         int _arpm = 4000, int _hrpm = 9000, float _bs = 83, float _sf = 1.4) {
    START_GEAR = _sg;
    LOW_GEAR_LIMIT = _lgl;
    LOW_RPM = _lrpm;
    AVERAGE_RPM = _arpm;
    HIGH_RPM = _hrpm;
    BASE_SPEED = _bs;
    SPEED_FACTOR = _sf;
    currentGear = START_GEAR;
}

InsideTrack::InsideTrack(InsideTrack const &) {}

CarControl InsideTrack::drive(FSMDriver3 *fsmdriver3, CarState &cs) {
	float steer = getSteer(cs);
    setTargetSpeed(cs);
	int gear = getGear(cs);
	float accel  = getAccel(cs);
    float brake = getBrake(cs);
	float clutch = 0;
    // cout << "speed: " << this->targetSpeed << endl;
	return CarControl(accel, brake, gear, steer, clutch);
}

int InsideTrack::getGear(CarState &cs) {
    int gear = cs.getGear();
    if(gear <= 0) return START_GEAR;

    int rpm = cs.getRpm();

    if(shouldIncreaseGear(gear, rpm)) ++gear;
    else if(shouldDecreaseGear(gear, rpm)) --gear;

    return gear;
}

bool InsideTrack::shouldDecreaseGear(int currentGear, int rpm) {
    if(isLowGear(currentGear) && runningOnLow(rpm)) return true;
    if(isHighGear(currentGear) && runningUnderAverage(rpm)) return true;
    return false;
}

inline bool InsideTrack::runningOnLow(int rpm) {
    return (rpm < LOW_RPM);
}

inline bool InsideTrack::runningUnderAverage(int rpm) {
    return (rpm <= AVERAGE_RPM);
}

inline bool InsideTrack::runningOnHigh(int rpm) {
    return (rpm > HIGH_RPM);
}

inline bool InsideTrack::isLowGear(int gear) {
    return (gear > START_GEAR && gear < LOW_GEAR_LIMIT);
}

inline bool InsideTrack::isHighGear(int gear) {
    return (gear >= LOW_GEAR_LIMIT);
}

inline bool InsideTrack::shouldIncreaseGear(int currentGear, int rpm) {
    return runningOnHigh(rpm);
}

bool InsideTrack::isFacingWrongWay(CarState &cs) {
    return cs.getAngle() < -M_PI/2 || cs.getAngle() > M_PI/2;
}

float InsideTrack::getAccel(CarState &cs) { //@todo Change accelaration logic.
    return cs.getSpeedX() > targetSpeed ? 0:1;
}

float InsideTrack::getBrake(CarState cs) {
    return cs.getSpeedX() > targetSpeed ? 0.3:0;
}

void InsideTrack::setTargetSpeed(CarState &cs) {
    this->targetSpeed = BASE_SPEED + SPEED_FACTOR*this->distance;
}

float InsideTrack::findFarthestDirection(CarState &cs) {
    float farthestSensor = -INFINITY;
    float farthestDirection = 0;
       for (int i = 0; i < 19; i++) {
          if (farthestSensor < cs.getTrack(i)) {
            farthestSensor = cs.getTrack(i);
            farthestDirection = i;
        }
    }
    this->distance = farthestSensor;
    farthestDirection = -M_PI/2 + farthestDirection*M_PI/18;
    return normalizeSteer(-farthestDirection);
}

float InsideTrack::normalizeSteer(float angle) {
    const float MAXSTEER = 0.785398;
    return angle/MAXSTEER;
}

float InsideTrack::getSteer(CarState &cs) {
    return isFacingWrongWay(cs) ? cs.getAngle() : findFarthestDirection(cs);
}

InsideTrack::~InsideTrack() {}