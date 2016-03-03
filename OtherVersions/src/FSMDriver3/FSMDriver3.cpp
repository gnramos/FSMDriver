/**  @file: FSMDriver3.cpp
 *
 * https://github.com/bruno147/fsmdriver
 * 
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version. 
 */

#include "FSMDriver3.h"

/******************************************************************************/
#define NUM_SENSORS 19
/******************************************************************************/

int signum(float n) {
    if (n < 0) return -1;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//FSMDriver3 Class

/**
*FSMDriver3 Constructor: it initilize at straightline state in the begining of the race, here the parameters are set with fixed values.
*/
FSMDriver3::FSMDriver3() {
    inside_track = new InsideTrack();
    out_of_track = new OutOfTrack();
    stuck = new Stuck();
    changeTo(inside_track);
    tested = UNKNOWN;
}

void
FSMDriver3::onRestart() {
    cout << "Restarting the race!" << endl;
}

void
FSMDriver3::onShutdown() {
    cout << "End of race!" << endl;
    cout << this->trackName << endl;
    cout << "End of race!" << endl;

    ofstream logFile("log.txt", ios::trunc);

    if(!logFile){
        cout << "Impossível de gerar log.txt";
    }
    else{
        logFile << "Inside track: " << inside_track->get_ticks_in_state() << " ticks" << endl;
        logFile << "Outside track: " << out_of_track->get_ticks_in_state() << " ticks" << endl;
        logFile << "Stuck: " << stuck->get_ticks_in_state() << " ticks" << endl;
        logFile.close();
    }
}
/**
* Initializing the NUM_SENSORS track's angles using a gausian configuration, in order to make more sensors directed to the front of the car and consequently improve a curve detection 
*/
void
FSMDriver3::init(float *angles){
    for (int i = 0; i < NUM_SENSORS; ++i)
        angles[i]=signum((i*1.0/3)-3)*exp(-(0.5)*powf((((i+9)%18)*1.0/3)-3, 2))*90;
}

/** Parameters Evolved with Genetic Algorithm. */
void FSMDriver3::setROAD() {
        inside_track->setParameters(1, 2, 6956, 3728, 9411, 84.131, 0.973418);
        out_of_track->setParameters(32.0151, 0.146842, 165, 129, 368, 0.159876, 0.0839129);
        stuck->setParameters(5, 100, 300, 50);
}

/** Parameters Evolved with Genetic Algorithm. */
void FSMDriver3::setDIRT() {
        inside_track->setParameters(1, 4, 1796, 1857, 4340, 94.5951, 0.962757);
        out_of_track->setParameters(395.807, 0.0439577, 114, 113, 251, 0.0753426, 0.534217);
        stuck->setParameters(5, 100, 300, 50);
}

/** The transition choose the most fitted state at the moment of the race. */
void
FSMDriver3::transition(CarState &cs) {
    DrivingState *state = current_state;

    setTrackType();

    if(stuck->isStuck(cs)) {
        state = stuck;
    } else {
        if (cs.getTrack(1) > 0)
            state = inside_track;
        else {
            state = out_of_track;
        }
    }

    if (current_state != state) changeTo(state);
}

FSMDriver3::~FSMDriver3() {
    delete inside_track;
    delete out_of_track;
    delete stuck;
}


void FSMDriver3::setTrackType(){
    static bool flag = false;

    if((road_or_dirt == "ROAD") && (flag == false)) {
        setROAD();
        cout << "ROAD" << endl;
        flag = true;

        ofstream ordFile; // Auxiliar Output file road or dirt
        ordFile.open("road_dirt.txt", std::ios_base::trunc);
        ordFile << "ROAD" << endl;
        ordFile.close();
    }
    else if(road_or_dirt == "DIRT" && !flag) {
        setDIRT();
        cout << "DIRT" << endl;
        flag = true;

        ofstream ordFile;
        ordFile.open("road_dirt.txt", std::ios_base::trunc);
        ordFile << "DIRT" << endl;
        ordFile.close();
    }

    if (stage == BaseDriver::RACE || stage == BaseDriver::QUALIFYING || stage == BaseDriver::UNKNOWN) {
        ifstream irdFile; // Auxiliar Input file road or dirt
        string line;
        irdFile.open("road_dirt.txt", ios::in);
        getline(irdFile, line);
        // cout << line << endl;
        irdFile.close();

        if(line == string("ROAD"))  setROAD();
        else if(line == string("DIRT")) setDIRT();
    }
}

