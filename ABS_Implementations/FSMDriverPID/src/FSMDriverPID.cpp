/**  @file: FSMDriverABS.cpp
 *
 * https://github.com/bruno147/fsmdriver
 * 
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version. 
 */

#include "FSMDriverPID.h"

FSMDriverPID::FSMDriverPID() : FSMDriver3(){
	delete inside_track;
	inside_track = new InsideTrackPID();
}

void FSMDriverPID::onShutdown(){
	int i;
	FSMDriver3:: onShutdown();
	InsideTrackPID *ins_track = (InsideTrackPID *) inside_track;

	ofstream logFile("log.txt", ios::app);

	if(!logFile){
        cout << "Impossível de gerar log";
    }
    else{
    	logFile << endl << endl;
        logFile << "List Of Errors: " << endl;
        for(i=0; i < ins_track->get_errors_index(); i++){
        	logFile << "Indice " << i << " = " <<(ins_track->get_errors())[i];
        	logFile << " Speed Error = " << (ins_track->get_speed_errors())[i];
        	logFile << " Brake = " << (ins_track->get_brake_values())[i] << endl;
        }
        logFile.close();
    }
}