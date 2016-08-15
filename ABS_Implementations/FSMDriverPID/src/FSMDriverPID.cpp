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
