/**  @file: FSMDriverABS.h
 *
 * https://github.com/bruno147/fsmdriver
 * 
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version. 
 */

#ifndef UNB_FSMDRIVER_FSMDRIVERABS_H
#define UNB_FSMDRIVER_FSMDRIVERABS_H

#include "FSMDriver3.h"
#include "InsideTrackABS.h"
#include <fstream>


class FSMDriverABS : public FSMDriver3{
public:
	/** Transitions between states. */
    /**
    *   Overrides de FSMDriver3 transition method to work with the inside track state with ABS implemented.
    *	@param cs a data structure cointaining information from the car's sensors.
    */
    FSMDriverABS();
};

#endif // FSMDriverABS_H