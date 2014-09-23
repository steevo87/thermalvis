/*! \file	timing.hpp
 *  \brief	Declarations for timing analysis of chokepoints
*/

#ifndef THERMALVIS_TIMING_H
#define THERMALVIS_TIMING_H

#include "core/tools.hpp"

/// \brief		Used to analyze chokepoints in processing time
struct timeAnalyzer {
	
	int cycles;
	double vals[1024];
	double average;
	double sigma;
	
	struct timeval cycle_timer;
	
	timeAnalyzer();
	void calcParameters();
	void startRecording();
	void stopRecording();
	
};

#endif // THERMALVIS_TIMING_H
