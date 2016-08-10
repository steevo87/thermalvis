#include "core/timing.hpp"

timeAnalyzer::timeAnalyzer() {
	
	cycles = 0;
	average = 0.0;
	sigma = 0.0;
	
	ElapsedTimeMilliseconds(cycle_timer, true);
}

void timeAnalyzer::calcParameters() {
	if (cycles == 0) {
		return;
	}
	
	average = 0.0;
	sigma = 0.0;
	
	for (int iii = 0; iii < cycles; iii++) {
		average += vals[iii] / ((double) cycles);
	}
	
	for (int iii = 0; iii < cycles; iii++) {
		sigma += pow(vals[iii] - average, 2.0) / ((double) cycles);
	}
	
	sigma = pow(sigma, 0.5);
	
}

void timeAnalyzer::startRecording() {
	ElapsedTimeMilliseconds(cycle_timer, true);
}

void timeAnalyzer::stopRecording() {
	
	if (cycles < 1024) {
		vals[cycles] = ElapsedTimeMilliseconds(cycle_timer, true);
		cycles++;
	} else {
		ROS_ERROR("Too many cycles (%d) / (%d)", cycles, 1024);
	}
	
}
