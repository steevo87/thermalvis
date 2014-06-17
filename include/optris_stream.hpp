/*! \file	optris_stream.hpp
 *  \brief	Declarations for optris video input.
*/

#ifndef _THERMALVIS_OPTRIS_STREAM_H_
#define _THERMALVIS_OPTRIS_STREAM_H_
	
#include "input_stream.hpp"

class winOptrisStream : public inputStream {

public:
	void assignMemoryToRawImage(int height, int width);
	void assignDataToRawImage(uchar *buff);
	bool getPauseMode();
	bool switchPauseMode();

	void setMinTemp(double temperature); 
	void setMaxTemp(double temperature); 

	void load_standard(int mapCode, int mapParam = 0);
	int get_colormap_index();
	void set_colormap_index(int index);
	void set_autoscaleTemps(bool setting);
};
	
#endif
