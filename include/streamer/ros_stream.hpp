/*! \file	ros_stream.hpp
 *  \brief	Declarations for ROS video input.
*/

#ifndef _THERMALVIS_ROS_STREAM_H_
#define _THERMALVIS_ROS_STREAM_H_

#ifdef _BUILD_FOR_ROS_
	
#include "streamer.hpp"
	
#include "time.h"

#include "cxcore.h"
#include "highgui.h"

#include <signal.h>

#include "ros_resources.hpp"
#include "tools.hpp"
#include "improc.hpp"
#include "radiometric.hpp"
#include "video.hpp"

#ifdef _BUILD_FOR_ROS_
#include <dynamic_reconfigure/server.h>
#include "streamerConfig.h"
#endif

// SERIAL COMMS STUFF
#include "serial_comms.hpp"



// END SERIAL COMMS STUFF


#ifdef _BUILD_FOR_ROS_
typedef dynamic_reconfigure::Server < thermalvis::streamerConfig > Server;
#endif

bool wantsToShutdown = false;
void mySigintHandler(int sig);

//HGH




#endif
#endif
