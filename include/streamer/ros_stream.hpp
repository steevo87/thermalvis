/*! \file	ros_stream.hpp
 *  \brief	Declarations for ROS video input.
*/

#ifndef _THERMALVIS_ROS_STREAM_H_
#define _THERMALVIS_ROS_STREAM_H_

#ifdef _BUILD_FOR_ROS_

#include "input_stream.hpp"	

#include "time.h"

#include "cxcore.h"
#include "highgui.h"

#include <signal.h>

#include "ros_resources.hpp"
#include "tools.hpp"
#include "improc.hpp"
#include "radiometric.hpp"

#ifdef _AVLIBS_AVAILABLE_
#include "video.hpp"
#endif

#include <dynamic_reconfigure/server.h>
#include "streamerConfig.h"

#include "serial_comms.hpp"

typedef dynamic_reconfigure::Server < thermalvis::streamerConfig > Server;

bool wantsToShutdown = false;
void mySigintHandler(int sig);

#endif
#endif
