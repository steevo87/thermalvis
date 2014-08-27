/*! \file	streamer.hpp
 *  \brief	Declarations for the ROS <streamer> node.
*/

#ifndef _STREAMER_H_
#define _STREAMER_H_

#include "streamer/input_stream.hpp"

#include <dynamic_reconfigure/server.h>
#include "streamerConfig.h"

typedef dynamic_reconfigure::Server < thermalvis::streamerConfig > Server;

bool wantsToShutdown = false;
void mySigintHandler(int sig);

boost::shared_ptr < streamerNode > *globalNodePtr;

#endif
