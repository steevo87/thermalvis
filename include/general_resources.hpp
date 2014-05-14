/*! \file	general_resources.hpp
 *  \brief	For generic requirements of all (or most) nodes.
*/

#ifndef _THERMALVIS_GENERAL_RESOURCES_H_
#define _THERMALVIS_GENERAL_RESOURCES_H_

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	// ..
#else
	#include <unistd.h>
	#include <getopt.h>
	#include <dirent.h>
	#include <termios.h>
	#include <sys/time.h>
	#include <sys/mman.h>
	#include <sys/ioctl.h>
#endif

/***** General includes *****/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <sstream>
#include <string>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */

#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>




// Standard
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>



#include <sys/types.h>


#include <signal.h>

//using namespace std;

#endif
