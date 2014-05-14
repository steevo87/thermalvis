/*! \file	ffmpeg_resources.hpp
 *  \brief	For including FFMPEG related requirements.
*/

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	// ..
#else
	#ifndef _THERMALVIS_FFMPEG_RESOURCES_H_
	#define _THERMALVIS_FFMPEG_RESOURCES_H_

	#include <asm/types.h>          /* for videodev2.h */

	/***** Add FFMPEG AVI Stuff *****/
	#ifndef INT64_C
	#	define INT64_C(c) (c ## LL)
	#	define UINT64_C(c) (c ## ULL)
	#endif

	extern "C" {
	#include <linux/videodev2.h>
	#include <libavcodec/avcodec.h>
	#include <libavformat/avformat.h>
	#include <libavdevice/avdevice.h>
	#include <libswscale/swscale.h>
	}

	#endif
#endif


