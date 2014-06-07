cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

option(USE_OPTRIS_SDK "Build with Optris compatability" TRUE)

IF(USE_OPTRIS_SDK)

	set(OptrisSDK "$ENV{USERPROFILE}/Documents/Optris/SDK" CACHE FILEPATH "Include directory for Optris SDK")

	IF(EXISTS "${OptrisSDK}")
		set(OptrisSDK_FOUND TRUE)
		MESSAGE(STATUS "Found OptrisSDK directory at specified path <${OptrisSDK}>.")
	ELSE()
		set(OptrisSDK_FOUND FALSE)
		MESSAGE(FATAL_ERROR "Unable to find OptrisSDK directory at specified path <${OptrisSDK}>.")
	ENDIF()

	 IF(OptrisSDK_FOUND)
		IF(WIN32 OR win64)
			IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
				MESSAGE(WARNING "You cannot build the Windows Optris demo app in 64-bit mode. Please switch to 32-bit mode for this functionality.")
			ELSE()
				add_subdirectory(win-optris)
			ENDIF()
		ELSE()
			MESSAGE(WARNING "No optris demo app has been developed for Linux yet.")
		ENDIF()
	ENDIF()
ENDIF()	
