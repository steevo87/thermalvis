cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

IF(WIN32 OR win64)
	IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
		MESSAGE(WARNING "You cannot build the Windows Optris demo app in 64-bit mode. Please switch to 32-bit mode for this functionality.")
	ELSE()
		add_subdirectory(win-optris)
	ENDIF()
ENDIF()

