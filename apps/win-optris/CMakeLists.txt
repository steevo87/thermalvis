cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

SET(EXECUTABLE_NAME "OptrisWinTest")

# Flags necessary to get Managed Code functioning
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /clr")
STRING (REGEX REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
STRING (REGEX REPLACE "/EHsc" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)

project(win-optris)

set (ADDITIONAL_LIBS "")

INCLUDE_DIRECTORIES(${OptrisSDK})
LINK_DIRECTORIES(${OptrisSDK})
SET(ADDITIONAL_LIBS ${ADDITIONAL_LIBS} "ImagerIPC2.dll")

FILE(GLOB WINOPTRIS_HEADERS *.h*)
FILE(GLOB WINOPTRIS_SOURCES *.cpp)

INCLUDE_DIRECTORIES("../../include")
INCLUDE_DIRECTORIES("${OptrisSDK}/Samples VC++ .net/Shared Files/")

IF (NOT ("${_OpenCV_LIB_PATH}" STREQUAL ""))
	LINK_DIRECTORIES(${_OpenCV_LIB_PATH})
ENDIF()

add_executable (${EXECUTABLE_NAME} ${WINOPTRIS_SOURCES} ${WINOPTRIS_HEADERS} )
target_link_libraries (${EXECUTABLE_NAME} ${ADDITIONAL_LIBS} thermalvis "thermalvis-streamer")

SET(WINOPTRIS_EXTERNAL_LIBS "opencv" "qt" "boost")

# opencv_core, opencv_highgui, opencv_calib3d, opencv_features2d, opencv_flann, opencv_imgproc, opencv_objdetect, opencv_ml

foreach(EXTERNAL_LIB ${WINOPTRIS_EXTERNAL_LIBS}) 
	string(TOUPPER ${EXTERNAL_LIB} UPPERCASE_NAME)
	IF(USE_${UPPERCASE_NAME} AND ${EXTERNAL_LIB}_FOUND)
		SET(LINKING_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/link_${EXTERNAL_LIB}_dlls.cmake")
		IF(EXISTS "${LINKING_FILE}")
			include(${LINKING_FILE})
		ENDIF()
	ENDIF()
endforeach(EXTERNAL_LIB)
