cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

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

add_executable (OptrisWinTest ${WINOPTRIS_SOURCES} ${WINOPTRIS_HEADERS} )
target_link_libraries (OptrisWinTest ${ADDITIONAL_LIBS} thermalvis streamer)

IF(EXISTS "${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}.dll")
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_highgui${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_calib3d${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_features2d${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_flann${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_imgproc${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_objdetect${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_ml${OPENCV_VER}.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	IF (OPENCV_VIZ_FOUND)
		IF(NOT EXISTS "${OpenCV_BIN_DIR_OPT}/opencv_viz${OPENCV_VER}.dll")
			MESSAGE(FATAL_ERROR "OpenCV viz module was found, but the libraries don't appear to have been built correctly. Corrupt modules will adversely affect the buld process. Please either reconfigure OpenCV with the viz module deselected, or re-build the viz module. The most common reason for failure to build the opencv viz module is linking to the incorrect version of VTK (e.g. 64-bit instead of 32-bit).")
		ENDIF()
	ENDIF()
ELSE()
	MESSAGE(WARNING "<${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}.dll> unable to be located. Has OpenCV been built in Release mode?")
ENDIF()

IF(EXISTS "${OpenCV_BIN_DIR_DBG}/opencv_core${OPENCV_VER}d.dll")
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_core${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_highgui${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_calib3d${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_features2d${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_flann${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_imgproc${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_objdetect${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	ADD_CUSTOM_COMMAND(TARGET OptrisWinTest POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_ml${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:OptrisWinTest>)
	IF (OPENCV_VIZ_FOUND)
		IF(NOT EXISTS "${OpenCV_BIN_DIR_DBG}/opencv_viz${OPENCV_VER}d.dll")
			MESSAGE(FATAL_ERROR "OpenCV viz module was found, but the libraries don't appear to have been built correctly. Corrupt modules will adversely affect the buld process. Please either reconfigure OpenCV with the viz module deselected, or re-build the viz module. The most common reason for failure to build the opencv viz module is linking to the incorrect version of VTK (e.g. 64-bit instead of 32-bit).")
		ENDIF()
	ENDIF()
ELSE()
	MESSAGE(WARNING "<${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}d.dll> unable to be located. Has OpenCV been built in Debug mode?")
ENDIF()

