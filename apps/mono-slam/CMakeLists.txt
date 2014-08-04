SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

IF(BUILD_GUI)
	# Find includes in corresponding build directories
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	# Instruct CMake to run moc automatically when needed.
	set(CMAKE_AUTOMOC ON)
ENDIF()

IF(WIN32 OR win64)
	SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)
ELSE()
	SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
ENDIF()
 
project(mono-slam)

ADD_DEFINITIONS( -D_INCLUDE_INTERFACING_ )

set (ADDITIONAL_LIBS "")

FILE(GLOB LOCAL_HEADERS *.h*)
FILE(GLOB LOCAL_SOURCES *.cpp)

INCLUDE_DIRECTORIES("../../include")
INCLUDE_DIRECTORIES(../../include/streamer)
INCLUDE_DIRECTORIES(../../include/flow)
INCLUDE_DIRECTORIES(../../include/slam)

IF(BUILD_GUI)
	INCLUDE_DIRECTORIES("${QTSTREAMER_BUILD_PATH}")
	INCLUDE_DIRECTORIES("../../qt/streamer")
	INCLUDE_DIRECTORIES("${QTFLOW_BUILD_PATH}")
	INCLUDE_DIRECTORIES("../../qt/flow")
	INCLUDE_DIRECTORIES("${QTSLAM_BUILD_PATH}")
	INCLUDE_DIRECTORIES("../../qt/slam")
	LIST(APPEND ADDITIONAL_LIBS Qt5::Widgets)
ENDIF()

LIST(APPEND ADDITIONAL_LIBS ${OpenCV_LIBRARIES})
LIST(APPEND ADDITIONAL_LIBS ${Boost_LIBRARIES})

LIST(APPEND ADDITIONAL_LIBS thermalvis)
LIST(APPEND ADDITIONAL_LIBS streamer)
LIST(APPEND ADDITIONAL_LIBS flow)
LIST(APPEND ADDITIONAL_LIBS slam)

SET(EXECUTABLE_NAME "MonocularSLAM")
IF(BUILD_GUI)

	IF(NOT EXISTS "${QTSTREAMER_BUILD_PATH}/ui_mainwindow_streamer.h")
		MESSAGE(FATAL_ERROR "<ui_mainwindow_streamer.h> could not be found in <${QTSTREAMER_BUILD_PATH}>. Has Qt Creator been used to generate this file? Or, is the CMake variable incorrect?")
	ENDIF()
	
	IF(NOT EXISTS "${QTFLOW_BUILD_PATH}/ui_mainwindow_flow.h")
		MESSAGE(FATAL_ERROR "<ui_mainwindow_flow.h> could not be found in <${QTFLOW_BUILD_PATH}>. Has Qt Creator been used to generate this file? Or, is the CMake variable incorrect?")
	ENDIF()
	
	IF(NOT EXISTS "${QTSLAM_BUILD_PATH}/ui_mainwindow_slam.h")
		MESSAGE(FATAL_ERROR "<ui_mainwindow_slam.h> could not be found in <${QTSLAM_BUILD_PATH}>. Has Qt Creator been used to generate this file? Or, is the CMake variable incorrect?")
	ENDIF()

	add_executable(${EXECUTABLE_NAME} ${LOCAL_SOURCES} ${LOCAL_HEADERS} 
	../../qt/streamer/mainwindow_streamer.cpp 
	../../qt/streamer/mainwindow_streamer.h 
	${QTSTREAMER_BUILD_PATH}/ui_mainwindow_streamer.h
	../../qt/flow/mainwindow_flow.cpp 
	../../qt/flow/mainwindow_flow.h 
	${QTFLOW_BUILD_PATH}/ui_mainwindow_flow.h
	../../qt/slam/mainwindow_slam.cpp 
	../../qt/slam/mainwindow_slam.h 
	${QTSLAM_BUILD_PATH}/ui_mainwindow_slam.h)
ELSE()
	add_executable(${EXECUTABLE_NAME} ${LOCAL_SOURCES} ${LOCAL_HEADERS} )
ENDIF()
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${ADDITIONAL_LIBS} )

IF(EXISTS "${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}.dll")
	
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_highgui${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_calib3d${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_features2d${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_flann${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_imgproc${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_objdetect${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_ml${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/opencv_video${OPENCV_VER}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	IF (OPENCV_VIZ_FOUND)
		IF(NOT EXISTS "${OpenCV_BIN_DIR_OPT}/opencv_viz${OPENCV_VER}.dll")
			MESSAGE(FATAL_ERROR "OpenCV viz module was found, but the libraries don't appear to have been built correctly. Corrupt modules will adversely affect the buld process. Please either reconfigure OpenCV with the viz module deselected, or re-build the viz module. The most common reason for failure to build the opencv viz module is linking to the incorrect version of VTK (e.g. 64-bit instead of 32-bit).")
		ENDIF()
	ENDIF()
ELSE()
	MESSAGE(WARNING "<${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}.dll> unable to be located. Has OpenCV been built in Release mode?")
ENDIF()

IF(EXISTS "${OpenCV_BIN_DIR_DBG}/opencv_core${OPENCV_VER}d.dll")

	MESSAGE(STATUS "OpenCV_BIN_DIR_DBG = ${OpenCV_BIN_DIR_DBG}")

	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_core${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_highgui${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_calib3d${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_features2d${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_flann${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_imgproc${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_objdetect${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_ml${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/opencv_video${OPENCV_VER}d.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	IF (OPENCV_VIZ_FOUND)
		IF(NOT EXISTS "${OpenCV_BIN_DIR_DBG}/opencv_viz${OPENCV_VER}d.dll")
			MESSAGE(FATAL_ERROR "OpenCV viz module was found, but the libraries don't appear to have been built correctly. Corrupt modules will adversely affect the buld process. Please either reconfigure OpenCV with the viz module deselected, or re-build the viz module. The most common reason for failure to build the opencv viz module is linking to the incorrect version of VTK (e.g. 64-bit instead of 32-bit).")
		ENDIF()
	ENDIF()
	
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_core${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_highgui${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_calib3d${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_features2d${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_flann${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_imgproc${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_objdetect${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_ml${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/../staticlib/opencv_video${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	
ELSE()
	MESSAGE(WARNING "<${OpenCV_BIN_DIR_OPT}/opencv_core${OPENCV_VER}d.dll> unable to be located. Has OpenCV been built in Debug mode?")
ENDIF()

IF(BUILD_GUI)
	IF(EXISTS "${Qt5Widgets_DIR}/../../../bin/Qt5Widgetsd.dll")
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Widgetsd.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Widgets.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Cored.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Core.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Guid.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Gui.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/icuin52.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/icuuc52.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/icudt52.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ELSE()
		MESSAGE(WARNING "<${Qt5Widgets_DIR}/../../../bin/Qt5Widgetsd.dll> unable to be located. Has Qt5 been installed properly?")
	ENDIF()
ENDIF()

IF(USE_BOOST)
	IF(EXISTS "${Boost_LIBRARY_DIR}/boost_system${_boost_COMPILER}${_boost_MULTITHREADED}-${Boost_LIB_VERSION}.dll")
		add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Boost_LIBRARY_DIR}/boost_system${_boost_COMPILER}${_boost_MULTITHREADED}-${Boost_LIB_VERSION}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ELSE()
		MESSAGE(WARNING "Unable to find DLL at ${Boost_LIBRARY_DIR}/boost_system${_boost_COMPILER}${_boost_MULTITHREADED}-${Boost_LIB_VERSION}.dll - has Boost Release been built?")
	ENDIF()
	
	IF(EXISTS "${Boost_LIBRARY_DIR}/boost_system${_boost_COMPILER}${_boost_MULTITHREADED}-gd-${Boost_LIB_VERSION}.dll")
		add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Boost_LIBRARY_DIR}/boost_system${_boost_COMPILER}${_boost_MULTITHREADED}-gd-${Boost_LIB_VERSION}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ELSE()
		MESSAGE(WARNING "Unable to find DLL at ${Boost_LIBRARY_DIR}/boost_system${_boost_COMPILER}${_boost_MULTITHREADED}-gd-${Boost_LIB_VERSION}.dll - has Boost Debug been built?")
	ENDIF()	
ENDIF()