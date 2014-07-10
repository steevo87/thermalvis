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

set (ADDITIONAL_LIBS "")

FILE(GLOB LOCAL_HEADERS include/*.h*)
FILE(GLOB LOCAL_SOURCES src/*.cpp)

INCLUDE_DIRECTORIES("../../include")

IF(BUILD_GUI)
	INCLUDE_DIRECTORIES("../../qt/${Qt_STREAMER_BUILD_DIR}")
	INCLUDE_DIRECTORIES("../../qt/streamer")
	LIST(APPEND ADDITIONAL_LIBS Qt5::Widgets)
ENDIF()

LIST(APPEND ADDITIONAL_LIBS ${OpenCV_LIBRARIES})
LIST(APPEND ADDITIONAL_LIBS ${Boost_LIBRARIES})

LIST(APPEND ADDITIONAL_LIBS thermalvis)

SET(EXECUTABLE_NAME "MonocularSLAM")
IF(BUILD_GUI)
	add_executable(${EXECUTABLE_NAME} ${LOCAL_SOURCES} ${LOCAL_HEADERS} 
	../../qt/streamer/mainwindow_streamer.cpp 
	../../qt/streamer/mainwindow_streamer.h 
	../../qt/${Qt_STREAMER_BUILD_DIR}/ui_mainwindow_streamer.h)
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