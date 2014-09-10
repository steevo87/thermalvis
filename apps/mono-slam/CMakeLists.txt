SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

IF(BUILD_GUI)
	# Find includes in corresponding build directories
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	# Instruct CMake to run moc automatically when needed.
	set(CMAKE_AUTOMOC ON)
ENDIF()

IF(IS_WINDOWS)
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
LIST(APPEND ADDITIONAL_LIBS thermalvis-streamer)
LIST(APPEND ADDITIONAL_LIBS thermalvis-flow)
LIST(APPEND ADDITIONAL_LIBS thermalvis-slam)

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
set_target_properties (${EXECUTABLE_NAME} PROPERTIES COMPILE_FLAGS ${THERMALVIS_COMPILE_FLAGS})

IF(EXISTS "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_core${OPENCV_VER}.${DLL_EXT}")
	
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_core${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_highgui${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_calib3d${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_features2d${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_flann${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_imgproc${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_objdetect${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_ml${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_video${OPENCV_VER}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_ffmpeg${OPENCV_VER}_64.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	IF (OPENCV_VIZ_FOUND)
		IF(NOT EXISTS "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_viz${OPENCV_VER}.${DLL_EXT}")
			MESSAGE(FATAL_ERROR "OpenCV viz module was found, but the libraries don't appear to have been built correctly. Corrupt modules will adversely affect the buld process. Please either reconfigure OpenCV with the viz module deselected, or re-build the viz module. The most common reason for failure to build the opencv viz module is linking to the incorrect version of VTK (e.g. 64-bit instead of 32-bit).")
		ENDIF()
	ENDIF()
ELSE()
	MESSAGE(WARNING "<${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_core${OPENCV_VER}.${DLL_EXT}> unable to be located. Has OpenCV been built in Release mode?")
ENDIF()

IF(EXISTS "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_core${OPENCV_VER}d.${DLL_EXT}")

	MESSAGE(STATUS "OpenCV_BIN_DIR_DBG = ${OpenCV_BIN_DIR_DBG}")

	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_core${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_highgui${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_calib3d${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_features2d${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_flann${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_imgproc${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_objdetect${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_ml${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_video${OPENCV_VER}d.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_ffmpeg${OPENCV_VER}_64.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	IF (OPENCV_VIZ_FOUND)
		IF(NOT EXISTS "${OpenCV_BIN_DIR_DBG}/${LIB_PREFIX}opencv_viz${OPENCV_VER}d.${DLL_EXT}")
			MESSAGE(FATAL_ERROR "OpenCV viz module was found, but the libraries don't appear to have been built correctly. Corrupt modules will adversely affect the buld process. Please either reconfigure OpenCV with the viz module deselected, or re-build the viz module. The most common reason for failure to build the opencv viz module is linking to the incorrect version of VTK (e.g. 64-bit instead of 32-bit).")
		ENDIF()
	ENDIF()
	
	IF(IS_WINDOWS)
	
		IF(EXISTS "${OpenCV_BIN_DIR_DBG}/../staticlib")
			SET(PDB_PREFIX "/../staticlib")
		ELSE()
			SET(PDB_PREFIX "")
		ENDIF()
		
		IF (EXISTS "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_core${OPENCV_VER}d.pdb")
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_core${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_highgui${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_calib3d${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_features2d${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_flann${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_imgproc${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_objdetect${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_ml${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
			ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}/opencv_video${OPENCV_VER}d.pdb" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ELSE()
			MESSAGE(WARNING "Unable to find PDB files for debugging in directory <${OpenCV_BIN_DIR_DBG}${PDB_PREFIX}>. You may not be able to debug OpenCV functions.")
		ENDIF()
	ENDIF()
ELSE()
	MESSAGE(WARNING "<${OpenCV_BIN_DIR_OPT}/${LIB_PREFIX}opencv_core${OPENCV_VER}d.${DLL_EXT}> unable to be located. Has OpenCV been built in Debug mode?")
ENDIF()

IF(BUILD_GUI)
	IF(EXISTS "${Qt5Widgets_DIR}/../../../bin/Qt5Widgetsd.${DLL_EXT}")
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Widgetsd.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Widgets.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Cored.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Core.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Guid.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/Qt5Gui.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/icuin52.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/icuuc52.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/icudt52.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ELSE()
		MESSAGE(WARNING "<${Qt5Widgets_DIR}/../../../bin/Qt5Widgetsd.${DLL_EXT}> unable to be located. Has Qt5 been installed properly?")
	ENDIF()
ENDIF()

IF(USE_BOOST)

	IF(IS_WINDOWS)
		SET(BOOST_SUFFIX_1 "${_boost_COMPILER}${_boost_MULTITHREADED}-")
		SET(BOOST_SUFFIX_2 "${Boost_LIB_VERSION}")
	ELSE()
		SET(BOOST_SUFFIX_1 "")
		SET(BOOST_SUFFIX_2 "")
	ENDIF()

	IF(EXISTS "${Boost_LIBRARY_DIR}/${LIB_PREFIX}boost_system${BOOST_SUFFIX_1}${BOOST_SUFFIX_2}.${DLL_EXT}")
		add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Boost_LIBRARY_DIR}/${LIB_PREFIX}boost_system${BOOST_SUFFIX_1}${BOOST_SUFFIX_2}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ELSE()
		MESSAGE(WARNING "Unable to find DLL at ${Boost_LIBRARY_DIR}/${LIB_PREFIX}boost_system${BOOST_SUFFIX_1}${BOOST_SUFFIX_2}.${DLL_EXT} - has Boost Release been built?")
	ENDIF()
	
	IF(EXISTS "${Boost_LIBRARY_DIR}/${LIB_PREFIX}boost_system${BOOST_SUFFIX_1}gd-${BOOST_SUFFIX_2}.${DLL_EXT}")
		add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Boost_LIBRARY_DIR}/${LIB_PREFIX}boost_system${BOOST_SUFFIX_1}gd-${BOOST_SUFFIX_2}.${DLL_EXT}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ELSE()
		MESSAGE(WARNING "Unable to find DLL at ${Boost_LIBRARY_DIR}/${LIB_PREFIX}boost_system${BOOST_SUFFIX_1}gd-${BOOST_SUFFIX_2}.${DLL_EXT} - has Boost Debug been built?")
	ENDIF()	
ENDIF()
