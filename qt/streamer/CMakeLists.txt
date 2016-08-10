cmake_minimum_required(VERSION 2.8.11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

SET(NODE_NAME "streamer")

string(TOUPPER ${NODE_NAME} NODE_NAME_UPPER)
SET(EXECUTABLE_NAME "${NODE_NAME}_qt_test")

MESSAGE(STATUS "Including DIR: ${Qt_${NODE_NAME_UPPER}_BUILD_DIR}")

INCLUDE_DIRECTORIES(../${Qt_${NODE_NAME_UPPER}_BUILD_DIR})
INCLUDE_DIRECTORIES(../../include/${NODE_NAME})
		
IF(NOT EXISTS "${QT${NODE_NAME_UPPER}_BUILD_PATH}/ui_mainwindow_${NODE_NAME}.h")
	MESSAGE(FATAL_ERROR "<ui_mainwindow_${NODE_NAME}.h> could not be found in <${QT${NODE_NAME_UPPER}_BUILD_PATH}>. Has Qt Creator been used to generate this file? If not, open <thermalvis/qt/${NODE_NAME}/${NODE_NAME}.pro> in qt Creator and build it. Alternatively, check if the CMake variable incorrect.")
ENDIF()

# Tell CMake to create the executable
add_executable(${EXECUTABLE_NAME} WIN32 
	main.cpp 
	mainwindow_${NODE_NAME}.cpp 
	../../src/${NODE_NAME}/${NODE_NAME}_config.cpp
	mainwindow_${NODE_NAME}.h 
	${QT${NODE_NAME_UPPER}_BUILD_PATH}/ui_mainwindow_${NODE_NAME}.h
	../../include/${NODE_NAME}/${NODE_NAME}_config.hpp
)

# Use the Widgets module from Qt 5.
target_link_libraries(${EXECUTABLE_NAME} Qt5::Widgets)

include(../../cmake/link/qt.cmake)
