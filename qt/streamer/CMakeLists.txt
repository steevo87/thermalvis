cmake_minimum_required(VERSION 2.8.11)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
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
	../../src/${NODE_NAME}/input_stream_config.cpp
	mainwindow_${NODE_NAME}.h 
	${QT${NODE_NAME_UPPER}_BUILD_PATH}/ui_mainwindow_${NODE_NAME}.h
	../../include/${NODE_NAME}/input_stream_config.hpp
)

# Use the Widgets module from Qt 5.
target_link_libraries(${EXECUTABLE_NAME} Qt5::Widgets)

SET(ALL_QTDLL_FOUND TRUE)
LIST(APPEND QTDLL_NAMES "Qt5Widgetsd" "Qt5Widgets" "Qt5Cored" "Qt5Core" "Qt5Guid" "Qt5Gui" "icuin52" "icuuc52" "icudt52")
foreach(QTDLL_NAME ${QTDLL_NAMES})
	IF(EXISTS "${Qt5Widgets_DIR}/../../../bin/${QTDLL_NAME}.dll")
		ADD_CUSTOM_COMMAND(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${Qt5Widgets_DIR}/../../../bin/${QTDLL_NAME}.dll" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
	ELSEIF(ALL_QTDLL_FOUND)
		MESSAGE(WARNING "<${Qt5Widgets_DIR}/../../../bin/${QTDLL_NAME}.dll> unable to be located. Has Qt5 been installed properly?")
		SET(ALL_QTDLL_FOUND FALSE)
	ENDIF()
endforeach(QTDLL_NAME) 
