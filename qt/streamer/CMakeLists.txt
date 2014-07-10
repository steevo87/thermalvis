cmake_minimum_required(VERSION 2.8.11)

project(testproject)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

SET(EXECUTABLE_NAME "streamer_qt_test")

SET(Qt_STREAMER_BUILD_DIR "../build-streamer-Desktop_Qt_${QT_VERSION_MAJOR}_${QT_VERSION_MINOR}_${MSVC_ABBREVIATION_CAPS}_openGL_${ARCH_BITS}bit-Debug")
INCLUDE_DIRECTORIES(${Qt_STREAMER_BUILD_DIR})

# Tell CMake to create the helloworld executable
add_executable(${EXECUTABLE_NAME} WIN32 main.cpp mainwindow_streamer.cpp mainwindow_streamer.h ${Qt_STREAMER_BUILD_DIR}/ui_mainwindow_streamer.h)

# Use the Widgets module from Qt 5.
target_link_libraries(${EXECUTABLE_NAME} Qt5::Widgets)

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