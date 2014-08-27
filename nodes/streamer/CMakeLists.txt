cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(NODE_NAME "streamer")
project(${NODE_NAME})

FILE(GLOB "${NODE_NAME}_HEADERS" *.h*)
FILE(GLOB "${NODE_NAME}_SOURCES" *.cpp)

INCLUDE_DIRECTORIES("../../include")
include_directories(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("../../cfg/cpp/thermalvis/")

IF (NOT ("${_OpenCV_LIB_PATH}" STREQUAL ""))
	LINK_DIRECTORIES(${_OpenCV_LIB_PATH})
ENDIF()

add_executable(${NODE_NAME} ${${NODE_NAME}_SOURCES} ${${NODE_NAME}_HEADERS})
target_link_libraries(${NODE_NAME} thermalvis thermalvis-${NODE_NAME} ${catkin_LIBRARIES} ${ADDITIONAL_LIBRARIES}) # very sensitive to order!
add_dependencies(${NODE_NAME} ${catkin_EXPORTED_TARGETS})
