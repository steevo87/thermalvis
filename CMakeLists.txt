cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

option(BUILD_APPS "Build demo apps." TRUE)
option(BUILD_FOR_ROS "Build ROS interfaces and functions." FALSE)
option(USE_PCL "Build PCL-dependent functions." FALSE)
option(USE_EIGEN "Build Eigen-dependent functions." FALSE)

SET(ADDITIONAL_INCLUDES "")
SET(ADDITIONAL_LIBRARIES "")
SET(THERMALVIS_COMPILE_FLAGS "-D_THERMALVIS_")

IF(USE_EIGEN)
	SET(Eigen_DIR "C:/eigen" CACHE STRING "..." FORCE)

	IF(NOT EXISTS "$Eigen_DIR")
		MESSAGE(FATAL_ERROR "Provided path for Eigen does not appear to contain valid debug OR release libraries!")
	ENDIF()

	find_package(Eigen HINTS "C:/eigen/cmake")

	IF(Eigen_FOUND)
		add_definitions( -D_USE_EIGEN_ )
		set (ADDITIONAL_INCLUDES ${ADDITIONAL_INCLUDES} ${EIGEN3_INCLUDE_DIR})
	ENDIF()
ENDIF()

add_definitions( -D_CRT_SECURE_NO_WARNINGS )

IF(USE_PCL)

	set(Boost_USE_STATIC_LIBS OFF) 
	set(Boost_USE_MULTITHREADED ON)  
	set(Boost_USE_STATIC_RUNTIME OFF) 
	
	find_package(Boost 1.50.0)
	
	if(Boost_FOUND)
		find_package( PCL COMPONENTS common )
		message(STATUS "PCL Found: " ${PCL_FOUND})
		
		IF(PCL_FOUND)
			add_definitions( -D_USE_PCL_ )
			set (ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${PCL_LIBRARIES})
		ENDIF()
	endif()

	
ENDIF()

IF(BUILD_FOR_ROS)
	add_definitions( -D_BUILD_FOR_ROS_ )
ENDIF()
	
find_package( OpenCV 2.4.8 HINTS "$ENV{USERPROFILE}/Documents/GitHub/BUILDS/OpenCV" "C:/Users/Public/Documents/opencv/build")

string(REGEX REPLACE "/Release/Release" "/Release" OpenCV_LIB_DIR_OPT "${OpenCV_LIB_DIR_OPT}")
string(REGEX REPLACE "/Debug/Debug" "/Debug" OpenCV_LIB_DIR_DBG "${OpenCV_LIB_DIR_DBG}")

IF(OpenCV_FOUND AND ("${OpenCV_DIR}" STREQUAL "${OpenCV_CONFIG_PATH}") AND (NOT("${OpenCV_LIB_DIR_DBG}" STREQUAL "/Debug")) AND (NOT("${OpenCV_LIB_DIR_OPT}" STREQUAL "/Release")) )
	message(STATUS "OpenCV Found!")
	string(REGEX REPLACE "/lib" "/bin" OpenCV_BIN_DIR_OPT "${OpenCV_LIB_DIR_OPT}")
	string(REGEX REPLACE "/lib" "/bin" OpenCV_BIN_DIR_DBG "${OpenCV_LIB_DIR_DBG}")
ELSEIF(EXISTS "${OpenCV_DIR}/")
	message(WARNING "OpenCV not entirely found, but directory address <OpenCV_DIR> located and so assuming local build at ${OpenCV_DIR}")
	
	IF( (NOT EXISTS "${OpenCV_DIR}/lib/Debug/") AND (NOT EXISTS "${OpenCV_DIR}/lib/Release/") )
		MESSAGE(FATAL_ERROR "Provided path for OpenCV does not appear to contain valid debug OR release libraries!")
	ENDIF()
	
	SET(OpenCV_LIB_DIR_OPT "${OpenCV_DIR}/lib/Release" CACHE STRING "..." FORCE)
	SET(OpenCV_LIB_DIR_DBG "${OpenCV_DIR}/lib/Debug" CACHE STRING "..." FORCE)
	SET(OpenCV_3RDPARTY_LIB_DIR_OPT "${OpenCV_DIR}/lib/Release" CACHE STRING "..." FORCE)
	SET(OpenCV_3RDPARTY_LIB_DIR_DBG "${OpenCV_DIR}/lib/Debug" CACHE STRING "..." FORCE)
	
	LIST(APPEND _OpenCV_LIB_PATH "${OpenCV_DIR}/bin/Debug")
	LIST(APPEND _OpenCV_LIB_PATH "${OpenCV_DIR}/bin/Release")
	
	SET(OPENCV_FOUND TRUE CACHE BOOL "..." FORCE)
	SET(OpenCV_FOUND TRUE CACHE BOOL "..." FORCE)
	
	SET(OpenCV_BIN_DIR_OPT "${OpenCV_DIR}/bin/Release")
	SET(OpenCV_BIN_DIR_DBG "${OpenCV_DIR}/bin/Debug")
	SET(OpenCV_CONFIG_PATH "${OpenCV_DIR}" CACHE STRING "..." FORCE)
ELSE()
	MESSAGE(FATAL_ERROR "OpenCV not found! Please set the <OpenCV_DIR> variable to the OpenCV build directory.")
ENDIF()

IF(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	
	IF (NOT ("${_OpenCV_LIB_PATH}" STREQUAL ""))
		link_directories(${_OpenCV_LIB_PATH})
	ENDIF()
	
	LIST(APPEND ADDITIONAL_LIBRARIES ${OpenCV_LIBS})
	STRING(REGEX REPLACE "\\." "" OPENCV_VER "${OpenCV_VERSION}")
	IF (OpenCV_VERSION_MAJOR GREATER 2)
		MESSAGE(STATUS "OpenCV Version is 3+, so some functions differ. This should be handled by compiler directives.")
		SET(THERMALVIS_COMPILE_FLAGS "${THERMALVIS_COMPILE_FLAGS} -D_OPENCV_VERSION_3_PLUS_")
	ENDIF()
	
	IF (OPENCV_GPU_FOUND)
		MESSAGE(STATUS "OpenCV GPU module found, so including it.")
		SET(THERMALVIS_COMPILE_FLAGS "${THERMALVIS_COMPILE_FLAGS} -D_USE_OPENCV_GPU_")
	ENDIF()
ENDIF()

# To add all necessary source and header files (courtesy of https://github.com/StevenHickson)
file(GLOB_RECURSE THERMALVIS_HEADERS include/*.h*)
file(GLOB_RECURSE THERMALVIS_SOURCES src/*.cpp)
set (THERMALVIS_INCLUDE_DIRS "")
foreach (_headerFile ${THERMALVIS_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND THERMALVIS_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES THERMALVIS_INCLUDE_DIRS)

message(STATUS "headers: " ${THERMALVIS_HEADERS})

#list(REMOVE_ITEM THERMALVIS_HEADERS "IlmImf/b44ExpLogTable.cpp")
#list(REMOVE_ITEM THERMALVIS_SOURCES "IlmImf/b44ExpLogTable.cpp")

include_directories(${THERMALVIS_INCLUDE_DIRS})
include_directories(${ADDITIONAL_INCLUDES})

IF(USE_PCL)
	message(STATUS "PCL_INCLUDE_DIRS: " ${PCL_LIBRARIES})
	include_directories(${PCL_INCLUDE_DIRS})
ENDIF()

add_library (thermalvis ${THERMALVIS_SOURCES} ${THERMALVIS_HEADERS} )
target_link_libraries (thermalvis ${OpenCV_LIBS} ${ADDITIONAL_LIBRARIES})
set_target_properties (thermalvis PROPERTIES COMPILE_FLAGS ${THERMALVIS_COMPILE_FLAGS})

IF(WIN32 OR win64)
   add_subdirectory(apps)
ENDIF()