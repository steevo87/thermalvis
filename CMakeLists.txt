cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

option(BUILD_APPS "Build demo apps." TRUE)
option(BUILD_FOR_ROS "Build ROS interfaces and functions." FALSE)
option(USE_PCL "Build PCL-related functions." FALSE)

set (ADDITIONAL_LIBS "")
set (ADDITIONAL_INCLUDES "")

find_package(Eigen)

IF(Eigen_FOUND)
	add_definitions( -D_USE_EIGEN_ )
	set (ADDITIONAL_INCLUDES ${ADDITIONAL_INCLUDES} ${EIGEN3_INCLUDE_DIR})
endif()

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
			set (ADDITIONAL_LIBS ${ADDITIONAL_LIBS} ${PCL_LIBRARIES})
		ENDIF()
	endif()

	
ENDIF()

IF(BUILD_FOR_ROS)
	add_definitions( -D_BUILD_FOR_ROS_ )
ENDIF()
	
find_package( OpenCV REQUIRED )

IF(WIN32 OR win64)
   add_subdirectory(apps)
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
target_link_libraries (thermalvis ${OpenCV_LIBS} ${ADDITIONAL_LIBS})