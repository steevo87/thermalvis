cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

# Flags necessary to get Managed Code functioning for Windows-Optris demo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /clr")
STRING (REGEX REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
STRING (REGEX REPLACE "/EHsc" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

IF(WIN32 OR win64)
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)
ELSE()
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
ENDIF()

find_package( OpenCV REQUIRED )

project(win-optris)

option(USE_OPTRIS_SDK "Build with Optris compatability" TRUE)

set (ADDITIONAL_LIBS "")

IF(WIN32 OR win64)
	IF(USE_OPTRIS_SDK)
		set(OptrisSDK "$ENV{USERPROFILE}/Documents/Optris/SDK" CACHE FILEPATH "Include directory for Optris SDK")
		include_directories(${OptrisSDK})
		link_directories(${OptrisSDK})
		set (ADDITIONAL_LIBS ${ADDITIONAL_LIBS} "ImagerIPC2.dll")
	ENDIF()
ENDIF()

# To add all necessary source and header files (courtesy of https://github.com/StevenHickson)
file(GLOB_RECURSE WINOPTRIS_HEADERS include/*.h)
file(GLOB_RECURSE WINOPTRIS_SOURCES src/*.cpp)
set (WINOPTRIS_INCLUDE_DIRS "")
foreach (_headerFile ${WINOPTRIS_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND WINOPTRIS_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES WINOPTRIS_INCLUDE_DIRS)

include_directories("../../include")
include_directories(${WINOPTRIS_INCLUDE_DIRS})
include_directories("${OptrisSDK}/Samples VC++ .net/Shared Files/")

link_directories(${_OpenCV_LIB_PATH})

IF(WIN32 OR win64)

	IF(USE_OPTRIS_SDK)
		add_executable (OptrisWinTest ${WINOPTRIS_SOURCES} ${WINOPTRIS_HEADERS} )
		target_link_libraries (OptrisWinTest ${OpenCV_LIBS} ${ADDITIONAL_LIBS} thermalvis)
		
		# Custom build steps to move DLL into execution directory
		# Based on suggestion from http://stackoverflow.com/questions/10671916/how-to-copy-dll-files-into-the-same-folder-as-the-executable-using-cmake

		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_core248.dll"      		# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		add_custom_command(TARGET OptrisWinTest POST_BUILD  # Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_core248d.dll"       	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
				
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_highgui248.dll"      	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_highgui248d.dll"    	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
				
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_calib3d248.dll"      		# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_calib3d248d.dll"    	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
				
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_features2d248.dll"      # <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_features2d248d.dll"    	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
				
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_flann248.dll"      		# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_flann248d.dll"    		# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
				
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_imgproc248.dll"      	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_imgproc248d.dll"    	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
				
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_objdetect248.dll"      	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		add_custom_command(TARGET OptrisWinTest POST_BUILD        	# Adds a post-build event to MyTest
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  			# which executes "cmake - E copy_if_different..."
				"${_OpenCV_LIB_PATH}/opencv_objdetect248d.dll"    	# <--this is in-file
				$<TARGET_FILE_DIR:OptrisWinTest>)                 	# <--this is out-file path
		
	ENDIF()
	
ENDIF()

