SET(USE_OPENCV TRUE)

If(BUILD_FOR_ROS)
        find_package(OpenCV 2.4.8 REQUIRED)
        SET(opencv_FOUND TRUE)
        SET(opencv_LIBRARIES ${OpenCV_LIBS})
        include_directories(${OpenCV_INCLUDE_DIRS})
        LIST(APPEND ADDITIONAL_LIBRARIES ${OpenCV_LIBS})
        link_directories(${_OpenCV_LIB_PATH})
        return()
ENDIF()

LIST(APPEND OpenCV_Basic_Components_List "opencv_core")
LIST(APPEND OpenCV_Basic_Components_List "opencv_highgui")
LIST(APPEND OpenCV_Basic_Components_List "opencv_calib3d")
LIST(APPEND OpenCV_Basic_Components_List "opencv_objdetect")
LIST(APPEND OpenCV_Basic_Components_List "opencv_video")
LIST(APPEND OpenCV_Basic_Components_List "opencv_videoio")
LIST(APPEND OpenCV_Basic_Components_List "opencv_imgcodecs")

LIST(APPEND OpenCV_Prior_Components_List "opencv_contrib")
LIST(APPEND OpenCV_Prior_Components_List "opencv_legacy")

LIST(APPEND OpenCV_Advanced_Components_List "opencv_viz") # "opencv_gpu"

LIST(APPEND OpenCV_Components_List ${OpenCV_Basic_Components_List})
LIST(APPEND OpenCV_Components_List ${OpenCV_Prior_Components_List})

OPTION(OpenCV_USE_ADVANCED_COMPONENTS "E.g. opencv_viz." TRUE)
IF(OpenCV_USE_ADVANCED_COMPONENTS)
  LIST(APPEND OpenCV_Components_List ${OpenCV_Advanced_Components_List})
ENDIF()

SET(OPENCV_HINTS "/usr/local/share/OpenCV" "${USERPROFILE}/Documents/opencv/build" "C:/Users/Public/Documents/opencv/build" "${USERPROFILE}/Documents/Code/BUILDS/opencv/build" "${USERPROFILE}/Documents/GitHub/BUILDS/OpenCV")
find_package(OpenCV 2.4.8 COMPONENTS ${OpenCV_Components_List} QUIET)

IF(OpenCV_USE_ADVANCED_COMPONENTS AND NOT OPENCV_VIZ_FOUND) # If you can't find OpenCV with viz, exclude it and try again
	SET(OpenCV_Components_List "")
	find_package(OpenCV 2.4.8 COMPONENTS ${OpenCV_Components_List} QUIET)
	IF(NOT OpenCV_FOUND) # If you still can't find it, add viz back in as a requirement, and try your alternative "manual" approach further down the file
		LIST(APPEND OpenCV_Components_List ${OpenCV_Advanced_Components_List})
	ENDIF()
ENDIF()

SET(APOSITION "0")
IF (NOT "${OpenCV_LIB_DIR_OPT}" STREQUAL "")
	while (NOT "${APOSITION}" STREQUAL "-1")
		STRING( FIND ${OpenCV_LIB_DIR_OPT} "Release/Release" APOSITION )
		string(REGEX REPLACE "/Release/Release" "/Release" OpenCV_LIB_DIR_OPT "${OpenCV_LIB_DIR_OPT}")
		string(REGEX REPLACE "/Debug/Debug" "/Debug" OpenCV_LIB_DIR_DBG "${OpenCV_LIB_DIR_DBG}")
	endwhile()
ENDIF()

IF(OpenCV_FOUND AND ("${OpenCV_DIR}" STREQUAL "${OpenCV_CONFIG_PATH}") AND (NOT("${OpenCV_LIB_DIR_DBG}" STREQUAL "/Debug")) AND (NOT("${OpenCV_LIB_DIR_OPT}" STREQUAL "/Release")) )
	message(STATUS "OpenCV Found!")
	
	if ("${OpenCV_LIB_DIR_OPT}" STREQUAL "")
		IF (NOT IS_WINDOWS)
			SET(OpenCV_LIB_DIR_OPT "/usr/local/lib")
			SET(OpenCV_LIB_DIR_DBG "/usr/local/lib")
			SET(OpenCV_BIN_DIR_OPT "/usr/local/lib")
			SET(OpenCV_BIN_DIR_DBG "/usr/local/lib")
		ELSE()
			MESSAGE(FATAL_ERROR "OpenCV_LIB_DIR_OPT is empty!")
		ENDIF()
	else()
		STRING(REGEX REPLACE "/lib" "/bin" OpenCV_BIN_DIR_OPT "${OpenCV_LIB_DIR_OPT}")
		STRING(REGEX REPLACE "/lib" "/bin" OpenCV_BIN_DIR_DBG "${OpenCV_LIB_DIR_DBG}")
	endif()
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
	
	SET(OpenCV_BIN_DIR_OPT "${OpenCV_DIR}/bin/Release")
	SET(OpenCV_BIN_DIR_DBG "${OpenCV_DIR}/bin/Debug")
	SET(OpenCV_CONFIG_PATH "${OpenCV_DIR}" CACHE STRING "..." FORCE)
ELSE()
	MESSAGE(FATAL_ERROR "OpenCV not found! Please set the <OpenCV_DIR> variable to the OpenCV build directory.")
ENDIF()

SET(opencv_FOUND TRUE)

SET(opencv_LIBRARIES ${OpenCV_LIBS})

include_directories(${OpenCV_INCLUDE_DIRS})
LIST(APPEND ADDITIONAL_LIBRARIES ${OpenCV_LIBS})
link_directories(${_OpenCV_LIB_PATH})

IF(IS_WINDOWS)
        STRING(REGEX REPLACE "\\." "" OPENCV_VER "${OpenCV_VERSION}")
ELSE()
        SET(OPENCV_VER "")
ENDIF()

LIST(APPEND OpenCV_DLLs_List ${OpenCV_Basic_Components_List})
LIST(APPEND OpenCV_DLLs_List ${OpenCV_Advanced_Components_List})

LIST(APPEND OpenCV_DLLs_List "opencv_imgproc" "opencv_flann" "opencv_features2d")

IF(OpenCV_VERSION_MAJOR GREATER 2)
	add_definitions(-D_OPENCV_VERSION_3_PLUS_)
	# LIST(APPEND OpenCV_DLLs_List "opencv_imgcodecs" "opencv_videoio")
ELSE()
	LIST(APPEND OpenCV_DLLs_List ${OpenCV_Prior_Components_List})
ENDIF()

IF(OPENCV_GPU_FOUND)
	ADD_DEFINITIONS( -D_USE_OPENCV_GPU_ )
ENDIF()

IF(OPENCV_VIZ_FOUND)
	add_definitions(-D_USE_OPENCV_VIZ_)
ENDIF()
