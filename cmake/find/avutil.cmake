IF(IS_WINDOWS)
	SET(HAS_AVLIBS_AVAILABLE FALSE)
	SET(avutil_FOUND FALSE)
ELSE()
	SET(HAS_AVLIBS_AVAILABLE TRUE)

        LIST(APPEND avlibs_LIST avutil avformat avcodec swscale) # va avdevice avconv
        foreach(AVLIBS_SUB_LIB ${avlibs_LIST})
                string(TOUPPER "${AVLIBS_SUB_LIB}" UPPERCASE_NAME)
                FIND_LIBRARY("${UPPERCASE_NAME}_LIBRARY" NAMES ${AVLIBS_SUB_LIB} PATHS ${SEARCH_LIBRARY_PATHS} QUIET)
                if(HAS_AVLIBS_AVAILABLE AND (NOT "${UPPERCASE_NAME}_LIBRARY"))
                    IF(HAS_AVLIBS_AVAILABLE)
                        MESSAGE(STATUS "Unable to find library ${AVLIBS_SUB_LIB} so terminating search for further avutil-related libraries.")
                    ENDIF()
                        SET(HAS_AVLIBS_AVAILABLE FALSE)
                endif()
        endforeach(AVLIBS_SUB_LIB)

        IF(HAS_AVLIBS_AVAILABLE)
            SET(avutil_FOUND TRUE)
        ELSE()
			SET(avutil_FOUND FALSE)
		ENDIF()
ENDIF()

IF(avutil_FOUND)
    OPTION(USE_AVUTIL "Use the avutil and related libraries." TRUE)
ELSE()
    OPTION(USE_AVUTIL "Use the avutil and related libraries." FALSE)
ENDIF()

IF(USE_AVUTIL)
    IF(NOT avutil_FOUND)
        MESSAGE(FATAL_ERROR "Cannot find avutil and related libraries. Please deselect <USE_AVUTIL>.")
    ENDIF()

    foreach(AVLIBS_SUB_LIB ${avlibs_LIST})
            string(TOUPPER "${AVLIBS_SUB_LIB}" UPPERCASE_NAME)
            LIST(APPEND ADDITIONAL_LIBRARIES ${${UPPERCASE_NAME}_LIBRARY})
    endforeach(AVLIBS_SUB_LIB)

    add_definitions( -D_AVLIBS_AVAILABLE_ )
ENDIF()


