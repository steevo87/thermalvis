find_package(Doxygen QUIET)

IF(DOXYGEN_FOUND)
        SET(doxygen_FOUND TRUE)
        SET(USE_DOXYGEN TRUE CACHE BOOL "Build documentation using Doxygen.")
        MESSAGE(STATUS "Doxygen was found!")
ELSE()
        SET(doxygen_FOUND FALSE)
        SET(USE_DOXYGEN FALSE CACHE BOOL "Build documentation using Doxygen.")
ENDIF()

IF(USE_DOXYGEN)
        MESSAGE(STATUS "Doxygen will be used to build documentation.")
ENDIF()
