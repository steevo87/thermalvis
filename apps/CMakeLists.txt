cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

IF(WIN32 OR win64)
   add_subdirectory(win-optris)
ENDIF()

