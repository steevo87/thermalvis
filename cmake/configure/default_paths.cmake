SET(THERMALVIS_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}" 
  CACHE STRING "Directory containing thermalvis source.")
ADD_DEFINITIONS(-D_THERMALVIS_SOURCE_="${THERMALVIS_SOURCE}")

SET(DEFAULT_SAMPLE_DATA
  "${CMAKE_CURRENT_SOURCE_DIR}/media/sample_video/optris" 
  CACHE STRING "Directory containing raw thermal-infrared images that can be played by default.")
ADD_DEFINITIONS(-D_DEFAULT_SAMPLE_DATA_="${DEFAULT_SAMPLE_DATA}")

SET(DEFAULT_LAUNCH_DIR 
  "${CMAKE_CURRENT_SOURCE_DIR}/launch" 
  CACHE STRING "Directory containing launch files.")
ADD_DEFINITIONS(-D_DEFAULT_LAUNCH_DIR_="${DEFAULT_LAUNCH_DIR}")
