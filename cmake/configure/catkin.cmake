MESSAGE(STATUS "This project will be built for ROS. If you don't know what that is, you might want to consider deselecting the option <BUILD_FOR_ROS>.")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

add_message_files(FILES feature_tracks.msg pose_confidence.msg)

generate_messages(DEPENDENCIES ${MSG_DEP_SET})

generate_dynamic_reconfigure_options(
  cfg/calibrator.cfg
  cfg/flow.cfg
  cfg/slam.cfg
  cfg/streamer.cfg
)

catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES thermalvis
	CATKIN_DEPENDS message_runtime ${MSG_DEP_SET}
	#DEPENDS system_lib
)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(cfg/cpp/thermalvis)
include_directories(${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
