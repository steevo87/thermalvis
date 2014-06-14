thermalvis
==========

Cross-platform, OpenCV-based functionality for image processing and computer vision in thermal-infrared.
For Windows, the recommended build environment is Visual Studio 2012, though it should work on others.

required dependencies:
----------------------

OPENCV
	recommended to download a pre-compiled binary installer e.g. from here: http://sourceforge.net/projects/opencvlibrary/files/opencv-win/

optional dependencies:
----------------------

BOOST
	recommended version is boost_1_55_0-msvc-11.0-64.exe (http://sourceforge.net/projects/boost/files/boost-binaries/1.55.0/)
	may need to add the following to the System Path: 
		BOOST_LIBRARYDIR = C:\local\boost_1_55_0\lib64-msvc-11.0
		BOOST_ROOT = C:\local\boost_1_55_0
	beware of the directions of the slashes in the directory paths!

PCL
	There is no all-in-one installer for VS2012, or for the latest version (1.7.1)
	May want to compile from source:
		it's a GitHub project, at: https://github.com/PointCloudLibrary/pcl
	Try compiling from source and then let me know if you have any problems!

EIGEN
	can download from here: http://eigen.tuxfamily.org/index.php?title=Main_Page
	preferable to unzip into C:/eigen (i.e. change the name to remove the suffix code)
		when unzipping, make sure you don't end up with a folder within a folder, such as: C:/eigen/eigen-eigen-6b38706d90a9
	if you get the following error when building:
		Could NOT find Eigen3 (missing:  EIGEN3_INCLUDE_DIR EIGEN3_VERSION_OK) (Required is at least version "2.91.0")
	then you may need to add the directory (e.g. C:\eigen) to the System PATH variable
	to rebuild, deleting the cache and reconfiguring is not enough - CMake must be reloaded for this change to take effect
	
troubleshooting:
----------------

DODGY IMAGE FROM CAMERA
	try refocussing by manually rotating the physical lens
	try shutting down the Optris PI Connect software, unplugging the camera, and then testing again
