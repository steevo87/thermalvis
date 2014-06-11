thermalvis
==========

Cross-platform, OpenCV-based functionality for image processing and computer vision in thermal-infrared

optional dependencies:
----------------------

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

DODGY IMAGE
	try refocussing by manually rotating the physical lens
	try shutting down the Optris PI Connect software, unplugging the camera, and then testing again
