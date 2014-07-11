thermalvis
==========

Cross-platform, OpenCV-based functionality for image processing and computer vision in thermal-infrared.
For Windows, the recommended build environment is Visual Studio 2012, though it should work on others.

required dependencies:
----------------------

* OPENCV
 * recommended to download a pre-compiled binary installer e.g. from here: http://sourceforge.net/projects/opencvlibrary/files/opencv-win/
 * preferable to unpack the library at one of the following locations:
  * "C:/Users/_USERNAME_/Documents/opencv/build" OR "C:/Users/Public/Documents/opencv/build"

optional dependencies:
----------------------

###### BOOST
 * recommended version is boost_1_55_0-msvc-11.0-64.exe (http://sourceforge.net/projects/boost/files/boost-binaries/1.55.0/)
 * may need to add the following to the System Path: 
  * BOOST_LIBRARYDIR = C:\local\boost_1_55_0\lib64-msvc-11.0
  * BOOST_ROOT = C:\local\boost_1_55_0
 * beware of the directions of the slashes in the directory paths!

##### PCL
* There is no all-in-one installer for VS2012, or for the latest version (1.7.1)
* May want to compile from source:
 * it's a GitHub project, at: https://github.com/PointCloudLibrary/pcl
* Try compiling from source and then let me know if you have any problems!

###### EIGEN
* can download from here: http://eigen.tuxfamily.org/index.php?title=Main_Page
* preferable to unzip into C:/eigen (i.e. change the name to remove the suffix code)
 * when unzipping, make sure you don't end up with a folder within a folder, such as: C:/eigen/eigen-eigen-6b38706d90a9
* if you get the following error when building:
 * Could NOT find Eigen3 (missing:  EIGEN3_INCLUDE_DIR EIGEN3_VERSION_OK) (Required is at least version "2.91.0")
* then you may need to add the directory (e.g. C:\eigen) to the System PATH variable
* to rebuild, deleting the cache and reconfiguring is not enough - CMake must be reloaded for this change to take effect
	
###### Qt
* minimum version Qt5
* try to download using the online installer: http://qt-project.org/downloads
 * however, there's a good chance this won't work
* can download a full installer from the website, e.g.
 * qt-opensource-windows-x86-msvc2012_opengl-5.3.1.exe
 * if the EXE doesn't launch properly, it's possible not all of the data was downloaded before the download manager or browser was closed, so it is corrupted
* in CMake, you will need to make sure that the <Qt5Widgets_DIR> variable points to the directory containing the Qt5WidgetsConfig.cmake file:
 * usually something like: C:\Qt\Qt5.3.1\5.3\msvc2012_opengl\lib\cmake\Qt5Widgets
* if CMake cannot find the relevent UI files (e.g. "ui_mainwindow_streamer.h"), you will need to use Qt Creator (installed automatically with above steps) to compile each of the Qt GUIs:
 * Open Qt Creator, and open a project (e.g. thermalvis\qt\streamer\streamer.pro)
 * Press < CTRL + SHIFT + B > to build
 * This will create an adjacent "build-..." directory, which will importantly contain a file such as <ui_mainwindow_streamer.h>
 * Repeat this procedure for each non-"build-..." directory in the "qt" folder
	
troubleshooting:
----------------

###### NO DATA FROM OPTRIS PI450 IN WINDOWS
* launch the Optris PI Connect software, and verify that the camera is streaming correctly
 * go to [ Tools -> Configuration -> External Communication ] select "IPC" and click "Apply"
* rebooting the Optris PI Connect software sometimes helps
* NOTE: often this problem is caused by terminating the program using the "STOP button" in Visual Studio

###### CORRUPT IMAGE FROM PI450 IN WINDOWS
* try shutting down the Optris PI Connect software, unplugging the camera, and then testing again
	
###### POOR QUALITY IMAGE FROM PI450 (ANY OS)
* try refocussing by manually rotating the physical lens
