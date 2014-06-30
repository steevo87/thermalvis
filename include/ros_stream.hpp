/*! \file	ros_stream.hpp
 *  \brief	Declarations for ROS video input.
*/

#ifndef _THERMALVIS_ROS_STREAM_H_
#define _THERMALVIS_ROS_STREAM_H_

#ifdef _BUILD_FOR_ROS_
	
#include "streamer.hpp"
	
#include "time.h"

#include "cxcore.h"
#include "highgui.h"

#include <signal.h>

#include "ros_resources.hpp"
#include "tools.hpp"
#include "improc.hpp"
#include "radiometric.hpp"
#include "video.hpp"

#ifdef _BUILD_FOR_ROS_
#include <dynamic_reconfigure/server.h>
#include "streamerConfig.h"
#endif

// SERIAL COMMS STUFF
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>
#include <termio.h>
#endif

#include <stdio.h>

#include <fcntl.h>

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define BAUDRATE 								B38400
#define MODEMDEVICE 							"/dev/ttyS0"
#define _POSIX_SOURCE 							1         // POSIX compliant source
#define FALSE 									0
#define TRUE 									1
#define SERIAL_BUFF_SIZE 						1024

#define MAX_VALID_THERMISTOR 					80.0
#define MIN_VALID_THERMISTOR 				   -20.0
#define MAX_TIME_WITHOUT_FLAGS 					45.0
#define MAX_TIME_WITHOUT_NUC_conservative		45.0

volatile int STOP=FALSE;

void signal_handler_IO (int status);    		// definition of signal handler
int wait_flag=TRUE;                     		// TRUE while no signal received
char devicename[80];
long Baud_Rate = 115200;         				// default Baud Rate (110 through 38400)
long BAUD;                      				// derived baud rate from command line
long DATABITS;
long STOPBITS;
long PARITYON;
long PARITY;
int Data_Bits = 8;              				// Number of data bits
int Stop_Bits = 1;              				// Number of stop bits
int Parity = 0;                 				// Parity as follows:
												// 00 = NONE, 01 = Odd, 02 = Even, 03 = Mark, 04 = Space
int Format = 4;
FILE *input;
FILE *output;
int status;

char message[90];

// Other stuff from: http://ubuntuforums.org/showthread.php?t=1395180
#include <stdio.h>
#include <string.h>
#include <fcntl.h> 								// File control
#include <errno.h> 								// Error number def

#define DEFAULT_SERIAL_POLLING_RATE			 	0.04

// END SERIAL COMMS STUFF

#define DEFAULT_READ_RATE 						25.0
#define MAX_READ_RATE 							100.0
#define FASTEST_READ_RATE 						1000.0

#define DEFAULT_IMAGE_WIDTH 					640
#define DEFAULT_IMAGE_HEIGHT 					480

#define MAX_ROWS 								1920
#define MAX_COLS 								2560

#define MAX_THERMISTOR_READINGS_TO_STORE 		100

#define UVC_STREAM_PTS                                  (1 << 2)
#define UVC_STREAM_SCR                                  (1 << 3)

#define NO_REPUBLISH_CODE 0
#define REPUBLISH_CODE_8BIT_MONO 1
#define REPUBLISH_CODE_8BIT_COL 2
#define REPUBLISH_CODE_16BIT 3

#define SYNCMODE_HARD						0
#define SYNCMODE_SOFT						1
#define SYNCMODE_IMAGEONLY					2

#define CALIBMODE_OFF						0
#define CALIBMODE_ALT_SHUTTER				1 // Alternates shutter and sends NUCs
#define CALIBMODE_ALT_OUTPUT				2 // Alternates between RAW and INS camera outputs
#define CALIBMODE_LONG_SHUTTER				3 // Alternates shutter, but lets NUCs occur naturally

#define MIN_THERMISTOR_READNG				-20.0
#define MAX_THERMISTOR_READING				60.0

#define SERIAL_COMMS_CONFIG_DEFAULT			0	// Seems to work with the 1-1 USB serial converter
#define SERIAL_COMMS_CONFIG_1				1	// Seems to work with the USB-serial Hub

const char __PROGRAM__[] = "STREAMER";

#ifdef _BUILD_FOR_ROS_
typedef dynamic_reconfigure::Server < thermalvis::streamerConfig > Server;
#endif

bool wantsToShutdown = false;
void mySigintHandler(int sig);
int getMapIndex(string mapping);
void getMapping(int mapIndex, bool extremes, int& mapCode, int& mapParam);

/// \brief		Stores some basic camera information in OpenCV format
struct camData_ {
	cv::Mat cameraMatrix, distCoeffs, imageSize;
	
	cv::Size cameraSize;
	cv::Mat newCamMat;
	
	cv::Mat R, T;
};

//HGH
/// \brief		Stores some basic extrinsic camera information in OpenCV format
struct camExtrinsicsData_ {
    cv::Mat R, T;
    cv::Mat R0, R1, T0, T1, P0, P1, cameraMatrix0, cameraMatrix1, distCoeffs0, distCoeffs1;
    cv::Rect roi0, roi1;
};

/// \brief		Manages the driver / streamer
class streamerNode {
private:

	#ifdef _BUILD_FOR_ROS_
	image_transport::ImageTransport *it;
	image_transport::Subscriber image_sub;
	ros::Subscriber info_sub;
	image_transport::CameraSubscriber camera_sub;
	ros::Subscriber nuc_management_sub;
	ros::Time info_time, image_time, original_time, dodgeTime, lastFlagReceived, lastNucPerformed_at_the_earliest;
	cv_bridge::CvImagePtr cv_ptr;
	sensor_msgs::Image msg_color, msg_16bit, msg_8bit;

	ros::Timer timer, serial_timer;
	image_transport::Publisher pub_color_im;
	image_transport::Publisher pub_16bit_im;
	image_transport::Publisher pub_8bit_im;
        //HGH
        image_transport::Publisher pub_republish_im;
	
	image_transport::CameraPublisher pub_color;
	image_transport::CameraPublisher pub_16bit;
	image_transport::CameraPublisher pub_8bit;
        //HGH
        image_transport::CameraPublisher pub_republish;
	sensor_msgs::CameraInfo original_camera_info, camera_info;
	ros::ServiceServer set_camera_info;
	dynamic_reconfigure::Server<thermalvis::streamerConfig> server;
	dynamic_reconfigure::Server<thermalvis::streamerConfig>::CallbackType f;
	#endif

	char nodeName[256];
	
	double lastMinDisplayTemp, lastMaxDisplayTemp;
	bool deviceCreated;
	bool currentDesiredNucProtectionMode, currentNucProtectionMode;
	bool wantsToDisableNuc;

	bool firstCall;
	
	bool settingsDisabled;
	
	unsigned int recordedThermistorReadings;
	double thermistorBuffer[MAX_THERMISTOR_READINGS_TO_STORE][2];
	
	bool canRadiometricallyCorrect;
	
	double lastDisplayed;
	
	rScheme radMapper;
	
	bool lastIsDuplicate;
	
	bool updateNucInterval;
	bool updateDetectorMode, updateUSBMode;
	
	bool altStatus;
	bool performingNuc;
	
	ofstream ofs;

	int alternateCounter;
	
	double pastMeans[256];
	int pastMeanIndex;
	
	string callLogFile, retrieveLogFile, internalLogFile, writeLogFile, duplicatesLogFile, thermistorLogFile;
	ofstream ofs_call_log, ofs_internal_log, ofs_retrieve_log, ofs_write_log, ofs_duplicates_log, ofs_thermistor_log;
	
	cv::Mat testMat, temperatureMat;
	
	double medianPercentile;
	
	float lastThermistorReading, newThermistorReading;
	
	streamerData configData;
	
	float originalInternalTime;
	
	
	
	double shiftDiff;
	
	unsigned long original_bx, original_by;

	
	unsigned long internal_time;
	
	#ifdef _WIN32
	signed long long int firmwareTime;
	#else
	int64_t firmwareTime;
	#endif
	

	bool firstServerCallbackProcessed;
	
	double fusionFactor;
	bool centerPrincipalPoint;

	double lastMedian, newMedian, lastLowerLimit, lastUpperLimit, oldMaxDiff;

	double minVal, maxVal;
	
	cv::Mat lastFrame;

	bool firstFrame;

	cv::Mat map1, map2;
	
	cv::Mat rzMat;

	cScheme colourMap;

	int writeIndex;
	int frameCounter;
	int lastWritten;
	
	cv::VideoWriter vid_writer;
	
	bool videoInitialized;

	bool readyToPublish;
	bool alphaChanged;

	

	cv::Mat _8bitMat, _16bitMat, colourMat;
	cv::Mat preFilteredMat, smoothedMat, scaled16Mat;
	cv::Mat _8bitMat_pub, _16bitMat_pub, colourMat_pub;
	cv::Mat newImage, normalizedMat, frame, workingFrame, undistorted;
	
	

	camData_ globalCameraInfo;
        //HGH
        camExtrinsicsData_ globalExtrinsicsData;


	

	bool isActuallyGray;

	
	
	vector<string> inputList;
	int fileCount;
	
	
	
	bool videoValid;

#ifndef _WIN32
	streamerSource *mainVideoSource;
#endif

	cv::VideoCapture cap;
	
	
	
	// Serial Comms
	int mainfd; /* File descriptor */


	
public:

	#ifdef _BUILD_FOR_ROS_
	streamerNode(ros::NodeHandle& nh, streamerData startupData);
	void handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg);
	void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr);
	void handle_info(const sensor_msgs::CameraInfoConstPtr& info_msg);
	void handle_nuc_instruction(const std_msgs::Float32::ConstPtr& nuc_msg);
	bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);
	void serialCallback(const ros::TimerEvent& e);
	void timerCallback(const ros::TimerEvent& e);
	void serverCallback(thermalvis::streamerConfig &config, uint32_t level);
	void updateCameraInfo();
	void assignCameraInfo();
	void assignDefaultCameraInfo();
	void updateCameraInfoExtrinsics();
	#endif
	
	CvCapture* capture;
	
	bool isVideoValid();
	void setValidity(bool val);

	

#ifndef _WIN32
	streamerSource * getMainVideoSource();
#endif

	cv::VideoCapture * getVideoCapture();

	bool streamCallback(bool capture = true);
	void acceptImage(void *ptr);
	bool processImage();
	void publishTopics();
	void writeData();
	bool processFolder();
	
	
	void overwriteCameraDims();

	

	bool sendSerialCommand(char *command, int max_attempts = 1);

        //HGH
        

	
	void refreshCameraAdvertisements();
	void initializeMessages();
	
	void act_on_image();
	
	// Source alternatives
	bool runBag();
	bool runRead();
	bool runLoad();
	bool runDevice();
	
	bool performNuc();
	
	void updateMap();
	
	void prepareForTermination();
	
	void markCurrentFrameAsDuplicate();
	
	bool getNucSettingsReading(int& delay, double& diff);
	
	bool configureSerialComms();
	float getThermistorReading();
	double smoothThermistorReading();
	
	bool setupDevice();
	void releaseDevice();
	
	//int mygetch();
	int open_port();

        //HGH
        void getRectification();
	
};

#endif
#endif