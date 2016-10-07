/*! \file	video.hpp
 *  \brief	Declarations for managing video sources and formats.
*/

#ifndef THERMALVIS_VIDEO_H
#define THERMALVIS_VIDEO_H

#if !defined(_IS_WINDOWS_) && defined(_AVLIBS_AVAILABLE_)

#include "core/general_resources.hpp"
#include "core/ros_resources.hpp"

#include <asm/types.h>          /* for videodev2.h */

/***** Add FFMPEG AVI Stuff *****/
#ifndef INT64_C
#	define INT64_C(c) (c ## LL)
#	define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
	#include <linux/videodev2.h>
	#include <libavcodec/avcodec.h>
	#include <libavformat/avformat.h>
	//#include <libavdevice/avdevice.h>
	#include <libswscale/swscale.h>
}

// http://credentiality2.blogspot.com.au/2010/04/v4l2-example.html

#ifdef _BUILD_FOR_ROS_
namespace enc = sensor_msgs::image_encodings;
#endif

#define CODEC_TYPE_VIDEO AVMEDIA_TYPE_VIDEO
#define CODEC_TYPE_AUDIO AVMEDIA_TYPE_AUDIO

#define CLEAR(x) memset (&(x), 0, sizeof (x))

/// \brief		Stores critical USB camera information
typedef struct {
  int width;
  int height;
  int bytes_per_pixel;
  int image_size;
  char *image;
  int is_new;
} usb_cam_camera_image_t;

typedef enum {
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
} usb_cam_io_method;

typedef enum {
  PIXEL_FORMAT_YUYV,
  PIXEL_FORMAT_UYVY,
  PIXEL_FORMAT_MJPEG,
} usb_cam_pixel_format;

/// \brief		Buffer for storing USB video data
struct usb_buffer {
  void * start;
  size_t length;
};

/// \brief		Interacts with LIBAV and other relevant libraries to read video data from USB
class streamerSource {
public:
	
	struct SwsContext *video_sws;

	//AVFormatContext *pFormatCtx;
	int i, videoStream;
	AVCodecContext *pCodecCtx;
	AVCodec *pCodec;
	AVFrame *pFrame;
	AVFrame *pFrameRGB;
	AVPacket packet;
	int frameFinished;
	int numBytes;
	uint8_t *buffer;
	uint8_t *videoBuffer;

	char *camera_dev;
	unsigned int pixelformat;
	
	usb_cam_io_method io;
	int fd;
	struct usb_buffer * buffers;
	unsigned int n_buffers;
	AVFrame *avframe_camera;
	AVFrame *avframe_rgb;
	AVCodec *avcodec;
	AVCodecContext *avcodec_context;
	int avframe_camera_size;
	int avframe_rgb_size;
	usb_cam_camera_image_t* image;

	// Portugese one:
	int bRet, ix;
	// Input
	    //const char *sFile = "/dev/video0";
	    AVFormatContext *pIFormatCtx;
	    AVCodecContext *pICodecCtx;
	    AVCodec *pICodec;
	    //AVFrame *pFrame;
	    int ixInputStream;
	    AVInputFormat *pIFormat;
	    AVPacket oPacket;
	    int fFrame;
	// Output
	    AVCodecContext *pOCodecCtx;
	    AVCodec *pOCodec;
	    //uint8_t *pBuffer;
	    int szBuffer;
	    int szBufferActual;
	    int bImgFormat;
	    int bQuality;
	    FILE *fdJPEG;

	/*
	static usb_cam_io_method io = IO_METHOD_MMAP;
	static int fd = -1;
	struct buffer * buffers = NULL;
	static unsigned int n_buffers = 0;
	static AVFrame *avframe_camera = NULL;
	static AVFrame *avframe_rgb = NULL;
	static AVCodec *avcodec = NULL;
	static AVCodecContext *avcodec_context = NULL;
	static int avframe_camera_size = 0;
	static int avframe_rgb_size = 0;
	*/

	// Generic functions
	void initialize_video_source();

	// File functions
	int setup_video_file(std::string filename);

	int close_video_file(std::string filename);

	// Capture functions
	int setup_video_capture(std::string devicename, int& deviceWidth, int& deviceHeight, bool verbose = false);
	void close_video_capture();


	void usb_cam_camera_start(const char* dev, usb_cam_io_method io, usb_cam_pixel_format pf, int width, int height);
	// shutdown camera
	void usb_cam_camera_shutdown(void);
	// grabs a new image from the camera
	void usb_cam_camera_grab_image();

	void open_device();
	void close_device();
	void init_device(int image_width, int image_height);
	void init_userp(unsigned int buffer_size);
	void init_mmap();
	void init_read(unsigned int buffer_size);
	void uninit_device();
	void start_capturing();
	void stop_capturing();
	int read_frame();
	void process_image(const void * src, int len, usb_cam_camera_image_t *dest);
	void mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels);
	int init_mjpeg_decoder(int image_width, int image_height);
	void yuyv2rgb(char *YUV, char *RGB, int NumPixels);
	void uyvy2rgb (char *YUV, char *RGB, int NumPixels);

	void YUV2RGB(const unsigned char y,
		const unsigned char u,
		const unsigned char v,
		unsigned char* r,
		unsigned char* g,
		unsigned char* b);

	//unsigned char CLIPVALUE(int val);

	int xioctl(int fd, int request, void * arg);
	void errno_exit(const char * s);

};

#endif // THERMALVIS_VIDEO_H
#endif
