/*! \file	video.cpp
 *  \brief	Definitions for managing video sources and formats.
*/

#if !defined(_IS_WINDOWS_) && defined(_AVLIBS_AVAILABLE_)
	
#include "streamer/video.hpp"

#define __CONFIG_0__

void streamerSource::initialize_video_source() {

	bImgFormat = PIX_FMT_YUYV422; // original: PIX_FMT_YUVJ420P, then tried PIX_FMT_YUV420P
	bQuality = 3;
	//int fFrame = 0;
	video_sws = NULL;
	
	ixInputStream = -1;
	io = IO_METHOD_MMAP; //IO_METHOD_MMAP;
	fd = -1;
	buffers = NULL;
	n_buffers = 0;
	avframe_camera = NULL;
	avframe_rgb = NULL;
	avcodec = NULL;
	avcodec_context = NULL;
	avframe_camera_size = 0;
	avframe_rgb_size = 0;
	image = (usb_cam_camera_image_t *) calloc(1, sizeof(usb_cam_camera_image_t));

    /***** FFMPEG AVI Initialization *****/
    printf("%s << Initializing video source...\n", __FUNCTION__);
	av_init_packet(&packet);
    printf("%s << Registering video source...\n", __FUNCTION__);
	av_register_all();
    printf("%s << Video source initialized and registered.\n", __FUNCTION__);

};

int streamerSource::close_video_file(std::string filename) {
    // Free the RGB image
    av_free(buffer);
    av_free(pFrameRGB);

    // Free the YUV frame
    av_free(pFrame);

    // Close the codec
    avcodec_close(pCodecCtx);

    // Close the video file
    //av_close_input_file(pIFormatCtx);
    avformat_close_input(&pIFormatCtx);
    
    return 0;
}


int streamerSource::setup_video_file(std::string filename) {
	
	//return 0;
	
	printf("%s << Trying to set up file...\n", __FUNCTION__);
    //av_register_all();
	// Open video file
	
	pIFormatCtx = NULL;
	// AVInputFormat *avIF = av_find_input_format("video4linux2");
	
#ifdef __CONFIG_0__ // Steve's config
    //pFormatCtx = NULL;
    printf("%s << Trying <avformat_open_input> with (%s)...\n", __FUNCTION__, filename.c_str());
    int err = avformat_open_input (&pIFormatCtx, filename.c_str(), NULL, NULL);
#endif

#ifdef __CONFIG_1__ // Hajmi's config
    ROS_INFO("%s << Trying <av_open_input_file> with (%s)...\n", __FUNCTION__, filename.c_str());
    int err = (av_open_input_file(&pIFormatCtx, filename.c_str(), NULL, 0, NULL) != 0);
#endif
	
	if (err != 0) {
		//if (av_open_input_file(&pFormatCtx, filename.c_str(), NULL, 0, NULL) != 0) {
			
			
			if (err == ENOMEM) {
				printf("%s << Error = ENOMEM\n", __FUNCTION__);
			} else if (err == EINVAL) {
				printf("%s << Error = EINVAL\n", __FUNCTION__);
			} else {

				printf("%s << Error = (%d)\n", __FUNCTION__, err);
			}
			
            char error_string[256];
            av_strerror	(err, error_string, sizeof(error_string));
            printf("%s << Error Description = (%s)\n", __FUNCTION__, error_string);
            //print_error_and_exit(err, "avformat_open_input()");

			//printf("%s << Couldn't open file (%s)...\n", __FUNCTION__, filename.c_str());
			
			return -1; // Couldn't open file
		}
	
        

    // Retrieve stream information
    //if (av_find_stream_info(pIFormatCtx) < 0) {
    if (avformat_find_stream_info(pIFormatCtx, NULL) < 0) {
        printf("%s << Couldn't find stream info...\n", __FUNCTION__);
        return -1; // Couldn't find stream information
	}

	#ifdef __CONFIG_0__ // Steve's config
	    // Dump information about file onto standard error
		av_dump_format(pIFormatCtx, 0, filename.c_str(), 0);	
	#endif


    // Find the first video stream
    videoStream = -1;
    for (i = 0; i < pIFormatCtx->nb_streams; i++)
        if (pIFormatCtx->streams[i]->codec->codec_type == CODEC_TYPE_VIDEO)
        {
            videoStream = i;
            break;
        }
    if (videoStream == -1) {
        printf("%s << Didn't find video stream...\n", __FUNCTION__);
        return -1; // Didn't find a video stream
	}

    // Get a pointer to the codec context for the video stream
    pCodecCtx = pIFormatCtx->streams[videoStream]->codec;

    // Find the decoder for the video stream
    pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
    if (pCodec == NULL)
    {
        fprintf(stderr, "Unsupported codec!\n");
        return -1; // Codec not found
    }
    // Open codec
    // if (avcodec_open(pCodecCtx, pCodec) < 0) {
	if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
        printf("%s << Couldn't open codec...\n", __FUNCTION__);
        return -1; // Could not open codec
	}

    // Allocate video frame
    pFrame = avcodec_alloc_frame();

    // Allocate an AVFrame structure
    pFrameRGB = avcodec_alloc_frame();
    if (pFrameRGB == NULL) {
        printf("%s << Couldn't allocate frame...\n", __FUNCTION__);
        return -1;
	}

    // Determine required buffer size and allocate buffer
    numBytes = avpicture_get_size(PIX_FMT_RGB24, pCodecCtx->width,
            pCodecCtx->height);
    buffer = (uint8_t *) av_malloc(numBytes * sizeof(uint8_t));

    // Assign appropriate parts of buffer to image planes in pFrameRGB
    // Note that pFrameRGB is an AVFrame, but AVFrame is a superset
    // of AVPicture
    avpicture_fill((AVPicture *) pFrameRGB, buffer, PIX_FMT_RGB24,
            pCodecCtx->width, pCodecCtx->height);
            
	return 0;
}

void streamerSource::close_video_capture() {
	
	avformat_close_input (&pIFormatCtx);
	//av_close_input_file(pIFormatCtx);
	
}

int streamerSource::setup_video_capture(std::string devicename, int& deviceWidth, int& deviceHeight, bool verbose) {

    if (verbose) { printf("%s << Registering...\n", __FUNCTION__); }
	av_register_all();

    if (verbose) { printf("%s << Finding input video format...\n", __FUNCTION__); }
    pIFormat = av_find_input_format("video4linux2");
    pIFormatCtx = NULL;
    
    if (verbose) { printf("%s << Opening format...\n", __FUNCTION__); }
    int err = avformat_open_input (&pIFormatCtx, devicename.c_str(), pIFormat, NULL);
    // int err = av_open_input_file(&pIFormatCtx, devicename.c_str(), pIFormat, 0, NULL);

	if (err != 0) {
        if (err == ENOMEM) {
            printf("%s << Error opening (%s) with avformat_open_input() = (ENOMEMd)\n", __FUNCTION__, devicename.c_str());
        } else if (err == EINVAL) {
            printf("%s << Error opening (%s) with avformat_open_input() = (EINVAL)\n", __FUNCTION__, devicename.c_str());
        } else {
            printf("%s << Error opening (%s) with avformat_open_input() = (%d)\n", __FUNCTION__, devicename.c_str(), err);
        }

        return 1;
    }

    /* Retrieve stream information */
    if (verbose) { printf("%s << Retrieving stream info...\n", __FUNCTION__); }
    //if (av_find_stream_info(pIFormatCtx) < 0) {
	if (avformat_find_stream_info(pIFormatCtx, NULL) < 0) {
        fprintf(stderr, "No stream info\n");
        return 1;
    }
 
    /* Find the first video stream */
    ixInputStream = -1;
    if (verbose) { printf("%s << Finding first stream...\n", __FUNCTION__); }
    for (ix = 0; ix < pIFormatCtx->nb_streams; ix++) {
        if (pIFormatCtx->streams[ix]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
            ixInputStream = ix;
            break;
        }
    }
    if (ixInputStream == -1) {
        fprintf(stderr, "No video stream in file\n");
        return 1;
    }
 
    /* Get a pointer to the codec context for the video stream */
    if (verbose) { printf("%s << Obtain codec context pointer...\n", __FUNCTION__); }
    pICodecCtx = pIFormatCtx->streams[ixInputStream]->codec;
 
    /* Find the decoder for the video stream */
    if (verbose) { printf("%s << Find decoder...\n", __FUNCTION__); }
    pICodec = avcodec_find_decoder(pICodecCtx->codec_id);
    if (!pICodec) {
        fprintf(stderr, "Codec not found\n");
        return 1;
    }
 
    /* Open input codec */
    if (verbose) { printf("%s << Open input codec...\n", __FUNCTION__); }
    //if (avcodec_open(pICodecCtx, pICodec) < 0) {
	if (avcodec_open2(pICodecCtx, pICodec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        return 1;
    }
 
    /* Allocate video frame */
    if (verbose) { printf("%s << Allocating video frame...\n", __FUNCTION__); }
    pFrame = avcodec_alloc_frame();
 
    /* Determine required buffer size and allocate buffer */
	//printf("%s << DEBUG: Hack %d\n", __FUNCTION__, 32151);
	//if (verbose) { printf("%s << Getting buffer size...\n", __FUNCTION__); }
	if (verbose) { printf("%s << Video stream frame size = (%d, %d)\n", __FUNCTION__, pICodecCtx->width, pICodecCtx->height); }
    
    deviceWidth = pICodecCtx->width;
    deviceHeight = pICodecCtx->height;
    
    //szBuffer = avpicture_get_size((PixelFormat) bImgFormat, pICodecCtx->width, pICodecCtx->height);
    //if (verbose) { printf("%s << Buffer size = (%d {%d}: [%d,%d])\n", __FUNCTION__, szBuffer, bImgFormat, pICodecCtx->width, pICodecCtx->height); }
    //pBuffer = (uint8_t*) av_mallocz( szBuffer);
 
    /* Allocate Output Codec */
    if (verbose) { printf("%s << Allocating output codec...\n", __FUNCTION__); }
    //pOCodecCtx = avcodec_alloc_context();
    pOCodecCtx = avcodec_alloc_context3(NULL);
    if (!pOCodecCtx) {
        fprintf(stderr, "Could not allocate codec\n");
        return 1;
    }
 
    /* Initialize picture size and other format parameters */
    if (verbose) { printf("%s << Initializing image size etc...\n", __FUNCTION__); }
    pOCodecCtx->bit_rate = pICodecCtx->bit_rate;
    pOCodecCtx->width = pICodecCtx->width;
    pOCodecCtx->height = pICodecCtx->height;
    pOCodecCtx->pix_fmt = (PixelFormat) bImgFormat;
    pOCodecCtx->codec_id = CODEC_ID_RAWVIDEO; //CODEC_ID_MJPEG;
    pOCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    pOCodecCtx->time_base.num = pICodecCtx->time_base.num;
    pOCodecCtx->time_base.den = pICodecCtx->time_base.den;
 
    /* Allocate codec for JPEG */
    if (verbose) { printf("%s << Allocating codec for JPEG...\n", __FUNCTION__); }
    pOCodec = avcodec_find_encoder(pOCodecCtx->codec_id);
    if (!pOCodec) {
        fprintf(stderr, "Codec not found\n");
        return 1;
    }
    
    if (verbose) { printf("%s << Attempting to open codec...\n", __FUNCTION__); }
    //if (avcodec_open(pOCodecCtx, pOCodec) < 0) {
	if (avcodec_open2(pOCodecCtx, pOCodec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        return 1;
    }
 
    /* Initialize all VBR settings */
    if (verbose) { printf("%s << Initializing VBR settings...\n", __FUNCTION__); }
    pOCodecCtx->qmin = pOCodecCtx->qmax = bQuality;
    pOCodecCtx->mb_lmin = pOCodecCtx->lmin = pOCodecCtx->qmin * FF_QP2LAMBDA;
    pOCodecCtx->mb_lmax = pOCodecCtx->lmax = pOCodecCtx->qmax * FF_QP2LAMBDA;
    pOCodecCtx->flags |= CODEC_FLAG_QSCALE;
    pOCodecCtx->global_quality = pOCodecCtx->qmin * FF_QP2LAMBDA;
    
    if (verbose) { printf("%s << Exiting...\n", __FUNCTION__); }
    
    return 0;
}

void streamerSource::errno_exit(const char * s)
{
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));

	//printf("%s << HERE (%d) \n", __FUNCTION__, 0);
  exit(EXIT_FAILURE);
  //printf("%s << THERE (%d)\n", __FUNCTION__, 1);
}


int streamerSource::xioctl(int fd, int request, void * arg)
{
  int r;

  do
    r = ioctl(fd, request, arg); while (-1==r&&EINTR==errno);

  return r;
}



/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
//unsigned char streamerSource::CLIPVALUE(int val)
//{
  // Old method (if)
/*   val = val < 0 ? 0 : val; */
/*   return val > 255 ? 255 : val; */

  // New method (array)
//  return uchar_clipping_table[val+clipping_table_offset];
//}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
void streamerSource::YUV2RGB(const unsigned char y,
        const unsigned char u,
        const unsigned char v,
        unsigned char* r,
        unsigned char* g,
        unsigned char* b)
{
  //const int y2=(int)y;
  //const int u2=(int)u-128;
  //const int v2=(int)v-128;
  //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;


  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  //int r2 = y2 + ( (v2*37221) >> 15);
  //int g2 = y2 - ( ((u2*12975) + (v2*18949)) >> 15 );
  //int b2 = y2 + ( (u2*66883) >> 15);
  //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;


  // Cap the values.
	/*
  *r=CLIPVALUE(r2);
  *g=CLIPVALUE(g2);
  *b=CLIPVALUE(b2);
	*/
}

void streamerSource::uyvy2rgb (char *YUV, char *RGB, int NumPixels) {
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r = 0, g = 0, b = 0;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      u = (unsigned char) YUV[i + 0];
      y0 = (unsigned char) YUV[i + 1];
      v = (unsigned char) YUV[i + 2];
      y1 = (unsigned char) YUV[i + 3];
      YUV2RGB (y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}

void streamerSource::yuyv2rgb(char *YUV, char *RGB, int NumPixels) {
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r = 0, g = 0, b = 0;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      y0 = (unsigned char) YUV[i + 0];
      u = (unsigned char) YUV[i + 1];
      y1 = (unsigned char) YUV[i + 2];
      v = (unsigned char) YUV[i + 3];
      YUV2RGB (y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}

int streamerSource::init_mjpeg_decoder(int image_width, int image_height)
{
  //avcodec_init();
  av_register_all();
  avcodec_register_all();

  avcodec = avcodec_find_decoder(CODEC_ID_MJPEG);
  if (!avcodec)
  {
    fprintf(stderr,"Could not find MJPEG decoder\n");
    return 0;
  }

  //avcodec_context = avcodec_alloc_context();
  avcodec_context = avcodec_alloc_context3(NULL);
  avframe_camera = avcodec_alloc_frame();
  avframe_rgb = avcodec_alloc_frame();

  avpicture_alloc((AVPicture *)avframe_rgb, PIX_FMT_RGB24, image_width, image_height);

  avcodec_context->codec_id = CODEC_ID_MJPEG;
  avcodec_context->width = image_width;
  avcodec_context->height = image_height;

  avframe_camera_size = avpicture_get_size(PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size = avpicture_get_size(PIX_FMT_RGB24, image_width, image_height);

  /* open it */
  //if (avcodec_open(avcodec_context, avcodec) < 0) {
  if (avcodec_open2(avcodec_context, avcodec, NULL) < 0) {
    fprintf(stderr,"Could not open MJPEG Decoder\n");
    return 0;
  }
  return 1;
}

void streamerSource::mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels)
{
  int got_picture;

  memset(RGB, 0, avframe_rgb_size);
	
	printf("%s << Entering attempted fix...\n", __FUNCTION__);

  //avcodec_decode_video(avcodec_context, avframe_camera, &got_picture, (uint8_t *) MJPEG, len);

	av_init_packet(&packet);

	avcodec_decode_video2(avcodec_context, avframe_camera, &got_picture, &packet);

	printf("%s << Exiting attempted fix...\n", __FUNCTION__);

  if (!got_picture) {
    fprintf(stderr,"Webcam: expected picture but didn't get it...\n");
    return;
  }

  int xsize = avcodec_context->width;
  int ysize = avcodec_context->height;
  int pic_size = avpicture_get_size(avcodec_context->pix_fmt, xsize, ysize);
  if (pic_size != avframe_camera_size) {
    fprintf(stderr,"outbuf size mismatch.  pic_size: %d bufsize: %d\n",pic_size,avframe_camera_size);
    return;
  }

  video_sws = sws_getContext( xsize, ysize, avcodec_context->pix_fmt, xsize, ysize, PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);
  sws_scale(video_sws, avframe_camera->data, avframe_camera->linesize, 0, ysize, avframe_rgb->data, avframe_rgb->linesize );
//  img_convert((AVPicture *) avframe_rgb, PIX_FMT_RGB24, (AVPicture *) avframe_camera, avcodec_context->pix_fmt, xsize, ysize);

  int size = avpicture_layout((AVPicture *) avframe_rgb, PIX_FMT_RGB24, xsize, ysize, (uint8_t *)RGB, avframe_rgb_size);
  if (size != avframe_rgb_size) {
    fprintf(stderr,"webcam: avpicture_layout error: %d\n",size);
    return;
  }
}

void streamerSource::process_image(const void * src, int len, usb_cam_camera_image_t *dest)
{
  if(pixelformat==V4L2_PIX_FMT_YUYV)
    yuyv2rgb((char*)src, dest->image, dest->width*dest->height);
  else if(pixelformat==V4L2_PIX_FMT_UYVY)
    uyvy2rgb((char*)src, dest->image, dest->width*dest->height);
  else if(pixelformat==V4L2_PIX_FMT_MJPEG)
    mjpeg2rgb((char*)src, len, dest->image, dest->width*dest->height);
}

int streamerSource::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  switch (io) {
  case IO_METHOD_READ:

	printf("%s << IO_METHOD_READ\n", __FUNCTION__);
    len = read(fd, buffers[0].start, buffers[0].length);
    if (len==-1) {
      switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("read");
      }
    }

    process_image(buffers[0].start, len, image);

    break;

  case IO_METHOD_MMAP:

	printf("%s << IO_METHOD_MMAP\n", __FUNCTION__);
    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1==xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }

    assert (buf.index < n_buffers);
    len = buf.bytesused;

	printf("%s << IO_METHOD_MMAP: Processing image...\n", __FUNCTION__);
    process_image(buffers[buf.index].start, len, image);

    if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");

    break;

  case IO_METHOD_USERPTR:

	printf("%s << IO_METHOD_USERPTR\n", __FUNCTION__);
    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;

    if (-1==xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }

    for(i = 0; i<n_buffers; ++i)
      if (buf.m.userptr==(unsigned long) buffers[i].start&&buf.length==buffers[i].length)
        break;

    assert (i < n_buffers);
    len = buf.bytesused;
    process_image((void *) buf.m.userptr, len, image);

    if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");

    break;
  }

  return 1;
}

void streamerSource::stop_capturing(void)
{
  enum v4l2_buf_type type;

  switch (io) {
	  case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	  case IO_METHOD_MMAP:
	  case IO_METHOD_USERPTR:
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)) {
			printf("%s << DEBUG (%d)\n", __FUNCTION__, 0);
		  errno_exit("VIDIOC_STREAMOFF");
		  printf("%s << DEBUG (%d)\n", __FUNCTION__, 1);
	  }
		break;
  }
}

void streamerSource::start_capturing(void)
{
  unsigned int i;
  enum v4l2_buf_type type;

  switch (io) {
  case IO_METHOD_READ:
    /* Nothing to do. */
    break;

  case IO_METHOD_MMAP:
	printf("%s << method = IO_METHOD_MMAP\n", __FUNCTION__);
    for(i = 0; i<n_buffers; ++i) {
      struct v4l2_buffer buf;

      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1==xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");

    break;

  case IO_METHOD_USERPTR:
	printf("%s << method = IO_METHOD_USERPTR\n", __FUNCTION__);
    for(i = 0; i<n_buffers; ++i) {
      struct v4l2_buffer buf;

      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = i;
      buf.m.userptr = (unsigned long) buffers[i].start;
      buf.length = buffers[i].length;

      if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1==xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");

    break;
  }
  
  std::cin.get();
}


void streamerSource::uninit_device(void)
{
  unsigned int i;

  switch (io) {
  case IO_METHOD_READ:
    free(buffers[0].start);
    break;

  case IO_METHOD_MMAP:
    for(i = 0; i<n_buffers; ++i)
      if (-1==munmap(buffers[i].start, buffers[i].length))
        errno_exit("munmap");
    break;

  case IO_METHOD_USERPTR:
    for(i = 0; i<n_buffers; ++i)
      free(buffers[i].start);
    break;
  }

  free(buffers);
}

void streamerSource::init_read(unsigned int buffer_size)
{
	printf("%s << DEBUG %d (n_buffers = %d)\n", __FUNCTION__, 21421, n_buffers);
  
	//buffers = new buffer;
 buffers = (usb_buffer*)calloc(1, sizeof(*buffers));

	//buffers = (buffer*)calloc(1, sizeof(buffer));


	//buffers = new buffer[n_buffers];

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  buffers[0].length = buffer_size;
  buffers[0].start = malloc(buffer_size);

  if (!buffers[0].start) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
}

void streamerSource::init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1==xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL==errno) {
      fprintf(stderr, "%s does not support memory mapping\n", camera_dev);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count<2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", camera_dev);
    exit(EXIT_FAILURE);
  }

	printf("%s << DEBUG %d\n", __FUNCTION__, 23981);

	// MONKEY

	printf("%s << n_buffers = %d (%d, %d)\n", __FUNCTION__, n_buffers, req.count, int(sizeof(*buffers)));

	/*
	for (int iii = 0; iii < req.count; iii++) {
		// buffers[iii] = new buffer;
	}
	*/
	
	//buffers = usb_buffer

    buffers = (usb_buffer*) calloc(req.count, sizeof(*buffers));
	//buffers = new buffer*[req.count];

// DyBytes*[MAX_SIZE];

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  for(n_buffers = 0; n_buffers<req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1==xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ|PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);

    if (MAP_FAILED==buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

void streamerSource::init_userp(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  buffer_size = (buffer_size+page_size-1)&~(page_size-1);

  CLEAR (req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1==xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL==errno) {
      fprintf(stderr, "%s does not support "
        "user pointer i/o\n", camera_dev);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

	printf("%s << DEBUG %d\n", __FUNCTION__, 62626);
	   buffers = (usb_buffer*)calloc(4, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  for(n_buffers = 0; n_buffers<4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = memalign(/* boundary */page_size, buffer_size);

    if (!buffers[n_buffers].start) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }
  }
}

void streamerSource::init_device(int image_width, int image_height)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1==xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL==errno) {
      fprintf(stderr, "%s is no V4L2 device\n", camera_dev);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities&V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n", camera_dev);
    exit(EXIT_FAILURE);
  }

  switch (io) {
  case IO_METHOD_READ:
    if (!(cap.capabilities&V4L2_CAP_READWRITE)) {
      fprintf(stderr, "%s does not support read i/o\n", camera_dev);
      exit(EXIT_FAILURE);
    }

    break;

  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    if (!(cap.capabilities&V4L2_CAP_STREAMING)) {
      fprintf(stderr, "%s does not support streaming i/o\n", camera_dev);
      exit(EXIT_FAILURE);
    }

    break;
  }

  /* Select video input, video standard and tune here. */

  CLEAR (cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0==xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1==xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        /* Cropping not supported. */
        break;
      default:
        /* Errors ignored. */
        break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR (fmt);

//  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//  fmt.fmt.pix.width = 640;
//  fmt.fmt.pix.height = 480;
//  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
//  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = pixelformat;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;


  if (-1==xioctl(fd, VIDIOC_S_FMT, &fmt))
    errno_exit("VIDIOC_S_FMT");

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width*2;
  if (fmt.fmt.pix.bytesperline<min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline*fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage<min)
    fmt.fmt.pix.sizeimage = min;

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  switch (io) {
  case IO_METHOD_READ:
    init_read(fmt.fmt.pix.sizeimage);
    break;

  case IO_METHOD_MMAP:
    init_mmap();
    break;

  case IO_METHOD_USERPTR:
    init_userp(fmt.fmt.pix.sizeimage);
    break;
  }
}

void streamerSource::close_device()
{
  if (-1==close(fd))
    errno_exit("close");

  fd = -1;
}

void streamerSource::open_device()
{
  struct stat st;

  if (-1==stat(camera_dev, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n", camera_dev, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR (st.st_mode)) {
    fprintf(stderr, "%s is no device\n", camera_dev);
    exit(EXIT_FAILURE);
  }

  fd = open(camera_dev, O_RDWR /* required */|O_NONBLOCK, 0);

  if (-1==fd) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n", camera_dev, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void streamerSource::usb_cam_camera_start(const char* dev, usb_cam_io_method io_method,
    usb_cam_pixel_format pixel_format, int image_width, int image_height)
{
  camera_dev = (char*)calloc(1,strlen(dev)+1);
  strcpy(camera_dev,dev);

  io = io_method;
  if(pixel_format == PIXEL_FORMAT_YUYV)
    pixelformat = V4L2_PIX_FMT_YUYV;
  else if(pixel_format == PIXEL_FORMAT_UYVY)
    pixelformat = V4L2_PIX_FMT_UYVY;
  else if(pixel_format == PIXEL_FORMAT_MJPEG) {
    pixelformat = V4L2_PIX_FMT_MJPEG;
    init_mjpeg_decoder(image_width, image_height);
  }
  else {
    fprintf(stderr, "Unknown pixelformat.\n");
    exit(EXIT_FAILURE);
  }

  open_device();
  init_device(image_width, image_height);
  start_capturing();

  image->width = image_width;
  image->height = image_height;
  image->bytes_per_pixel = 16;

  image->image_size = image->width*image->height*image->bytes_per_pixel;
  image->is_new = 0;
  image->image = (char *) calloc(image->image_size, sizeof(char));
  memset(image->image, 0, image->image_size*sizeof(char));

}

void streamerSource::usb_cam_camera_shutdown()
{
  stop_capturing();
  uninit_device();
  close_device();

  if (avcodec_context) {
    avcodec_close(avcodec_context);
    av_free(avcodec_context);
    avcodec_context = NULL;
  }
  if (avframe_camera)
    av_free(avframe_camera);
  avframe_camera = NULL;
  if (avframe_rgb)
    av_free(avframe_rgb);
  avframe_rgb = NULL;
}

void streamerSource::usb_cam_camera_grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO (&fds);
  FD_SET (fd, &fds);

  /* Timeout. */
  tv.tv_sec = 2;
  tv.tv_usec = 0;

  r = select(fd+1, &fds, NULL, NULL, &tv);

  if (-1==r) {
    if (EINTR==errno)
      return;

    errno_exit("select");
  }

  if (0==r) {
    fprintf(stderr, "select timeout\n");
    exit(EXIT_FAILURE);
  }

  read_frame();
  image->is_new = 1;
}

#endif
