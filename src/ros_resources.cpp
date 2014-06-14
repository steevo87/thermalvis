/*! \file	ros_resources.cpp
 *  \brief	Definitions for ROS inclusion requirements and ROS-related tools.
*/

#ifdef _BUILD_FOR_ROS_

#include "ros_resources.hpp"

#ifdef _BUILD_FOR_ROS_ // Should definitely be defined if this file is called..
void displayMessage(string msg, int msg_code) {
	switch (msg_code) {
	case MESSAGE_NORMAL:
		ROS_INFO(msg.c_str());
		break;
	case MESSAGE_WARNING:
		ROS_WARNING(msg.c_str());
		break;
	case MESSAGE_ERROR:
		ROS_ERROR(msg.c_str());
		break;
	default:
		ROS_INFO(msg.c_str());
	}
}
#endif

void changemode(int dir)
{
  static struct termios oldt, newt;
 
  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}
 
int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;
 
  tv.tv_sec = 0;
  tv.tv_usec = 0;
 
  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);
 
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);
 
}

double timeDiff(ros::Time time1, ros::Time time2) {
	
	double retVal;
	
	retVal = ((double) time1.sec) - ((double) time2.sec);
	
	retVal += 1e-9 * (((double) time1.nsec) - ((double) time2.nsec));
	
	return retVal;
	
}

ros::Time findAverageTime(ros::Time time1, ros::Time time2) {
	
	long int sec = 0, nsec = 0;
	
	nsec = (time1.nsec/2) + (time2.nsec/2);
	
	//printf("%s << sec = %d\n", __FUNCTION__, sec);
	//printf("%s << nsec = %d\n", __FUNCTION__, nsec);
		
	if ((time1.sec % 2) > 0) {
		sec += (time1.sec-1)/2;
	} else {
		sec += time1.sec/2;
	}
	
	if ((time2.sec % 2) > 0) {
		sec += (time2.sec-1)/2;
	} else {
		sec += time2.sec/2;
	}
	
	if (((time1.sec % 2) > 0) && ((time2.sec % 2) > 0)) {
		sec += 1;
	} else if (((time1.sec % 2) > 0) || ((time2.sec % 2) > 0)) {
		nsec += 500000000;
	}
	
	//printf("%s << sec = %d\n", __FUNCTION__, sec);
	//printf("%s << nsec = %d\n", __FUNCTION__, nsec);
		
	ros::Time avTime;
	
	avTime.sec = sec;
	avTime.nsec = nsec;
	
	return avTime;
	
}

#endif