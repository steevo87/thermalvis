#ifndef _WINDOWS_H_
#define _WINDOWS_H_

#define NOMINMAX 1	  // Don't remove this, even though it causes a warning!
#include <windows.h>
#include <time.h>
#include <math.h>

#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif

// http://social.msdn.microsoft.com/Forums/vstudio/en-US/430449b3-f6dd-4e18-84de-eebd26a8d668/gettimeofday?forum=vcgeneral
struct timezone 
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};
int gettimeofday(struct timeval *tv, struct timezone *tz);

// http://stackoverflow.com/questions/485525/round-for-float-in-c
/*
double round(double d) 
{
  return floor(d + 0.5);
}
*/

#endif // _WINDOWS_H_
