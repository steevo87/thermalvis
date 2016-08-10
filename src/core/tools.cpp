/*! \file	tools.cpp
 *  \brief	Definitions for generic tools not depending on libraries such as OpenCV, PCL and ROS.
*/

#include "core/tools.hpp"


void InitializeRandomNumberGeneration() 
{
#if _IS_WINDOWS_
	srand ( GetTickCount() );
#else
	srand ( time(NULL) );
#endif
}


// Courtesy of [ J.M.P. van Waveren ]
float ReciprocalSquareRoot( float x ) 
{
	long i;
	float y, r;
	
	y = x * 0.5f;
	i = *(long *)( &x );
	i = 0x5f3759df - ( i >> 1 );
	r = *(float *)( &i );
	r = r * ( 1.5f - r * r * y );
	return r;
}


void FindLinearModel( const double* x, const double* y, const int n, double &m, double &c ) 
{
	double meanX = 0.0;
  double meanY = 0.0;
	
	if ( n == 1 ) 
  {
		m = 0.0;
		c = y[0];
		return;
	}
	
	for (int i = 0; i < n; i++) 
  {
		meanX += ( x[i] / n );
		meanY += ( y[i] / n );
	}
	
	//printf("%s << means = (%f, %f)\n", __FUNCTION__, meanX, meanY);
	
	double s_x  = 0.0;
  double s_xy = 0.0; 
	
	for (int i = 0; i < n; i++) 
  {
		s_x += pow( x[i]-meanX, 2.0 );
		//s_y += pow( y[i]-meanY, 2.0 );
		s_xy += ( x[i]-meanX ) * ( y[i]-meanY );
	}
	
	//printf("%s << s's = (%f, %f)\n", __FUNCTION__, s_x, s_xy);
	
	m = s_xy / s_x;
	c = meanY - m*meanX;
	
	//printf("%s << eq = (%f, %f)\n", __FUNCTION__, m, c);
}


void ConvertUcharToBinaryIntArray( const unsigned char src, int* dst ) 
{
  unsigned char remainder = src;
  
  for ( int i = 0; i < BITS_PER_BYTE; i++ ) 
  {
    if ((int) remainder >= (int) pow(2, BITS_PER_BYTE-i-1)) 
    {
      dst[i] = 1;
      remainder -= (int) pow(2, BITS_PER_BYTE-i-1);
    } 
    else 
    {
      dst[i] = 0;
    }
  }
}


int CountElementsInFolder( const char* folderName, vector<string>& elementNames, const E_ElementType elementType ) 
{
#if _IS_WINDOWS_
	printf("%s << ERROR! THIS FUNCTION HAS NOT BEEN IMPLEMENTED IN WINDOWS!\n", __FUNCTION__);
	return ERROR_CODE;
#else

  int typeCode = 0;
  switch ( elementType )
  {
    case ELEMENT_FILE:
      typeCode = DT_REG;
      break;
    case ELEMENT_FOLDER:
      typeCode = DT_DIR;
      break;
    default:
      break;
  }

  char *folder = (char*) malloc(strlen(folderName) + 32);
  sprintf( folder, "%s", folderName );
  
  //printf("%s << folder = %s\n", __FUNCTION__, folder);
  DIR *dirp;
  dirp = opendir(folder);

  //printf("%s << folder opened.\n", __FUNCTION__);

  int elementCount = 0;
  struct dirent * entry;
  while ((entry = readdir(dirp)) != NULL) 
  {
    //printf("%s << entry is: %s\n", __FUNCTION__, entry->d_name);
    if ( (entry->d_type == typeCode) && (entry->d_name[0] != '.') )
    { 
       elementNames.push_back(string(entry->d_name));
       printf("%s << elementName[%d] = %s\n", __FUNCTION__, elementCount, elementNames.at(elementCount).c_str());
       elementCount++;
    }
  }
  closedir(dirp);

  return elementCount;
#endif
}


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__) 
int gettimeofday(struct timeval *tv, struct timezone *tz)
{
  FILETIME ft;
  unsigned __int64 tmpres = 0;
  static int tzflag;
 
  if (NULL != tv)
  {
    GetSystemTimeAsFileTime(&ft);
 
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;
 
    /*converting file time to unix epoch*/
    tmpres /= 10;  /*convert into microseconds*/
	tmpres -= DELTA_EPOCH_IN_MICROSECS; 
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);
  }
 
  if (NULL != tz)
  {
    if (!tzflag)
    {
      _tzset();
      tzflag++;
    }
    tz->tz_minuteswest = _timezone / 60;
    tz->tz_dsttime = _daylight;
  }
 
  return 0;
}
#endif


double ElapsedTimeMilliseconds( struct timeval& timer, const bool reset ) 
{
	struct timeval new_time;

	long seconds, useconds;

	gettimeofday(&new_time, NULL);

  seconds  = new_time.tv_sec  - timer.tv_sec;
  useconds = new_time.tv_usec - timer.tv_usec;

	double retVal = ((double) seconds) * 1000.0 + ((double) useconds) * 0.001;

	if (reset) 
  {
		timer = new_time;
	}

  return retVal;
}


void SelectRandomSubset( const vector<unsigned int>& src, vector<unsigned int>& dst, unsigned int maxVal ) 
{
	dst.clear();
	dst.insert( dst.end(), src.begin(), src.end() );

	if (dst.size() <= maxVal) 
  {
		return;
	}

	while ( dst.size() > maxVal ) 
  {
		dst.erase( dst.begin() + (rand() % dst.size()) );
	}
}


double AsymmetricGaussianValue( const double score, const double mean, const double loVariance, const double hiVariance ) 
{
	double sigma    = 1.0;
  double stddevs  = 3.0;

	if (score == mean) return 1.00;
  
  double upThresh = mean + stddevs * hiVariance;
  double loThresh = mean - stddevs * loVariance;
  
	if ( ( score > upThresh ) || ( score < loThresh ) ) return 0.00;
	
	(score > mean) ? sigma = abs(hiVariance - mean) : sigma = abs(loVariance - mean);

	double zScore = (score - mean) / sigma;

	return exp( -pow( zScore, 2.0 ) / 2.0);
}


void AddUniqueValuesToVector( vector<unsigned int>& dst, const vector<unsigned int>& src ) 
{
	for ( int i = 0; i < src.size(); i++ ) 
  {
		bool alreadyAdded = false;

		for ( int j = 0; j < dst.size(); j++ ) 
    {
			if ( dst.at(j) == src.at(i) ) 
      {
				alreadyAdded = true;
			}
		}

		if ( ! alreadyAdded ) 
    {
			dst.push_back( src.at(i) );
		}
	}
}


long Factorial( int x )
{
  long result = 1;
  
  for ( int i = 1; i <= x; ++i ) 
  {
      int remainingCapacity = std::numeric_limits<long>::max() / result;
      
      if ( remainingCapacity < x )
      {
        printf( "%s:%s : ERROR! Factorial of input [ %d ] is too large.\n", __FILE__, __FUNCTION__, x );
        return 0;
      }
      result *= x;
  }
  return result;
}


void UpdateCombinatorialArray( vector<unsigned int>& currentArray, const int k, const int n ) 
{
  bool valid = true;

  // If no current indices, initialize with (0, 1, 2 etc)
  if ( currentArray.size() == 0 )
  {
    for ( int i = 0; i < k; i++ )
    {
      currentArray.push_back( i );
    }
  }
  else
  {
    int elementsFromEnd = 0;
    while ( valid && (elementsFromEnd < k) )
    {
      int pos = currentArray.size()-elementsFromEnd-1;
      
      // If current index is about to go over its maximum...
      if ( int( currentArray.at( pos ) ) > ( n-2-elementsFromEnd ) )
      {
          //printf("%s << digit #(%d) is valid; less than %d\n", __FUNCTION__, currentIndices.size()-i-1, n-2-i);
          elementsFromEnd++;
      }
      else // Otherwise, just increment it, fill in trailing digits and exit while loop
      {
        currentArray.at( pos ) = currentArray.at( pos ) + 1;
        for ( int j = 0; j < elementsFromEnd; j++ )
        {
          currentArray.at( pos + j + 1 ) = currentArray.at( pos + j ) + 1;
        }
        valid = false;
      }
    }
  }
}


void CalculateMeanAndStdDev( const vector<double>& v, double& mean, double& stdev ) 
{
	double sum = 0.0;
	
	for ( unsigned int iii = 0; iii < v.size(); iii++ ) 
  {
		sum += v.at(iii);
	}
	
	mean = sum / v.size();

	double sqSum = 0.0; 
	
	for (unsigned int iii = 0; iii < v.size(); iii++) 
  {
		sqSum += pow( v.at(iii) - mean, 2.0 );
	}
	
	stdev = std::sqrt( sqSum / v.size() );
}


double EquivalentProbabilityScore( const double* values, const int n, const double prop )
{
  vector<double> scoreVector;

  for ( int j = 0; j < n; j++ )
  {
    scoreVector.push_back( values[j] );
  }
  
  double minScore = std::numeric_limits<double>::max();

  // Pop minimum values off vector until you've reached sufficient depth for the probability
#if _IS_WINDOWS_
  while ( scoreVector.size() >= (unsigned int)( max(int( prop*n ), 1) ) )
#else
  while ( scoreVector.size() >= (unsigned int)( std::max(int( prop*n ), 1) ) )
#endif
  {
    minScore = std::numeric_limits<double>::max();
    size_t minIndex = 0;

    for ( size_t j = 0; j < scoreVector.size(); j++ )
    {
      if ( scoreVector.at(j) < minScore )
      {
        minScore = scoreVector.at(j);
        minIndex = j;
      }
    }

    scoreVector.erase( scoreVector.begin() + minIndex );
  }

  return minScore;
}


void ConvertRawByteToBinaryCharArray( const void* src, char* dst ) 
{
	unsigned char* num = (unsigned char*) src;
	
	unsigned int factor = 128;
	
	for ( unsigned int i = 0; i < BITS_PER_BYTE; i++ ) 
  {
		if ( *num > factor ) 
    {
			dst[i] = '1';
		} 
    else 
    {
			dst[i] = '0';
		}
		factor /= 2;
	}
	dst[ BITS_PER_BYTE ] = '\0';
}


double TimeDifference( const ros::Time time1, const ros::Time time2) 
{
	double retVal = ((double) time1.sec) - ((double) time2.sec);
	retVal += 1e-9 * (((double) time1.nsec) - ((double) time2.nsec));
	return retVal;
}


ros::Time TimeMidpoint( const ros::Time time1, const ros::Time time2 ) 
{
  long int nsec = (time1.nsec/2) + (time2.nsec/2);
	
  long int sec  = 0;
	((time1.sec % 2) > 0) ? sec += (time1.sec-1)/2 : sec += time1.sec/2;
	((time2.sec % 2) > 0) ? sec += (time2.sec-1)/2 : sec += time2.sec/2;
	
	if ( ( (time1.sec % 2) > 0 ) && ( (time2.sec % 2) > 0 ) ) 
  {
		sec += 1;
	} 
  else if (((time1.sec % 2) > 0) || ((time2.sec % 2) > 0)) 
  {
		nsec += 500000000;
	}
			
	ros::Time avTime;
	avTime.sec = sec;
	avTime.nsec = nsec;
	return avTime;
}


void CleanAndSubstitutePath( std::string& path )
{
#ifndef _WIN32
  for ( int i = 0; i < path.length(); i++ ) 
  {
    if ( path.at(i) == '\\' ) path.at(i) = '/';
  }
#endif

  if ( ( path.length() > 0 ) && ( path[0] == '~') )
  {
#ifdef _WIN32
    path.replace( 0, 1, std::getenv("USERPROFILE") );
#else
    path.replace( 0, 1, _USERPROFILE_ );
#endif
  }

  if ( ( path.length() > 0 ) && ( path[0] == '.' ) )
  {
    path.replace( 0, 1, _THERMALVIS_SOURCE_ );
  }
}

