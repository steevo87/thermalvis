/*! \file	  tools.hpp
 *  \brief	Declarations for generic tools not depending on libraries such as OpenCV, PCL and ROS.
*/

#ifndef _THERMALVIS_TOOLS_H_
#define _THERMALVIS_TOOLS_H_

// OS-Specific
//#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#if _IS_WINDOWS_ 
#include "core/windows.hpp"
#else
#include "core/linux.hpp"
#endif

// Generic Headers
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#ifdef _BUILD_FOR_ROS_
#include <ros/ros.h>
#else
#include "core/ros_substitution.hpp"
#endif

enum E_MessageType
{
  MESSAGE_NORMAL      = 0,
  MESSAGE_WARNING     = 1,
  MESSAGE_ERROR       = 2,
  NO_OF_MESSAGE_TYPES // Must be the last value
};

using namespace std;

long        Factorial                         ( int num );
float       ReciprocalSquareRoot              ( float x );

double      AsymmetricGaussianValue           ( double score, double mean, double loVar, double hiVar );
void        CalculateMeanAndStdDev            ( const std::vector<double>& v, double& mean, double& stdev );
double      PerpDistBetweenParallelLines      ( double *line1, double *line2 );
void        SelectRandomSubset                ( std::vector<unsigned int>& src, std::vector<unsigned int>& dst, unsigned int max_val );

void        InitializeRandomNumberGeneration  ();

void        ConvertUcharToBinaryIntArray      ( unsigned char val, int* binaryArray );
void        ConvertRawByteToBinaryCharArray   ( void* src, char* dst );

void        CleanAndSubstitutePath            ( std::string& path );

int         CountElementsInFolder             ( const char* folderName, std::vector<std::string>& elementNames, int elementType );

void        GenerateCombinatorialArray        ( std::vector<unsigned int>& currentIndices, int r, int n);
void        AddUniqueValuesToVector           ( std::vector<unsigned int>& dst, std::vector<unsigned int>& src);
void        FindLinearModel                   ( double* x, double* y, int termsToConsider, double &m, double &c );

/// \brief  Selects the score which is minimally better than a specified proportion of all scores
double      EquivalentProbabilityScore        ( double* values, int quantity, double prob );

double      ElapsedTimeMilliseconds           ( struct timeval& timer, bool reset = true );
double      TimeDifference                    ( ros::Time time1, ros::Time time2 );
ros::Time   TimeMidpoint                      ( ros::Time time1, ros::Time time2 );

#endif
