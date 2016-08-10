/*! \file   tools.hpp
 *  \brief  Declarations for generic tools not depending on libraries such as OpenCV, PCL and ROS.
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

#define BITS_PER_BYTE       8
#define ERROR_CODE          -1

enum E_MessageType
{
  MESSAGE_NORMAL      = 0,
  MESSAGE_WARNING     = 1,
  MESSAGE_ERROR       = 2,
  NO_OF_MESSAGE_TYPES // Must be the final value
};

enum E_ElementType
{
  ELEMENT_FILE        = 0,
  ELEMENT_FOLDER      = 1,
  NO_OF_ELEMENT_TYPES // Must be the final value
};

using namespace std;

long        Factorial                         ( int x );
float       ReciprocalSquareRoot              ( float x );

double      AsymmetricGaussianValue           ( const double score, const double mean, const double loVar, const double hiVar );
void        CalculateMeanAndStdDev            ( const std::vector<double>& v, double& mean, double& stdev );
void        SelectRandomSubset                ( const std::vector<unsigned int>& src, std::vector<unsigned int>& dst, unsigned int maxVal );

void        InitializeRandomNumberGeneration  ();

void        ConvertUcharToBinaryIntArray      ( const unsigned char src, int* dst );
void        ConvertRawByteToBinaryCharArray   ( const void* src, char* dst );

void        CleanAndSubstitutePath            ( std::string& path );

int         CountElementsInFolder             ( const char* folderName, std::vector<std::string>& elementNames, const E_ElementType elementType );

void        AddUniqueValuesToVector           ( std::vector<unsigned int>& dst, const std::vector<unsigned int>& src );

/// \brief  From a set of n (x,y) pairs, calculates the gradient 'n' and y-intercept 'c' 
void        FindLinearModel                   ( const double* x, const double* y, const int n, double &m, double &c );

/// \brief  Generates the next ordered combination of size 'r' from 'n' ordered integer values
void        UpdateCombinatorialArray          ( std::vector<unsigned int>& currentArray, const int k, const int n );

/// \brief  Selects a score from the array 'sortedValues' of 'n' length which is minimally better than thespecified proportion of all scores
double      EquivalentProbabilityScore        ( const double* values, const int n, const double prop );

double      ElapsedTimeMilliseconds           ( struct timeval& timer, const bool reset = true );
double      TimeDifference                    ( const ros::Time time1, const ros::Time time2 );
ros::Time   TimeMidpoint                      ( const ros::Time time1, const ros::Time time2 );

#endif
