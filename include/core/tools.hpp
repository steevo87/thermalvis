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

int CountElementsInFolder( const char* folderName, std::vector<std::string>& elementNames, int elementType );

/// \brief      Converts an unsigned char into an array of 0s and 1s
void convertUcharToBinary(unsigned char val, int* binaryArray);

/// \brief		  Care of [ J.M.P. van Waveren ]
float ReciprocalSqrt( float x );

/// \brief      Calls the relevant system function to ensure number pseudo-randomness
void initializeRandomNums();

/// \brief      Calculate the mean and standard deviation of a vector of doubles
void calcParameters(const std::vector<double>& v, double& mean, double& stdev);

/// \brief      Calculates perpendicular distance between two "parallel" lines
double calcLinePerpDistance(double *line1, double *line2);

void findLinearModel(double* x, double* y, int termsToConsider, double &m, double &c);

/// \brief      Calculates time elapsed since the last time the timer was reset
double timeElapsedMS(struct timeval& timer, bool reset = true);

void addUniqueToVector( std::vector<unsigned int>& dst, std::vector<unsigned int>& src);

double asymmetricGaussianValue(double score, double mean, double loVar, double hiVar);

void randomSelection( std::vector<unsigned int>& src, std::vector<unsigned int>& dst, unsigned int max_val);



/// \brief		Calculates Factorial of an integer
long long int factorial(int num);

/// \brief      Gets next possible combination for an exhaustive combinatorial search
void getNextCombo( std::vector<unsigned int>& currentIndices, int r, int n);

/// \brief      Selects the score which is minimally better than a specified proportion of all scores
double findEquivalentProbabilityScore(double* values, int quantity, double prob);



/// \brief      Converts a raw byte as an 8-bit character array of 1s and 0s
void convert_byte_to_binary_string(void* src, char* dst);


double timeDiff(ros::Time time1, ros::Time time2);
ros::Time findAverageTime(ros::Time time1, ros::Time time2);

void CompletePath( std::string& path );

#endif
