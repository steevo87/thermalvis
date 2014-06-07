/*! \file	tools.hpp
 *  \brief	Declarations for generic tools not depending on libraries such as OpenCV, PCL and ROS.
*/

#ifndef _THERMALVIS_TOOLS_H_
#define _THERMALVIS_TOOLS_H_

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	#define NOMINMAX 1
	#include <windows.h>
#else
	#include <unistd.h>
	#include <getopt.h>
	#include <dirent.h>
	#include <sys/time.h>
	#include <sys/mman.h>
	#include <sys/ioctl.h>
	#include <termios.h>
	#include <string.h>
	#include <cstdlib>
	#include <cmath>
#endif

#include <iostream>
#include <fstream>
#include <vector>

#define USE_CLAHE 0

using namespace std;

typedef std::pair<unsigned int,unsigned int> mypair;
bool comparator ( const mypair& l, const mypair& r);

/// \brief      Counts and returns a list of all specified elements within a folder
int countElementsInFolder(const char* folderName, vector<string>& elementNames, int elementType);

void convertUcharToBinary(unsigned char val, int* binaryArray);







/// \brief      Calculate the mean and standard deviation of a vector of doubles
void calcParameters(const vector<double>& v, double& mean, double& stdev);

/// \brief      Calculates perpendicular distance between two "parallel" lines
double calcLinePerpDistance(double *line1, double *line2);

void findLinearModel(double* x, double* y, int termsToConsider, double &m, double &c);

/// \brief      Calculates time elapsed since the last time the timer was reset
double timeElapsedMS(struct timeval& timer, bool reset = true);



void addUniqueToVector(vector<unsigned int>& dst, vector<unsigned int>& src);

double asymmetricGaussianValue(double score, double mean, double loVar, double hiVar);

void randomSelection(vector<unsigned int>& src, vector<unsigned int>& dst, unsigned int max);





/// \brief		Calculates Factorial of an integer
long long int factorial(int num);



/// \brief      Gets next possible combination for an exhaustive combinatorial search
void getNextCombo(vector<unsigned int>& currentIndices, int r, int n);

/// \brief      Selects the score which is minimally better than a specified proportion of all scores
double findEquivalentProbabilityScore(double* values, int quantity, double prob);



/// \brief      Converts a raw byte as an 8-bit character array of 1s and 0s
void convert_byte_to_binary_string(void* src, char* dst);



/// \brief		http://stackoverflow.com/questions/485525/round-for-float-in-c
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	double round(double d);
#endif

#endif
