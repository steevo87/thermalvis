/*! \file	radiometric.hpp
 *  \brief	Declarations for radiometric image processing.
*/

#ifndef _THERMALVIS_RADIOMETRIC_H_
#define _THERMALVIS_RADIOMETRIC_H_

#include "core/improc.hpp"

/// \brief		For configuring and applying radiometric mapping to thermal images
class rScheme {
protected:
	
	/// \brief		Minimum sensor (thermistor) temperature that the model can work for 
	float minTemp;
	
	/// \brief		Maximum sensor (thermistor) temperature that the model can work for 
	float maxTemp;
	
	/// \brief		Minimum image graylevel that the model can work for 
	float minGraylevel;
	
	/// \brief		Maximum image graylevel that the model can work for 
	float maxGraylevel;

	/// \brief		Mapping matrix
	cv::Mat mappingMatrix;

public:
	/// \brief 		Constructor.
	rScheme();

	/// \brief 		Constructor.
	/// \param 		mM		Mapping matrix
	/// \param 		minT	Minimum temperature
	/// \param 		maxT	Maximum temperature
	/// \param 		minG	Minimum graylevel
	/// \param 		maxG	Maximum graylevel
	rScheme(const cv::Mat &mM, const double &minT, const double &maxT, const double &minG, const double &maxG);

	/// \brief 		Destructor.
	~rScheme();

	/// \brief		Assigns new data to the model
	/// \param 		mM		Mapping matrix
	/// \param 		minT	Minimum temperature
	/// \param 		maxT	Maximum temperature
	/// \param 		minG	Minimum graylevel
	/// \param 		maxG	Maximum graylevel
	void update(const cv::Mat &mM, const double &minT, const double &maxT, const double &minG, const double &maxG);

	/// \brief 		Applies the radiometric mapping to generate a new radiometrically consistent image
	/// \param 		src				Source (raw) image
	/// \param 		dst				Target (corrected) image
	/// \param 		thermistorTemp	Temperature of on-board thermistor
	/// \param 		interpolateVal	Flag to interpolate temp estimate using lookup table
	
	void apply(const cv::Mat& src, cv::Mat& dst, float thermistorTemp = 35.0, float interpolateVal = false);

};

#endif
