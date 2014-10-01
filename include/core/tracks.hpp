/*! \file	tracks.hpp
 *  \brief	Declarations for managing local feature tracks across video sequences.
*/

#ifndef _THERMALVIS_TRACKS_H_
#define _THERMALVIS_TRACKS_H_

#include "general_resources.hpp"
#include "camera.hpp"
#include "tools.hpp"
#include "improc.hpp"
#include "core/features.hpp"

#define DEFAULT_MAX_VELOCITY 			10000.0
#define MAX_TIME_DIFF_FOR_PREDICTION	1.0

/// \brief		Stores the spatial and temporal location of a single feature occurence in the video stream 
struct indexedFeature {
	unsigned int imageIndex;
	cv::Point2f featureCoord;
	
	indexedFeature(const int& i, const cv::Point2f& f) {
		imageIndex = i;
		featureCoord = f;
	}
	
	
};

/// \brief		Stores all spatial and temporal locations of a single feature in a video stream, along with its 3D estimate
struct featureTrack {
	
	long int trackIndex;
	vector<indexedFeature> locations;
	bool isTriangulated;
	unsigned int firstOccurence;
	double velocityWeighting;
	double velocity_x, velocity_y;
	
	featureTrack() : isTriangulated(false) { }
	
	/// \brief		Only sets 3d location if it hasn't already been set
	void set3dLoc(const cv::Point3d& loc);
	
	/// \brief		Sets 3d location whether it has been set or not
	void reset3dLoc(const cv::Point3d& loc);
	cv::Point3d get3dLoc() { return xyzEstimate; }
	
	void addFeature(indexedFeature& feat);
	
	cv::Point2f getCoord(unsigned int cam_idx);
	bool occursInSequence(const vector<unsigned int>& indices);
	
	protected:
		cv::Point3d xyzEstimate;
		
	
};

/// \brief		Clears tracks that are in danger of overflowed indices
void clearDangerFeatures(vector<featureTrack>& featureTrackVector, long int index);

/// \brief		Find position within track vector for a specific track index
int findTrackPosition(const vector<featureTrack>& featureTrackVector, long int index);

unsigned int getActiveTrackCount(const vector<featureTrack>& featureTrackVector, unsigned int previousIndex, unsigned int latestIndex);

void removeObsoleteElements(vector<featureTrack>& featureTrackVector, const vector<unsigned int>& activeFrameIndices);

// \brief		I think this will give you all of the 2D projections for points that were successfully tracked from idx1 to idx2
void getPointsFromTracks(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx1, int idx2);

/// \brief		Search structure for tracks, then predict their future locations
void assignEstimatesBasedOnVelocities(const vector<featureTrack>& featureTrackVector, const vector<cv::Point2f>& startingPoints, vector<cv::Point2f>& finishingPoints, unsigned int idx, double time1, double time2);

/// \brief		Calculate and assign speeds for features tracked between the provided frames
double updateFeatureSpeeds(vector<featureTrack>& featureTrackVector, unsigned int idx1, double time1, unsigned int idx2, double time2, double maxVelocity = DEFAULT_MAX_VELOCITY);

/// \brief		Calculate but don't assign speeds for features tracked between the provided frames
double obtainFeatureSpeeds(const vector<featureTrack>& featureTrackVector, unsigned int idx1, double time1, unsigned int idx2, double time2);

/// \brief		Add a whole bunch of candidates to the vector, unless they violate separation constraints
void addMatchesToVector(vector<featureTrack>& featureTrackVector, unsigned int index1, vector<cv::Point2f>& points1, unsigned int index2, vector<cv::Point2f>& points2, long int &starting_track_index, double minSeparation, bool debug = false);

/// \brief		Add a single new matched (tracked) projection pair or feature pair to the vector
int addMatchToVector(vector<featureTrack>& featureTrackVector, unsigned int index1, const cv::Point2f& point1, unsigned int index2, const cv::Point2f& point2, long int &starting_track_index, const cv::Point2f& velocity = cv::Point2f(0.0,0.0), bool debug = false);


/// \brief		Add a whole bunch of (single projection) candidates to the vector, unless they violate separation constraints
void addProjectionsToVector(vector<featureTrack>& featureTrackVector, unsigned int index, vector<cv::Point2f>& points, long int &starting_track_index, double minSeparation = 0.0);

/// \brief		Add a single new (untracked) projection or feature to the vector
void addProjectionToVector(vector<featureTrack>& featureTrackVector, unsigned int index, const cv::Point2f& point, long int &starting_track_index);


void drawFeatureTracks(cv::Mat& src, cv::Mat& dst, vector<featureTrack>& tracks, const cv::Scalar& tColor, const cv::Scalar& kColor, unsigned int index, unsigned int history = 10);

void redistortTracks(const vector<featureTrack>& src, vector<featureTrack>& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, unsigned int latest, const cv::Mat& newCamMat=cv::Mat::eye(3,3,CV_64FC1), unsigned int history = 5);

bool createTrackMatrix(const vector<featureTrack>& src, cv::Mat& dst, int latest = -1);

/// \brief		Find only the features from the first image that have NOT been tracked successfully in the second
void assignHistoricalPoints(const vector<featureTrack>& src, unsigned int idx_1, unsigned int idx_2, vector<cv::Point2f>& dst);

void reduceVectorsToTrackedPoints(const vector<cv::Point2f>& points1, vector<cv::Point2f>& trackedPoints1, const vector<cv::Point2f>& points2, vector<cv::Point2f>& trackedPoints2, vector<uchar>& statusVec);

double getFeatureMotion(vector<featureTrack>& tracks, vector<unsigned int>& indices, unsigned int idx_1, unsigned int idx_2);

void findRelevantIndices(vector<featureTrack>& tracks, vector<unsigned int>& triangulated, vector<unsigned int>& untriangulated, unsigned int last_index, unsigned int new_index);

#endif
