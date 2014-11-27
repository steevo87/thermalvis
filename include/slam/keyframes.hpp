/*! \file	keyframes.hpp
 *  \brief	Declarations for calculations and management relating to keyframes (for loop closure).
*/

#ifndef THERMALVIS_KEYFRAMES_H
#define THERMALVIS_KEYFRAMES_H

#include "core/general_resources.hpp"
#include "core/features.hpp"

#define KF_CONNECTION_WEAK			0
#define KF_CONNECTION_GEOMETRIC		1
#define KF_CONNECTION_HOMOGRAPHIC	2

#define KF_TYPE_FIRST				0
#define KF_TYPE_LAST				1
#define KF_TYPE_GEOMETRIC			2
#define	KF_TYPE_HOMOGRAPHIC			3
#define	KF_TYPE_EXHAUSTED			4
#define	KF_TYPE_WEAK				5

#define	MAXIMUM_FEATURES_PER_FRAME	200

#define	MATCHING_SIZE_CONSTRAINT	0.2
#define	MATCHING_DIST_CONSTRAINT	100

/// \brief		Contains feature information corresponding to a keyframe which may eventually be used for loop closure
struct keyframe {
	
	unsigned int idx; /**< Index of the keyframe */
	cv::Mat im; /**< Original image for the keyframe */
	std::vector<cv::KeyPoint> keypoints; /**< 2D features defining the keyframe */
	cv::Mat descriptors; /**< Matrix of descriptors associated with keypoints */
	bool poseDetermined; /**< Whether the camera pose associated with this keyframe has been determined or not */
	
	/// \brief      Uses an established keypoint detector to detect keypoints in the keyframe
	void detectKeypoints(cv::Ptr<cv::FeatureDetector>& detector);
	
	/// \brief      Uses an established descriptor extractor to extract descriptors in the keyframe
	void extractDescriptors(cv::Ptr<cv::DescriptorExtractor>& extractor);
};

/// \brief		Describes a link between two keyframes in terms of matched features
struct connection {
	
	unsigned int idx1; /**< Index of the first keyframe */
	unsigned int idx2; /**< Index of the second keyframe */
	cv::Mat matchingMatrix; /**< Matrix of matching scores between descriptors */
	int type; /**< Type of connection */
	double confidence; /**< Confidence of validity of this connection */
	cv::Mat relation; /**< 3D transformation between the two keyframes */
	bool processed; /**< Whether this connection has been processed or not */
	std::vector<std::vector<cv::DMatch> > matches1to2; /**< Vector of vector of matches between first and second keyframes */
	
};

/// \brief		Stores information linking keyframes within a SLAM sequence
struct keyframeStore {
	
	std::vector<keyframe> keyframes; /**< Vector storing all keyframes */
	std::vector<connection> connections; /**< Vector storing all connections found between keyframes */
	
	// Should you also store matches?
	
	/// \brief      Constructor for Keyframe Store
	keyframeStore();
	
	/// \brief      Finds the best pair of keyframes to initialize structure with
	bool getBestPair(int& idx1, int& idx2);
	
	/// \brief      Adds a keyframe to the store
    void addKeyframe(int idx, cv::Mat& image);
	
	/// \brief      Adds a connection between keyframes to the store
	void addConnection(int idx1, int idx2, int type = KF_CONNECTION_WEAK, cv::Mat rel = cv::Mat());
	
	/// \brief      Returns indices of connections that are strongly linked to this keyframe
	void findStrongConnections(int idx, std::vector<unsigned int>& cIndices);
	
	/// \brief      Finds all matches between weak keyframes
	void findMatches();

	/// \brief		Clear all data (ready to restart)
	void clearAll();
	
};

/// \brief		Functionality somewhat uncertain but seems to find best match in a matching matrix with some constraints..
bool getValidLocalMaxima(cv::Mat& scores, unsigned int last_iii, unsigned int last_jjj, unsigned int& opt_iii, unsigned int& opt_jjj);

#endif // THERMALVIS_KEYFRAMES_H
