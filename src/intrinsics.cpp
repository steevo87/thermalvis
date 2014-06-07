/*! \file	intrinsics.cpp
 *  \brief	Definitions for intrinsic (geometric) calibration.
*/

#include "intrinsics.hpp"

double calculateERE( Size imSize,
                     std::vector<Point3f>& physicalPoints,
                     std::vector< std::vector<Point2f> >& corners,
                     const Mat& cameraMatrix,
                     const Mat& distCoeffs,
                     bool removeSpatialBias, 
                     bool generateFigure, 
                     bool useUndistortedLocations)
{
	
	bool debug = false;
	
	
	if (debug) { printf("%s << physicalPoints.size() = (%d)\n", __FUNCTION__, physicalPoints.size()); }
	if (debug) { printf("%s << corners.size() = (%d)\n", __FUNCTION__, corners.size()); }
	
    int total_points = int(corners.size() * corners.at(0).size());
    
    Mat newCamMat;
    Mat identity = Mat::eye(3,3,CV_64FC1);
    
    if (useUndistortedLocations) {
		newCamMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imSize, 0.0);
		newCamMat = newCamMat.inv();
	}
	
    double *errValues;
    
    errValues = new double[total_points];
    
    Mat projectionCountGrid, cumulativeErrorGrid;
    
    if (debug) { printf("%s << REACHED (%d)\n", __FUNCTION__, 0); }
    
    if (1) { // (removeSpatialBias) {
		// Create two grid matrices, one for counts and one for cumulative errors
		
		double ratio = double(imSize.width) / double(imSize.height);
		
		//printf("%s << image size = (%d, %d)\n", __FUNCTION__, imSize.height, imSize.width);
		
		int desiredBins = min(MAX_BINS, max(MIN_BINS,total_points));
		
		double numRows, numCols;
		
		numRows = pow(desiredBins / ratio , 0.5);
		numCols = ratio * numRows;
		
		// Divide major axis into 16
		
		projectionCountGrid = Mat::zeros(int(ceil(numRows)), int(ceil(numCols)), CV_16UC1);
		cumulativeErrorGrid = Mat::zeros(int(ceil(numRows)), int(ceil(numCols)), CV_32FC1);
		
		//printf("%s << grid size = (%d, %d) [%d]\n", __FUNCTION__, projectionCountGrid.rows, projectionCountGrid.cols, total_points);
		
	}

    if (debug) { printf("%s << REACHED (%d)\n", __FUNCTION__, 1); }
    


    Mat fsRvec, fsTvec;
    std::vector<Point2f> cornerSet;
    cornerSet.resize(physicalPoints.size());

    double err = 0.0, xSum = 0, ySum = 0, tSum = 0, xMean = 0, yMean = 0, tMean = 0, xDev = 0, yDev = 0, tDev = 0;

    Point2f imLoc, imageDec, predictedDec;

	//printf("%s << A\n",__FUNCTION__);
    
    for (unsigned int i = 0; i < corners.size(); i++) {

		if (debug) { printf("%s << Set (%d) Solving PnP...\n", __FUNCTION__, i); }
		
		if (debug) { printf("%s << corners.at(%d).size() = (%d)\n", __FUNCTION__, i, corners.at(i).size()); }
		
        // Estimate pose of board
        solvePnP(Mat(physicalPoints), Mat(corners.at(i)), cameraMatrix, distCoeffs, fsRvec, fsTvec, false);

		//cout << "fsRvec = " << endl << fsRvec << endl;
		//cout << "fsTvec = " << endl << fsTvec << endl;

		if (debug) { printf("%s << Projecting points...\n", __FUNCTION__); }

        // Reproject the object points using estimated rvec/tvec
        projectPoints(Mat(physicalPoints), fsRvec, fsTvec, cameraMatrix, distCoeffs, cornerSet);
        // do we want distortion vector or not? (noDistVector) it would make undistorted comparison better..

        //image.copyTo(debugImage);
        
        if (debug) { printf("%s << About to start loop through points...\n",__FUNCTION__); }
        
        vector<Point2f> undistortedPoints;
        
        if (useUndistortedLocations) {
			undistortPoints(corners.at(i), undistortedPoints, cameraMatrix, distCoeffs, identity, newCamMat);
		}

        // Find the mean and standard deviations for differences between results (conventional)
        for (unsigned int j = 0; j < cornerSet.size(); j++) {
			
			if (debug) { printf("%s << Point loop (%d)\n", __FUNCTION__, j); }

            imageDec = Point2f(corners.at(i).at(j).x, corners.at(i).at(j).y);
            predictedDec = Point2f(cornerSet.at(j).x, cornerSet.at(j).y);
            
            double ptError = pow(pow(imageDec.x-predictedDec.x, 2)+pow(imageDec.y-predictedDec.y, 2), 0.5);

			

			//printf("%s << Error between (%f, %f) and (%f, %f) is (%f)\n", __FUNCTION__, imageDec.x, imageDec.y, predictedDec.x, predictedDec.y, ptError);

			

			//if (ptError > 10.0) {
			//	cin.get();
			//}

            if (errValues)
            {
				
				//printf("%s << Updating error (%d / %d) with (%f)\n", __FUNCTION__, i*cornerSet.size()+j, total_points, ptError);
				
                errValues[i*cornerSet.size()+j] = ptError;
                
                //printf("%s << errValues[%d] = %f\n", __FUNCTION__, i*cornerSet.size()+j, errValues[i*cornerSet.size()+j]);
            }
            
            
            
            if (1) { // (removeSpatialBias) {
				
				if (debug) { printf("%s << Removing spatial bias...\n",__FUNCTION__); }
				
				Point2f testPoint;
				
				
				
				if (useUndistortedLocations) {
					testPoint = Point2f(undistortedPoints.at(j).x, undistortedPoints.at(j).y);
					
					//printf("%s << Point moved from (%f, %f) to (%f, %f)\n", __FUNCTION__, imageDec.x, imageDec.y, undistortedPoints.at(j).x, undistortedPoints.at(j).y);
					
				} else {
					testPoint = imageDec;
				}
				
				
				if ((testPoint.x > 0.0) && (testPoint.x < imSize.width) && (testPoint.y > 0.0) && (testPoint.y < imSize.height)) {
					
					//printf("%s << Test point (%f, %f) within bounds (%f, %f)\n",__FUNCTION__, testPoint.x, testPoint.y, imSize.width, imSize.height);
					
					// Determine the appropriate bin for the original image location
					
					int gridRow = int(round( ( testPoint.y / float(imSize.height) ) * float(projectionCountGrid.rows) - 0.5 ));
					int gridCol = int(round( ( testPoint.x / float(imSize.width) ) * float(projectionCountGrid.cols) - 0.5 )); 
					
					//printf("%s << [x,y] point(%f, %f) binned to (%d, %d)\n", __FUNCTION__, imageDec.x, imageDec.y, gridCol, gridRow);
					
					projectionCountGrid.at<unsigned short>(gridRow, gridCol) += 1;
					cumulativeErrorGrid.at<float>(gridRow, gridCol) += float(ptError);
					
					//printf("%s << Updating grid (%d, %d) of (%d, %d)\n", __FUNCTION__, gridRow, gridCol, projectionCountGrid.rows, projectionCountGrid.cols);
					
					// Update the bins
					
				} else {
					//printf("%s << Test point (%f, %f) out of bounds (%f, %f)\n",__FUNCTION__, testPoint.x, testPoint.y, imSize.width, imSize.height);
				}
				
			}

			
			if (debug) { printf("%s << A6\n",__FUNCTION__); }

            
        }
        
        

		if (debug) { printf("%s << A8\n",__FUNCTION__); }

    }
    
    
    
    if (debug) { printf("%s << B\n",__FUNCTION__); }
    
    if (1) { // (removeSpatialBias) {
		
		//printf("%s << B1\n",__FUNCTION__);
		
		tSum = 0.0;
		float maxAverageError = 0.0;
		
		Mat heatMap, heatMapCol;
		
		if (generateFigure) {
			heatMap = Mat::ones(imSize, CV_8UC1);
			heatMapCol = Mat::ones(imSize, CV_8UC3);
		}
		
		//printf("%s << B2\n",__FUNCTION__);
			
		// Convert sums to averages, and determine maximum average error for occupied cells
		for (int iii = 0; iii < projectionCountGrid.rows; iii++) {
			for (int jjj = 0; jjj < projectionCountGrid.cols; jjj++) {
				
				if (projectionCountGrid.at<unsigned short>(iii, jjj) > 0) {
					cumulativeErrorGrid.at<float>(iii, jjj) /= float(projectionCountGrid.at<unsigned short>(iii, jjj));
					
					//printf("%s << cell (%d, %d) corrected error = (%f)\n", __FUNCTION__, iii, jjj, cumulativeErrorGrid.at<float>(iii, jjj));
					
					tSum += cumulativeErrorGrid.at<float>(iii, jjj);
					
					if (cumulativeErrorGrid.at<float>(iii, jjj) > maxAverageError) {
						maxAverageError = cumulativeErrorGrid.at<float>(iii, jjj);
					}
					
				} else {
					
					// Mark as zero on heatmap if no points in here, to preserve emptiness
					
					if (generateFigure) {
						unsigned int startRow = (unsigned int)(round((float(iii) / float(cumulativeErrorGrid.rows)) * float(heatMap.rows)));
						unsigned int endRow = (unsigned int)(ceil((float(iii+1) / float(cumulativeErrorGrid.rows)) * float(heatMap.rows)));
						unsigned int startCol = (unsigned int)(round((float(jjj) / float(cumulativeErrorGrid.cols)) * float(heatMap.cols)));
						unsigned int endCol = (unsigned int)(ceil((float(jjj+1) / float(cumulativeErrorGrid.cols)) * float(heatMap.cols)));
					
						for (unsigned int aaa = startRow; aaa < endRow; aaa++) {
							for (unsigned int bbb = startCol; bbb < endCol; bbb++) {
								heatMap.at<unsigned char>(aaa,bbb) = 0;
								heatMapCol.at<Vec3b>(aaa,bbb)[0] = 0;
								heatMapCol.at<Vec3b>(aaa,bbb)[1] = 0;
								heatMapCol.at<Vec3b>(aaa,bbb)[2] = 255;
							}
						}
					}
					
					
				}
				
				
			}
		}
		
		//printf("%s << B3\n",__FUNCTION__);
		
		//printf("%s << max corrected error = (%f)\n", __FUNCTION__, maxAverageError);
		
		// Assign unoccupied cells the maximum error (to penalize for not having them occupied!)
		for (int iii = 0; iii < projectionCountGrid.rows; iii++) {
			for (int jjj = 0; jjj < projectionCountGrid.cols; jjj++) {
				
				if (projectionCountGrid.at<unsigned short>(iii, jjj) == 0) {
					cumulativeErrorGrid.at<float>(iii, jjj) = maxAverageError;
					tSum += maxAverageError;
				}
				
			}
		}
		
		//printf("%s << B4\n",__FUNCTION__);
		
		err = tSum / (projectionCountGrid.rows*projectionCountGrid.cols);
		
		//printf("%s << err with de-biasing: (%f)\n", __FUNCTION__, err);
		
		if (generateFigure) {
			
			double lim = (1.0/5.0)*ceil(maxAverageError*5.0);
			
			lim = 1.0;
			
			//printf("%s << lim = (%f -> %f)\n", __FUNCTION__, maxAverageError, lim);
		
			for (int iii = 0; iii < heatMap.rows; iii++) {
				for (int jjj = 0; jjj < heatMap.cols; jjj++) {
					
					unsigned int quantRow = (unsigned int)(floor((float(iii+0) / float(heatMap.rows)) * float(cumulativeErrorGrid.rows)));
					unsigned int quantCol = (unsigned int)(floor((float(jjj+0) / float(heatMap.cols)) * float(cumulativeErrorGrid.cols)));
					
					// zero value reserved for cells which contain no points
					if (heatMap.at<unsigned char>(iii,jjj) != 0) {
						heatMap.at<unsigned char>(iii,jjj) = (unsigned char)(round(1.0 + 254.0 * (cumulativeErrorGrid.at<float>(quantRow, quantCol) / lim)));
						
						heatMapCol.at<Vec3b>(iii,jjj)[0] = heatMap.at<unsigned char>(iii,jjj);
						heatMapCol.at<Vec3b>(iii,jjj)[1] = heatMap.at<unsigned char>(iii,jjj);
						heatMapCol.at<Vec3b>(iii,jjj)[2] = heatMap.at<unsigned char>(iii,jjj);
					}
					
					
					
					
					
				}
			}
			
			//imshow("heatMap", heatMap);
			//waitKey();
			
			cScheme colourMap;
			colourMap.load_standard(300, 1);
			colourMap.falsify_image(heatMap, heatMapCol);
		
			for (int iii = 0; iii < heatMap.rows; iii++) {
				for (int jjj = 0; jjj < heatMap.cols; jjj++) {
					if (heatMap.at<unsigned char>(iii,jjj) == 0) {
						heatMapCol.at<Vec3b>(iii,jjj)[0] = 196;
						heatMapCol.at<Vec3b>(iii,jjj)[1] = 196;
						heatMapCol.at<Vec3b>(iii,jjj)[2] = 196;
					}
				}
			}
		
			char imageName[256];
			sprintf(imageName, "/home/steve/Dropbox/Steve-Peyman/jfr2013/draft/figures/intrinsics/heatmap-%3.3f.jpg", lim);
			imwrite(imageName, heatMapCol);
				
			//imshow("heatMap", heatMapCol);
			//waitKey();
			
			//printf("%s << B5\n",__FUNCTION__);
		
		}
		
		if (debug) { printf("%s << B6\n",__FUNCTION__); }
		
	}
	
	 
	
	if (!removeSpatialBias) {
		
		tSum = 0.0;
		// Calculate means
		for (unsigned int i = 0; i < corners.size(); i++)	{
			for (unsigned int j = 0; j < cornerSet.size(); j++) {

				if (errValues)
				{
					tSum += errValues[i*cornerSet.size()+j];
				}

			}
		}
		
		err = tSum / (corners.size()*cornerSet.size());
		
		//printf("%s << err without de-biasing: (%f)\n", __FUNCTION__, err);

	}
	

    if (errValues) { delete[] errValues; }
    
    return err;
    
}

void optimizeCalibrationSet(Size imSize,
                            std::vector< std::vector<Point2f> >& candidatePatterns,
                            std::vector< std::vector<Point2f> >& testPatterns,
                            std::vector<Point3f> row,
                            vector<int>& selectedTags,
                            int selection,
                            int num,
                            bool debugMode,
                            bool removeSpatialBias,
                            bool generateFigure,
                            bool useUndistortedLocations,
                            int intrinsicsFlags) 
{
	
	if (debugMode) { printf("%s << ENTERED.\n", __FUNCTION__); }
							
    selectedTags.clear();
    
    Mat distributionMap;
    
    if (debugMode) {
		distributionMap = Mat::zeros(imSize, CV_8UC1);
	}

    // If no optimization is desired
    if (selection == 0) {
        return;
    }

    // Initialize Random Number Generator
    srand ( (unsigned int)(time(NULL)) );

    // Calibration Variables
    std::vector< std::vector<Point3f> > objectPoints;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    
    cameraMatrix.at<double>(0,2) = (double(imSize.width)-1.0)/2.0;
    cameraMatrix.at<double>(1,2) = (double(imSize.height)-1.0)/2.0;
    Mat distCoeffs = cv::Mat::zeros(1, 8, CV_64F);
    std::vector<Mat> rvecs, tvecs;

    // Pointset Variables
    std::vector< std::vector<Point2f> > candidatePatternsCpy;
    candidatePatternsCpy.assign(candidatePatterns.begin(), candidatePatterns.end());     // So all corners are preserved for ERE calculation/s
    std::vector< std::vector<Point2f> > fullSetCorners;
    fullSetCorners.assign(testPatterns.begin(), testPatterns.end());     // So all corners are preserved for ERE calculation/s
    std::vector< std::vector<Point2f> > selectedFrames;
    std::vector< std::vector<Point2f> > tempFrameTester;
    std::vector< std::vector<Point2f> > newCorners;

    // Error Measurement Variables
    double err;
    Mat optimalResults(10, ABSOLUTE_MAX_FRAMES_TO_STORE, CV_64FC1);  // Raw results
    Mat rankedScores(10, ABSOLUTE_MAX_FRAMES_TO_STORE, CV_64FC1);    // Reordered results
    Mat rankedIndices(10, ABSOLUTE_MAX_FRAMES_TO_STORE, CV_8UC1);    // Indices in order - to correspond with 'rankedScores'

    // Display and Debugging Variables
    Mat distributionDisplay(distributionMap.size(), CV_8UC1);

    // Optimization Variables
    Mat binMap(30, 40, CV_32SC1);
    Mat binTemp(binMap.size(), CV_8UC1);
    binMap.setTo(0);

    // Scoring Variables
    double score, maxScore = 0.0;
    int maxIndex = 0;
    //int rankedFrames[ABSOLUTE_MAX_FRAMES_TO_STORE];
    //double rankedScoreVector[ABSOLUTE_MAX_FRAMES_TO_STORE];
    double *unrankedScores; // [MAX_FRAMES_TO_LOAD];
    //int addedIndices[MAX_FRAMES_TO_LOAD];
    vector<int> addedIndices;
    double bestScore = 0.0, topScore = 0.0;
    int bestIndex;

    vector<unsigned int> currentIndices, topIndices, bestIndices;

    // For optimum number of frames
    double prevBestScore = 9e99;
    int optimumNum = 0;
    unsigned long int possibleCombos;

    // Gaussian Variables
    Mat gaussianMat(binMap.size().height, binMap.size().width, CV_64FC1);
    createGaussianMatrix(gaussianMat, 0.3);

    // Other Variables
    int randomNum = 0;
    double testingProbability = 1.00;

    double bestErr = 9e99;

    // Random trials code
    int nTrials = 20;

    double median, p90, p99;

    double *values;
    
    //printf("%s << ENTERED. (%d)\n", __FUNCTION__, 0);

    values = new double[num*nTrials];

    /*
    for (int i = 0; i < num; i++) {
    	values[i] = new double[nTrials];
    }
    */

    num = min((int)num, (int)candidatePatterns.size());

    // SEED VARIABLES
    int nSeeds = 5;
    int nSeedTrials = 500;

    int *bestSeedSet, *currentSeedSet;

    double bestSeedScore = 9e50, currentSeedScore;

    bool alreadyUsed;

    //int newRandomNum;
    
    double radialDistribution[RADIAL_LENGTH];
    
    vector<int> tagNames;
    
    for (unsigned int iii = 0; iii < candidatePatterns.size(); iii++) {
		tagNames.push_back(iii);
	}

    // Clear radial distribution array
    for (int i = 0; i < RADIAL_LENGTH; i++)
    {
        radialDistribution[i] = 0.0;
    }
    
    //printf("%s << ENTERED. (%d)\n", __FUNCTION__, 1);

	if (debugMode) { printf("%s << About to enter switch...\n", __FUNCTION__); }
	
    switch (selection)
    {
        // ==================================================
    case SCORE_BASED_OPTIMIZATION_CODE:     //        SCORE-BASED OPTIMAL FRAME SELECTION
        // ==================================================
        // Until you've sufficiently filled the newCorners vector
        while (newCorners.size() < (unsigned int)(num))
        {
            maxScore = 0.0;
            maxIndex = 0;

            // For each corner remaining
            for (unsigned int i = 0; i < candidatePatterns.size(); i++)
            {
                score =  obtainSetScore(distributionMap, binMap, gaussianMat, candidatePatterns.at(i), radialDistribution);
                //printf("%s << Frame [%d] scores %f\n", __FUNCTION__, i, score);
                if (score > maxScore)
                {
                    maxScore = score;
                    maxIndex = i;
                }
                else if (score < 0)
                {
                    printf("%s << ERROR. Negative score. Returning.\n", __FUNCTION__);
                    return;
                }
            }

            //printf("%s << Top scoring frame #%d gets %f\n", __FUNCTION__, maxIndex, maxScore);
            //cin.get();

            newCorners.push_back(candidatePatterns.at(maxIndex));    // Push highest scorer onto new vector

            addToDistributionMap(distributionMap, newCorners.at(newCorners.size()-1));  // update distribution
            addToRadialDistribution(radialDistribution, newCorners.at(newCorners.size()-1), distributionMap.size());

            prepForDisplay(distributionMap, distributionDisplay);
            imshow("distributionMap", distributionDisplay);

            addToBinMap(binMap, newCorners.at(newCorners.size()-1), distributionMap.size()); // update binned mat
            convertScaleAbs(binMap, binTemp);
            simpleResize(binTemp, distributionDisplay, Size(480, 640));
            equalizeHist(distributionDisplay, distributionDisplay);
            //imshow("binMap", distributionDisplay);

            waitKey( 0 );

            candidatePatterns.erase(candidatePatterns.begin()+maxIndex);    // Erase it from original vector
        }

        candidatePatterns.clear();
        newCorners.swap(candidatePatterns);
        break;
        // ==================================================
    case RANDOM_SET_OPTIMIZATION_CODE:     //              RANDOM FRAME SELECTION
        // ==================================================
        for (int i = 0; i < num; i++)
        {
            randomNum = rand() % int(candidatePatterns.size());
            newCorners.push_back(candidatePatterns.at(randomNum));
            candidatePatterns.erase(candidatePatterns.begin()+randomNum);
            selectedTags.push_back(tagNames.at(randomNum));

            addToDistributionMap(distributionMap, newCorners.at(i));
            prepForDisplay(distributionMap, distributionDisplay);
            imshow("distributionMap", distributionDisplay);
            waitKey(40);
        }

        candidatePatterns.clear();
        newCorners.swap(candidatePatterns);
        break;
        // ==================================================
    case FIRST_N_PATTERNS_OPTIMIZATION_CODE:     //              FIRST N FRAMES SELECTION
        // ==================================================
        while (candidatePatterns.size() > (unsigned int)(num))
        {
            candidatePatterns.pop_back();
        }

        for (int i = 0; i < num; i++)
        {
            addToDistributionMap(distributionMap, candidatePatterns.at(i));
            selectedTags.push_back(tagNames.at(i));
            prepForDisplay(distributionMap, distributionDisplay);
            imshow("distributionMap", distributionDisplay);
            waitKey(40);
        }

        delete[] values;

        return;
        // ==================================================
    case ENHANCED_MCM_OPTIMIZATION_CODE:     //        MULTIPLE-TRIAL OPTIMAL FRAME SELECTION
        // ==================================================

		if (debugMode) { printf("%s << ENTERED. (%d)\n", __FUNCTION__, 2); }
        selectedFrames.clear();

        unrankedScores = new double[candidatePatternsCpy.size()];

        prevBestScore = 9e50;

        //printf("%s << num = %d\n", __FUNCTION__, num);

        for (int N = 0; N < num; N++)
        {

			if (debugMode) { printf("%s << N = (%d / %d)\n", __FUNCTION__, N, num); }
				
            objectPoints.push_back(row);

            //printf("%s << candidatePatternsCpy.size() = %d\n", __FUNCTION__, candidatePatternsCpy.size());

            for (unsigned int i = 0; i < candidatePatternsCpy.size(); i++)
            {
				
				if (debugMode) { printf("%s << i = (%d / %d)\n", __FUNCTION__, i, candidatePatternsCpy.size()); }

                tempFrameTester.clear();
                
                if (debugMode) { printf("%s << i = (%d / %d) {%d}\n", __FUNCTION__, i, candidatePatternsCpy.size(), 0); }
                
                tempFrameTester.assign(selectedFrames.begin(), selectedFrames.end());
                
                if (debugMode) { printf("%s << i = (%d / %d) {%d}\n", __FUNCTION__, i, candidatePatternsCpy.size(), 1); }
                
                tempFrameTester.push_back(candidatePatternsCpy.at(i));
                
                if (debugMode) { printf("%s << i = (%d / %d) {%d}\n", __FUNCTION__, i, candidatePatternsCpy.size(), 2); }

                bool alreadyAdded = false;

                for (unsigned int k = 0; k < addedIndices.size(); k++)
                {
					
					if (debugMode) { printf("%s << k = (%d / %d)\n", __FUNCTION__, k, addedIndices.size()); }
					
                    if (i == addedIndices.at(k))       // this was addedIndices[N] before - but that doesn't make sense...
                    {
                        alreadyAdded = true;
                        //printf("%s << WTF?\n", __FUNCTION__);
                    }
                }
                
                if (debugMode) { printf("%s << i = (%d / %d) {%d}\n", __FUNCTION__, i, candidatePatternsCpy.size(), 3); }

                if (alreadyAdded == true)
                {
                    err = -1.0;
                }
                else
                {

                    randomNum = rand() % 1000 + 1;  // random number between 1 and 1000 (inclusive)

                    Mat fovMat, errMat;
                    //double fovScore, errScore;

                    if (randomNum > (1 - testingProbability)*1000.0)
                    {
                        //printf("%s << Calibrating pattern #%d\n", __FUNCTION__, i);

                        //printf("%s << objectPoints.size() = %d; tempFrameTester.size() = %d\n", __FUNCTION__, objectPoints.size(), tempFrameTester.size());

						//printf("%s << imSize = (%d, %d); objectPoints.at(0).size() = %d; tempFrameTester.at(0).size() = %d\n", __FUNCTION__, imSize.height, imSize.width, objectPoints.at(0).size(), tempFrameTester.at(0).size());

                        calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

                        //printf("%s << objectPoints.at(0).size() = %d; fullSetCorners.size() = %d\n", __FUNCTION__, objectPoints.at(0).size(), fullSetCorners.size());
						
						//for (unsigned int xxx = 0; xxx < fullSetCorners.size(); xxx++) {
						//	printf("%s << fullSetCorners.at(%d).size() = %d\n", __FUNCTION__, xxx, fullSetCorners.at(xxx).size());
						//}

                        err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs, removeSpatialBias, false, useUndistortedLocations);
                        //printf("%s << err = %f\n", __FUNCTION__, err);
                    }
                    else
                    {
                        // If the frame is not to be tested (more likely with lower testingProbability)
                        err = -1.0;
                    }

                }
                
                if (debugMode) { printf("%s << i = (%d / %d) {%d}\n", __FUNCTION__, i, candidatePatternsCpy.size(), 4); }

                unrankedScores[i] = err;
                //printf("%s << score = %f\n", __FUNCTION__, err);
                
                

            }

            bestScore = 9e50;
            bestIndex = 0;
            
            if (debugMode) { printf("%s << X {%d}\n", __FUNCTION__, 0); }

            for (unsigned int j = 0; j < candidatePatternsCpy.size(); j++)
            {
				
				if (debugMode) { printf("%s << j = (%d / %d)\n", __FUNCTION__, j, candidatePatternsCpy.size()); }

                if ((unrankedScores[j] < bestScore) && (unrankedScores[j] > 0))
                {
                    bestScore = unrankedScores[j];
                    bestIndex = j;
                }
            }

            unrankedScores[bestIndex] = 9e50;

            //printf("%s << Best score for %d frame calibration: %f\n", __FUNCTION__, N+1, bestScore);

            selectedFrames.push_back(candidatePatternsCpy.at(bestIndex));

            // Corrupt best frame in 'originalFramesCpy'
            for (unsigned int i = 0; i < candidatePatternsCpy.at(bestIndex).size(); i++)
            {
				if (debugMode) { printf("%s << i(%d) = (%d / %d)\n", __FUNCTION__, bestIndex, i, candidatePatternsCpy.at(bestIndex).size()); }
                candidatePatternsCpy.at(bestIndex).at(i) = Point2f(0.0,0.0);
            }

            addedIndices.push_back(bestIndex);

            if (bestScore < prevBestScore)
            {
                prevBestScore = bestScore;
                optimumNum = N;
            }
            
            
			printf("%s << (%d) frames considered; best generalized error so far = (%f)\n", __FUNCTION__, N+1, prevBestScore);
			

        }

        delete[] unrankedScores;

        if (debugMode) { printf("%s << Optimum number of frames for calibration = %d\n", __FUNCTION__, optimumNum+1); }

        candidatePatterns.clear();
        candidatePatterns.assign(selectedFrames.begin(), selectedFrames.begin() + optimumNum+1);

        for (int i = 0; i < optimumNum+1; i++)
        {
            selectedTags.push_back(tagNames.at(addedIndices.at(i)));
        }
        
        if (debugMode) {
			
			printf("%s << Attempting to add patterns to distribution map..\n", __FUNCTION__);
			
			for (unsigned int iii = 0; iii < candidatePatterns.size(); iii++) {
				printf("%s << Adding pattern (%d)..\n", __FUNCTION__, iii);
				addToDistributionMap(distributionMap, candidatePatterns.at(iii));
			}
			
			
			
			

			//addToBinMap(binMap, newCorners.at(newCorners.size()-1), distributionMap.size()); // update binned mat
			//convertScaleAbs(binMap, binTemp);
			//simpleResize(binTemp, distributionDisplay, Size(480, 640));
			//equalizeHist(distributionDisplay, distributionDisplay);
			//imshow("binMap", distributionDisplay);

			if (generateFigure) {
				
				//printf("%s << Patterns added, prepping display\n", __FUNCTION__);
				prepForDisplay(distributionMap, distributionDisplay);
				//imshow("distributionMap", distributionDisplay);
				imwrite("/home/steve/Dropbox/Steve-Peyman/jfr2013/draft/figures/intrinsics/point_distribution.jpg", distributionDisplay);
				//waitKey( 0 );
			}
			
		}
        
        
        //printf("%s << ENTERED. (%d)\n", __FUNCTION__, 3);

        break;
        // ==================================================
    case RANDOM_SEED_OPTIMIZATION_CODE:     //        Random N-seed accumulative search
        // ==================================================

        selectedFrames.clear();

        unrankedScores = new double[candidatePatternsCpy.size()];

        prevBestScore = 9e50;

        //printf("%s << num = %d\n", __FUNCTION__, num);



        bestSeedSet = new int[nSeeds];
        currentSeedSet = new int[nSeeds];



        for (int iii = 0; iii < nSeedTrials; iii++)
        {

            objectPoints.clear();
            tempFrameTester.clear();

            randomNum = rand() % candidatePatternsCpy.size();

            currentSeedSet[0] = randomNum;
            objectPoints.push_back(row);


            tempFrameTester.push_back(candidatePatternsCpy.at(currentSeedSet[0]));

            for (int jjj = 1; jjj < nSeeds; jjj++)
            {
                do
                {
                    alreadyUsed = false;

                    randomNum = rand() % candidatePatternsCpy.size();

                    for (int kkk = 0; kkk < jjj; kkk++)
                    {
                        if (randomNum == currentSeedSet[kkk])
                        {
                            alreadyUsed = true;
                        }
                    }
                }
                while (alreadyUsed);

                currentSeedSet[jjj] = randomNum;

                objectPoints.push_back(row);
                tempFrameTester.push_back(candidatePatternsCpy.at(currentSeedSet[jjj]));
            }

            calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

            currentSeedScore = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs, removeSpatialBias, false, useUndistortedLocations);

            if (currentSeedScore < bestSeedScore)
            {
                bestSeedScore = currentSeedScore;

                printf("%s << Best seed score [trial = %d]: %f\n", __FUNCTION__, iii, bestSeedScore);

                for (int jjj = 0; jjj < nSeeds; jjj++)
                {
                    bestSeedSet[jjj] = currentSeedSet[jjj];
                }

            }

        }

        for (int jjj = 0; jjj < nSeeds; jjj++)
        {
            selectedFrames.push_back(candidatePatternsCpy.at(bestSeedSet[jjj]));
            unrankedScores[bestSeedSet[jjj]] = 9e50;

            // Corrupt seed frames
            for (unsigned int kkk = 0; kkk < candidatePatternsCpy.at(bestSeedSet[jjj]).size(); kkk++)
            {
                candidatePatternsCpy.at(bestSeedSet[jjj]).at(kkk) = Point2f(0.0,0.0);
            }

            addedIndices.push_back(bestSeedSet[jjj]);
        }

        bestScore = bestSeedScore;

        // Subtract 1 because later code is dodgy... :P
        optimumNum = nSeeds-1;

        for (int N = nSeeds; N < num; N++)
        {

            objectPoints.push_back(row);

            //printf("%s << candidatePatternsCpy.size() = %d\n", __FUNCTION__, candidatePatternsCpy.size());

            for (unsigned int i = 0; i < candidatePatternsCpy.size(); i++)
            {

                tempFrameTester.clear();
                tempFrameTester.assign(selectedFrames.begin(), selectedFrames.end());
                tempFrameTester.push_back(candidatePatternsCpy.at(i));

                bool alreadyAdded = false;

                for (unsigned int k = 0; k < addedIndices.size(); k++)
                {
                    if (i == addedIndices.at(k))       // this was addedIndices[N] before - but that doesn't make sense...
                    {
                        alreadyAdded = true;
                        //printf("%s << WTF?\n", __FUNCTION__);
                    }
                }

                if (alreadyAdded == true)
                {
                    err = -1.0;
                }
                else
                {

                    randomNum = rand() % 1000 + 1;  // random number between 1 and 1000 (inclusive)

                    Mat fovMat, errMat;
                    //double fovScore, errScore;

                    if (randomNum > (1 - testingProbability)*1000.0)
                    {
                        //printf("%s << Calibrating pattern #%d\n", __FUNCTION__, i);

                        //printf("%s << objectPoints.size() = %d; tempFrameTester.size() = %d\n", __FUNCTION__, objectPoints.size(), tempFrameTester.size());

                        calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

                        //printf("%s << objectPoints.at(0).size() = %d; fullSetCorners.size() = %d\n", __FUNCTION__, objectPoints.at(0).size(), fullSetCorners.size());

                        err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs, removeSpatialBias, false, useUndistortedLocations);
                        //printf("%s << err = %f\n", __FUNCTION__, err);
                    }
                    else
                    {
                        // If the frame is not to be tested (more likely with lower testingProbability)
                        err = -1.0;
                    }

                }

                unrankedScores[i] = err;

            }

            bestScore = 9e50;
            bestIndex = 0;

            for (unsigned int j = 0; j < candidatePatternsCpy.size(); j++)
            {

                if ((unrankedScores[j] < bestScore) && (unrankedScores[j] > 0))
                {
                    bestScore = unrankedScores[j];
                    bestIndex = j;
                }
            }

            unrankedScores[bestIndex] = 9e50;

            printf("%s << Best score for %d frame calibration: %f\n", __FUNCTION__, N+1, bestScore);

            selectedFrames.push_back(candidatePatternsCpy.at(bestIndex));

            // Corrupt best frame in 'originalFramesCpy'
            for (unsigned int i = 0; i < candidatePatternsCpy.at(bestIndex).size(); i++)
            {
                candidatePatternsCpy.at(bestIndex).at(i) = Point2f(0.0,0.0);
            }

            addedIndices.push_back(bestIndex);

            if (bestScore < prevBestScore)
            {
                prevBestScore = bestScore;
                optimumNum = N;
            }

        }

        delete[] unrankedScores;

        printf("%s << Optimum number of frames for calibration = %d\n", __FUNCTION__, optimumNum+1);

        candidatePatterns.clear();
        candidatePatterns.assign(selectedFrames.begin(), selectedFrames.begin() + optimumNum+1);

        for (int i = 0; i < optimumNum+1; i++)
        {
            selectedTags.push_back(tagNames.at(addedIndices.at(i)));
        }

        break;
        // ==================================================
    case EXHAUSTIVE_SEARCH_OPTIMIZATION_CODE:     //        EXHAUSTIVE TRUE-OPTIMAL SELECTION
        // ==================================================

        if (candidatePatternsCpy.size() > 20)
        {
            printf("%s << Too many frames for exhaustive approach.\n", __FUNCTION__);
            break;
        }
        else
        {
            printf("%s << Searching for absolute optimum.\n", __FUNCTION__);
        }

        bestScore = 9e99;

        // For each different value of N
        for (int N = 0; N < num; N++)
        {

            topScore = 9e99;

            printf("%s << N(+1) = %d\n", __FUNCTION__, N+1);

            objectPoints.push_back(row);

            possibleCombos = 1;

            possibleCombos = (unsigned long)(factorial(int(candidatePatternsCpy.size())) / (factorial(N+1) * factorial(int(candidatePatternsCpy.size()) - N - 1)));

            printf("%s << possibleCombos = %d\n", __FUNCTION__, possibleCombos);

            currentIndices.clear();

            // For each possible combo
            for (unsigned int i = 0; i < possibleCombos; i++)
            {

                tempFrameTester.clear();
                getNextCombo(currentIndices, N+1, int(candidatePatternsCpy.size()));

                for (unsigned int j = 0; j < currentIndices.size(); j++)
                {
                    //printf("%s << currentIndices.at(%d) = %d\n", __FUNCTION__, j, currentIndices.at(j));
                    tempFrameTester.push_back(candidatePatternsCpy.at(currentIndices.at(j)));
                }

                err = calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

                Mat fovMat, errMat;
                //double fovScore, errScore;

                err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs, removeSpatialBias, false, useUndistortedLocations);

                if (err < topScore)
                {
                    topScore = err;
                    topIndices.clear();
                    topIndices.assign(currentIndices.begin(), currentIndices.end());
                }

                if (err < bestScore)
                {
                    bestScore = err;
                    bestIndices.clear();
                    bestIndices.assign(currentIndices.begin(), currentIndices.end());
                }

            }

            printf("%s << topScore [(N+1) = %d] = %f\n", __FUNCTION__, N+1, topScore);

            for (unsigned int j = 0; j < topIndices.size(); j++)
            {
                printf("%s << topIndices.at(%d) = %d\n", __FUNCTION__, j, topIndices.at(j));
            }

        }

        candidatePatterns.clear();

        printf("%s << Optimum number of frames for calibration = %d\n", __FUNCTION__, bestIndices.size());
        printf("%s << bestScore = %f\n", __FUNCTION__, bestScore);

        for (unsigned int i = 0; i < bestIndices.size(); i++)
        {
            printf("%s << bestIndices.at(%d) = %d\n", __FUNCTION__, i, bestIndices.at(i));
            candidatePatterns.push_back(candidatePatternsCpy.at(bestIndices.at(i)));
        }

        break;
        // ==================================================
    case BEST_OF_RANDOM_PATTERNS_OPTIMIZATION_CODE:     //              MANY RANDOM TRIALS FRAME SELECTION
        // ==================================================

        bestErr = 9e99;

        printf("%s << Random trial selection\n", __FUNCTION__);

        for (int k = 0; k < nTrials; k++)
        {

            //printf("%s << Trial #%d.\n", __FUNCTION__, k);

            objectPoints.clear();
            candidatePatterns.clear();
            currentIndices.clear();
            newCorners.clear();

            candidatePatterns.assign(candidatePatternsCpy.begin(), candidatePatternsCpy.end());

            for (int N = 0; N < num; N++)
            {

                objectPoints.push_back(row);
                randomNum = rand() % int(candidatePatterns.size());
                currentIndices.push_back(randomNum);
                newCorners.push_back(candidatePatterns.at(randomNum));
                candidatePatterns.erase(candidatePatterns.begin()+randomNum);
                //printf("%s << oP.size() = %d; nC.size() = %d\n", __FUNCTION__, objectPoints.size(), newCorners.size());

                err = calibrateCamera(objectPoints, newCorners, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

                Mat fovMat, errMat;
                //double fovScore, errScore;
                err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs, removeSpatialBias, false, useUndistortedLocations);

                values[N*nTrials+k] = err;

                //printf("%s << trial #%d, N = %d, score = %f\n", __FUNCTION__, k, N, err);

                if (err < bestErr)
                {
                    bestErr = err;
                    bestIndices.clear();

                    selectedFrames.clear();

                    bestIndices.assign(currentIndices.begin(), currentIndices.end());

                    selectedFrames.assign(newCorners.begin(), newCorners.end());

                    //printf("%s << new bestErr[N = %d] = %f\n", __FUNCTION__, N, bestErr);
                }
            }
        }

        candidatePatterns.clear();

        for (int N = 0; N < num; N++)
        {
            median = findEquivalentProbabilityScore(&values[N*nTrials], nTrials, 0.5);
            p90 = findEquivalentProbabilityScore(&values[N*nTrials], nTrials, 0.9);
            p99 = findEquivalentProbabilityScore(&values[N*nTrials], nTrials, 0.99);

            //printf("%s << Random results for %d frames: median = %f; p90 = %f; p99 = %f\n", __FUNCTION__, N, median, p90, p99);
        }

        candidatePatterns.assign(selectedFrames.begin(), selectedFrames.end());

        break;
    default:

        delete[] values;

        return;
    }


    delete[] values;

}
