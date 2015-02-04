#include "dd_evaluator.hpp"

int main( int argc, char** argv )
{
	
    // =====================================================================================
    //                      INITIALIZATION AND ARGUMENT PARSING
    // =====================================================================================
#ifdef DONT_USE_THIS_CODE
    configData iD;
    initializeSubsetStrings();
    iD.processArguments(argc, argv);
    printf("%s << dataPath: %s\n", __FUNCTION__, iD.dataPath);

    fileTree fD;
    fD.prepareTree(iD);
    fD.displayTree();

    printf("%s << Selections:\n", __FUNCTION__);

    iD.printParameters();

    vector<string> inputImages;
    vector<vector<vector<Mat> > > superImageVector;

    vector<Mat> rawImageVec, grayImageVec, colImageVec;

    addressNode aD;

    homographyHandler hD;

    vector< Ptr<FeatureDetector> > allFeatureDetectors;

    initializeDetectors(allFeatureDetectors);

    vector< Ptr<DescriptorExtractor> > allFeatureDescriptors;

    initializeDescriptors(allFeatureDescriptors);

    vector< vector< vector<KeyPoint> > > allKeypoints[MAX_TRANSFORM_LEVELS];

    initializeKeypointsTripleVec(allKeypoints);

    eccBlockWrapper eD;

    printf("\n<<-----COMMENCING EVALUATION----->>\n\n");

    // =====================================================================================
    //                      MAIN LOOP
    // =====================================================================================
    for (unsigned int datasetCounter = 0; datasetCounter < fD.datasets.size(); datasetCounter++) {

        printf("%s << current dataset = %s\n", __FUNCTION__, (fD.datasets.at(datasetCounter)).c_str());

        sprintf(aD.imagePath, "%s/images/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str());

        if (!boost::filesystem::exists(aD.imagePath)) {
            printf("%s << aD.imagePath = %s\n", __FUNCTION__, aD.imagePath);
            continue;
        }

        fD.makeDirectories(iD, 0, datasetCounter);

        for (unsigned int modalityCounter = 0; modalityCounter < fD.modalities.size(); modalityCounter++) {
            printf("%s << current modality = %s\n", __FUNCTION__, (fD.modalities.at(modalityCounter)).c_str());

            sprintf(aD.imagePath, "%s/images/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str());

            if (!boost::filesystem::exists(aD.imagePath)) {
                continue;
            }

            fD.makeDirectories(iD, 1, datasetCounter, modalityCounter);

            for (unsigned int subsetCounter = 0; subsetCounter < fD.subsets.size(); subsetCounter++) {

                vector<Mat> warpedCompImages;

                vector<vector<Mat> > graySubsetImages, colSubsetImages;

                bool isDigitalTransformation = false;

                for (unsigned int ppp = 0; ppp < DD_DIGITAL_TRANSFORMATIONS_COUNT; ppp++) {
                    if (!strcmp((DD_DIGITAL_TRANSFORMATIONS[ppp]).c_str(), (fD.subsets.at(subsetCounter)).c_str())) {
                        //printf("%s << Is a digital transformation.\n", __FUNCTION__);
                        isDigitalTransformation = true;
                    }
                }

                printf("%s << current subset = %s\n", __FUNCTION__, (fD.subsets.at(subsetCounter)).c_str());

                sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());

                if (!boost::filesystem::exists(aD.imagePath)) {
                    continue;
                }

                fD.makeDirectories(iD, 2, datasetCounter, modalityCounter, subsetCounter);

                vector<string> subDirectories;

                char imageParentPath[256];
                sprintf(imageParentPath, "%s/images/%s/%s/%s/", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());

                // If profile, create just 1 single child directory with no characters in it
                bool currentlyProfile = false;

                if (!strcmp(fD.subsets.at(subsetCounter).c_str(), "profile")) {
                    currentlyProfile = true;
                }

                if (currentlyProfile) {
                    subDirectories.push_back("");
                } else {
                    assembleSubDirectories(imageParentPath, subDirectories);
                }

                char homogSuffix[64];

                // =====================================================================================
                //                      READ IN ALL IMAGES
                // =====================================================================================

                printf("%s << Reading all images...\n", __FUNCTION__);

                for (int sss = 0; sss < subDirectories.size(); sss++) {

                    if (subDirectories.at(sss) != "") {
                        // If there are some subdirectories, then it isn't a profile folder..?
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                    } else {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                    }

                    printf("%s << aD.imagePath = %s\n", __FUNCTION__, aD.imagePath);

                    inputImages.clear();

                    countAndSortFiles(aD.imagePath, inputImages);

                    printf("%s << aD.imagePath = %s\n", __FUNCTION__, aD.imagePath);
                    //printf("%s << inputImages.at(%d) = %s\n", __FUNCTION__, 0, inputImages.at(0).c_str());

                    grayImageVec.clear();
                    colImageVec.clear();

                    //int imagesToRead = std::min( ((unsigned int) MAX_IMAGES_PER_FOLDER), inputImages.size());

                    printf("%s << imagesToRead = %d\n", __FUNCTION__, imagesToRead);

                    for (int iii = 0; iii < imagesToRead; iii++) {

                        aD.processImage(inputImages.at(iii), rawImageVec, grayImageVec, colImageVec);

                        if (iD.fusionMode) {

                            // Get warped image!!!

                            sprintf(aD.compImagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "thermal", fD.subsets.at(subsetCounter).c_str());
                            //countAndSortFiles(aD.imagePath, inputImages);

                            sprintf(aD.compImageName, "%s/%04d.jpg", aD.compImagePath, iii);

                            printf("%s << compImageName = %s\n", __FUNCTION__, aD.compImageName);
                            Mat raw_therm_im = imread(aD.compImageName, -1);


                            // get homography


                            sprintf(aD.fusionHomogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality", "visible-thermal");

                            char fusionHomogSuffix[256];

                            sprintf(fusionHomogSuffix, "V%04dtoT%04d.xml", iii, iii);
                            sprintf(aD.fusionHomogAddress, "%s/%s", aD.fusionHomogPath, fusionHomogSuffix);

                            printf("%s << fusionaD.homogAddress = %s\n", __FUNCTION__, aD.fusionHomogAddress);

                            Mat H12f;
                            FileStorage fsf(aD.fusionHomogAddress, FileStorage::READ);

                            fsf["Homography"] >> H12f;

                            fsf.release();

                            cout << "H12f = " << H12f << endl;

                            H12f = H12f.inv();

                            Mat warpedImage;
                            warpPerspective(raw_therm_im, warpedImage, H12f, raw_therm_im.size());



                            warpedCompImages.push_back(warpedImage);


                        }
                    }

                    graySubsetImages.push_back(grayImageVec);
                    colSubsetImages.push_back(colImageVec);

                    printf("%s << %d images read in set %d.\n", __FUNCTION__, imagesToRead, sss);

                }

                // Pushing back all images for one modality
                superImageVector.push_back(graySubsetImages);

                printf("%s << All images read.\n", __FUNCTION__);

                // =====================================================================================
                //                      INTER-LEVEL HOMOGRAPHIES
                // =====================================================================================

                // Check for existence of inter-level homographies
                if (fD.subsets.at(subsetCounter) != "profile") {

                    printf("\n<<-----INTER-LEVEL HOMOGRAPHIES----->>\n\n");



                    sprintf(aD.newDirectoryPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), "levels");

                    //printf("%s << aD.newDirectoryPath = %s\n", __FUNCTION__, aD.newDirectoryPath);

                    mkdir(aD.newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);

                    //printf("%s << subDirectories.size() = %d\n", __FUNCTION__, subDirectories.size());

                    sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), "levels");

                    for (int sss = 0; sss < subDirectories.size(); sss++) {

                        bool levelHomographiesAlreadyExist = true;

                        sprintf(homogSuffix, "L%04dtoL%04d.xml", 0, sss);
                        sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                        if (!boost::filesystem::exists(aD.homogAddress)) {
                            levelHomographiesAlreadyExist = false;
                            printf("%s << (%s) doesn't exist...\n", __FUNCTION__, aD.homogAddress);
                        }
                    //}

                        hD.levelHomographyKeypoints[sss].clear();
                        hD.homographyDetector -> detect(graySubsetImages.at(sss).at(0), hD.levelHomographyKeypoints[sss]);

                        sortKeypoints(hD.levelHomographyKeypoints[sss]);

                        Mat drawImg;

                        drawKeypoints(colSubsetImages.at(sss).at(0), hD.levelHomographyKeypoints[sss], drawImg);

                        imshow("testWin", drawImg);
                        waitKey( 50 ); // 50

                        //printf("%s << DISPLAY 2\n", __FUNCTION__);

                        Mat descriptors;

                        hD.homographyExtractor->compute(graySubsetImages.at(sss).at(0), hD.levelHomographyKeypoints[sss], descriptors);

                        hD.homographyDescriptors.push_back(descriptors);



                        //cout << "DONE." << endl;

                    //}

                        if (sss > 0) {



                            if ((!levelHomographiesAlreadyExist) || (iD.regenerateHomographies) || (iD.regenerateAllResults)) {

                                cout << "Generating level homographies... " << endl;

                                Mat H_0_1;

                            //for (int sss = 1; sss < subDirectories.size(); sss++) {

                                Mat H12;
                                Mat H12_32f(3, 3, CV_32FC1);

                                CvMat tmpCvMat;

                                if ((isDigitalTransformation) && (sss > 1)) {
                                    // 0->1 level homography can just be used...

                                    H_0_1.copyTo(H12);

                                } else {
                                    printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, sss, 0);

                                    // First attempt automatic matching...


                                    vector<DMatch> filteredMatches;

                                    crossCheckMatching( hD.descriptorMatcher, hD.homographyDescriptors.at(0), hD.homographyDescriptors.at(sss), filteredMatches, 1 );

                                    printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, sss, 1);

                                    Mat drawImg;

                                    int matchesFlag = DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;

                                    printf("%s << vals = %d, %d, %d\n", __FUNCTION__, hD.levelHomographyKeypoints[0].size(), hD.levelHomographyKeypoints[sss].size(), filteredMatches.size());

                                    drawMatches( colSubsetImages.at(0).at(0), hD.levelHomographyKeypoints[0], colSubsetImages.at(sss).at(0), hD.levelHomographyKeypoints[sss], filteredMatches, drawImg, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), vector<char>(), matchesFlag ); // matchesFlag

                                    printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, sss, 2);

                                    imshow("testWin", drawImg);
                                    waitKey( 50 ); // 50

                                    vector<Point2f> points1, points2;

                                    bool automaticSuccess = true;

                                    if (filteredMatches.size() > 30) {
                                        // Prompt user to check? if bad then change flag

                                        printf("%s << Good number of matches, checking if auto-gen homography is OK.\n", __FUNCTION__);
                                        //cin.get();

                                        vector<int> queryIdxs( filteredMatches.size() ), trainIdxs( filteredMatches.size() );
                                        for( size_t i = 0; i < filteredMatches.size(); i++ )
                                        {
                                            queryIdxs[i] = filteredMatches[i].queryIdx;
                                            trainIdxs[i] = filteredMatches[i].trainIdx;
                                        }

                                        KeyPoint::convert(hD.levelHomographyKeypoints[0], points1, queryIdxs);
                                        KeyPoint::convert(hD.levelHomographyKeypoints[sss], points2, trainIdxs);

                                        vector<uchar> validityMask;
                                        vector<char> validityMask2;

                                        H12 = findHomography( Mat(points1), Mat(points2), validityMask, CV_RANSAC, hD.ransacReprojThreshold );

                                        int validityCount = 0;
                                        for (int qwe = 0; qwe < validityMask.size(); qwe++) {
                                            if (int(validityMask.at(qwe)) == 1) {
                                                validityCount++;
                                                validityMask2.push_back(char(1));
                                            } else {
                                                validityMask2.push_back(char(0));
                                            }
                                        }

                                        cout << "H12 = " << H12 << endl; // validityMask

                                        drawMatches( colSubsetImages.at(0).at(0), hD.levelHomographyKeypoints[0], colSubsetImages.at(sss).at(0), hD.levelHomographyKeypoints[sss], filteredMatches, drawImg, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), validityMask2, matchesFlag ); // matchesFlag

                                        printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, sss, 2);

                                        imshow("testWin", drawImg);
                                        waitKey( 50 ); // 50

                                        printf("%s << validityMask.size() = %d; validityCount = %d\n", __FUNCTION__, validityMask.size(), validityCount);

                                        //for (int qwe = 0; qwe < validityMask.size(); qwe++) {
                                        //    printf("%s << validityMask.at(%d) = %d\n", __FUNCTION__, qwe, int(validityMask.at(qwe)));
                                        //}

                                        //cin.get();

                                        if (validityCount < 8) {
                                            automaticSuccess = false;
                                        }


                                    } else {
                                        automaticSuccess = false;
                                    }

                                    // Or maybe check if

                                    if (!automaticSuccess) {
                                        printf("%s << Insufficient/bad matches, opening manual annotation feature.\n", __FUNCTION__);

                                        ::leftAnnotationPoints.clear();
                                        ::rightAnnotationPoints.clear();

                                        ::imageWidth = colSubsetImages.at(0).at(0).cols;

                                        graySubsetImages.at(0).at(0).copyTo(testImage_1);
                                        graySubsetImages.at(sss).at(0).copyTo(testImage_2);

                                        makeAnnotationPair(::annotationImage, colSubsetImages.at(0).at(0), colSubsetImages.at(sss).at(0));

                                        imshow("testWin", ::annotationImage);

                                        setMouseCallback( "testWin", onMouse, 0 );

                                        waitKey( );

                                        if (leftAnnotationPoints.size() > rightAnnotationPoints.size()) {
                                            leftAnnotationPoints.pop_back();
                                        } else if (rightAnnotationPoints.size() > leftAnnotationPoints.size()) {
                                            rightAnnotationPoints.pop_back();
                                        }

                                        points1.assign(leftAnnotationPoints.begin(), leftAnnotationPoints.end());
                                        points2.assign(rightAnnotationPoints.begin(), rightAnnotationPoints.end());

                                        for (int abc = 0; abc < points1.size(); abc++) {
                                            printf("%s << (%f, %f) to (%f, %f)\n", __FUNCTION__, points1.at(abc).x, points1.at(abc).y, points2.at(abc).x, points2.at(abc).y);
                                        }

                                        vector<uchar> validityMask;
                                        H12.release();

                                        H12 = findHomography( Mat(points1), Mat(points2), validityMask, CV_RANSAC, hD.ransacReprojThreshold );

                                        cout << "H12 = " << H12 << endl;

                                        // cin.get();
                                    }


                                    Mat H12_32f_default(3, 3, CV_32FC1);
                                    H12.assignTo(H12_32f, CV_32FC1);
                                    H12_32f.copyTo(H12_32f_default);

                                    //cout << "H12_32f = " << H12_32f  << endl;

                                    tmpCvMat = (CvMat) H12_32f;
                                    eD.warp_matrix = &tmpCvMat;

                                    cout << "H12_32f = " << H12_32f  << endl;

                                    IplImage tmpIplIm1, tmpIplIm2;

                                    tmpIplIm1 = (IplImage) graySubsetImages.at(sss).at(0);
                                    tmpIplIm2 = (IplImage) graySubsetImages.at(0).at(0);
                                    eD.target_image = &tmpIplIm1;
                                    eD.template_image = &tmpIplIm2;

                                    if (eD.warp_mode == WARP_MODE_AFFINE) {
                                        eD.warp_matrix->rows = 2;
                                    }

                                    printf("%s << implementing algorithm...\n", __FUNCTION__);


                                    CvMat dummyMat;

                                    CvMat * result_matrix = NULL;

                                    result_matrix = cvFindTransform (eD.template_image, eD.target_image, eD.warp_matrix, eD.warp_mode,
                                                        cvTermCriteria (cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                                                                        eD.number_of_iterations, eD.termination_eps));

                                    printf("%s << algorithm implemented.\n", __FUNCTION__);


                                    //imshow("wrp1", graySubsetImages.at(sss).at(0));
                                    //waitKey(50);
                                    Mat warpedImage;
                                    warpPerspective(graySubsetImages.at(0).at(0), warpedImage, H12_32f_default, graySubsetImages.at(0).at(0).size());
                                    //printf("%s << Showing warped image #1\n", __FUNCTION__);
                                    //imshow("wrp1", warpedImage);
                                    //waitKey(50);


                                    printf("%s << assigning matrix...\n", __FUNCTION__);

                                    /*
                                    if (result_matrix->data == NULL) {
                                        printf("%s << result_matrix->data == NULL!\n", __FUNCTION__);
                                    }
                                    */

                                    //printf("%s << result_matrix\n", __FUNCTION__, result_matrix);

                                    Mat H_result(3, 3, CV_32FC1);

                                    // H_result.at<float>(0,0) = 0.0;

                                    if (result_matrix == NULL) {
                                        //printf("%s << ICA Failed. Retaining feature-based estimation.\n", __FUNCTION__);
                                    } else {
                                        //printf("%s << result_matrix.size() = (%d,%d)\n", __FUNCTION__, result_matrix->rows, result_matrix->cols);
                                        H_result = Mat(result_matrix, false);
                                        //printf("%s << matrix assigned.\n", __FUNCTION__);

                                        //imshow("wrp2", graySubsetImages.at(sss).at(0));
                                        //waitKey(50);
                                        warpPerspective(graySubsetImages.at(0).at(0), warpedImage, H_result, graySubsetImages.at(0).at(0).size());
                                        //printf("%s << Showing warped image #2\n", __FUNCTION__);
                                        //imshow("wrp2", warpedImage);
                                        //waitKey(50);

                                        //cout << "H12_32f_default = " << H12_32f_default << endl;

                                        //cout << "H_result = " << H_result << endl;

                                        bool replacedWithNew = false;

                                        replacedWithNew = retainMostAppropriateHomography(graySubsetImages.at(0).at(0), graySubsetImages.at(sss).at(0), H12_32f_default, H_result);

                                        if (replacedWithNew) {
                                            //printf("%s << Replaced match-based homography with ICA refinement.\n", __FUNCTION__);
                                        } else {
                                            //printf("%s << Did not replace match-based homography with ICA refinement.\n", __FUNCTION__);
                                        }
                                    }

                                    H12_32f_default.assignTo(H12, CV_64FC1);

                                    if (sss == 1) {
                                        H12.copyTo(H_0_1);
                                    }
                                }



                                //cout << "H12 = " << H12 << endl;

                                //H_result.copyTo(H12);

                                //cout << "Homography[" << iii << "][" << jjj << "] = " << H12 << endl;

                                char homogSuffix[64];

                                sprintf(homogSuffix, "L%04dtoL%04d.xml", 0, sss);

                                sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                //cout << "homog address = " << aD.homogAddress << endl;

                                FileStorage fs(aD.homogAddress, FileStorage::WRITE);

                                fs << "Homography" << H12;

                                fs.release();

                                //cin.get();

                                // Then need to write homography out in the correct place...
                                // (may need to create folders)

                            //}

                            //cout << "DONE." << endl;
                            }
                        }
                    }
                }

                // =====================================================================================
                //                      REGULAR HOMOGRAPHIES
                // =====================================================================================

                int numImagesInBaseFolder = 0;

                for (int sss = 0; sss < subDirectories.size(); sss++) {

                    // Make directories only if needed
                    if (subDirectories.at(sss) != "") {
                        // If there are some subdirectories, then it isn't a profile folder..?
                        sprintf(aD.newDirectoryPath, "%s/results/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        mkdir(aD.newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);

                        sprintf(aD.newDirectoryPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        mkdir(aD.newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);

                        sprintf(aD.newDirectoryPath, "%s/keypoints/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        mkdir(aD.newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);

                        sprintf(aD.newDirectoryPath, "%s/descriptors/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        mkdir(aD.newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);

                        sprintf(aD.newDirectoryPath, "%s/descresults/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        mkdir(aD.newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);

                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.resultsPath, "%s/results/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());

                    } else {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.resultsPath, "%s/results/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                    }

                    //cout << "current image folder = " << aD.imagePath << endl;
                    //cout << "profile mode, therefore no image subdirectories..." << endl;

                    //cin.get();

                    // Count images
                    inputImages.clear();

                    countAndSortFiles(aD.imagePath, inputImages);

                    if (sss == 0) {
                        numImagesInBaseFolder = inputImages.size();
                    }

                    rawImageVec.clear();
                    grayImageVec.clear();
                    colImageVec.clear();

                    //cout << "Processing images... (" << inputImages.size() << ") ";

                    hD.homographyDescriptors.clear();

                    // Check if homographies exist
                    bool homographiesAlreadyExist = true;

                    int imagesToRead = min((unsigned int)MAX_IMAGES_PER_FOLDER, inputImages.size());

                    for (int iii = 0; iii < imagesToRead-1; iii++) {
                            for (int jjj = iii+1; jjj < imagesToRead; jjj++) {
                                sprintf(homogSuffix, "H%04dtoH%04d.xml", iii, jjj);
                                sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                if (!boost::filesystem::exists(aD.homogAddress)) {
                                    homographiesAlreadyExist = false;
                                }

                            }
                    }

                    for (int iii = 0; iii < imagesToRead; iii++) {
                        //cout << "current image = " << inputImages.at(iii).c_str() << endl;

                        //cout << ".";
                        //cout.flush();

                        //printf("%s << aD.imagePath = %s\n", __FUNCTION__, aD.imagePath);
                        //printf("%s << inputImages.at(%d) = %s\n", __FUNCTION__, iii, inputImages.at(iii).c_str());


                        sprintf(aD.imageAddress, "%s/%s", aD.imagePath, inputImages.at(iii).c_str());

                        Mat currImg, grayImg, colImg, drawImg;

                        currImg = imread(aD.imageAddress, -1);
                        imshow("testWin", currImg);
                        waitKey( 50 );

                        rawImageVec.push_back(currImg);

                        if (currImg.channels() == 3) {
                            cvtColor(currImg, grayImg, cv::COLOR_RGB2GRAY);
                            currImg.copyTo(colImg);
                        } else {
                            currImg.copyTo(grayImg);
                            cvtColor(currImg, colImg, cv::COLOR_GRAY2RGB);
                        }


                        grayImageVec.push_back(grayImg);
                        colImageVec.push_back(colImg);

                        for (int jjj = 0; jjj < iii; jjj++) {
                            bool similarityLevel = compareMatrices(grayImageVec.at(iii), grayImageVec.at(jjj));

                            if (similarityLevel) {
                                printf("%s << ERROR! Images #%d and #%d are identical.\n", __FUNCTION__, iii, jjj);
                                cin.get();
                                return -1;
                            }
                        }

                        if ((!homographiesAlreadyExist) || (iD.regenerateHomographies) || (iD.regenerateAllResults)) {

                            if ((isDigitalTransformation) && (sss > 1)) {
                                // Shouldn't need to extract any features to determine homographies..
                            } else {
                                hD.homographyKeypoints[iii].clear();
                                hD.homographyDetector -> detect(grayImg, hD.homographyKeypoints[iii]);

                                sortKeypoints(hD.homographyKeypoints[iii]);

                                drawKeypoints(colImg, hD.homographyKeypoints[iii], drawImg);

                                imshow("testWin", drawImg);
                                waitKey( 50 );

                                Mat descriptors;

                                hD.homographyExtractor->compute(grayImg, hD.homographyKeypoints[iii], descriptors);

                                hD.homographyDescriptors.push_back(descriptors);
                            }


                        }


                    }

                    //cout << "DONE." << endl;


                    if ((!homographiesAlreadyExist) || (iD.regenerateHomographies) || (iD.regenerateAllResults)) {

                        //cout << "Generating homographies... ";
                        printf("%s << Generating homographies...\n", __FUNCTION__);

                        for (int iii = 0; iii < imagesToRead-1; iii++) {
                            for (int jjj = iii+1; jjj < imagesToRead; jjj++) {

                                Mat H12;

                                // IF IT'S A DIGITAL TRANSFORMATION, AND NOT IN THE BASE-FOLDER
                                // THEN YOU CAN SIMPLY DETERMINE HOMOGRAPHIES USING INTRA-LEVEL HOMOGRAPHIES FROM BASE FOLDER

                                if ((isDigitalTransformation) && (sss > 1)) {
                                    // Read in relevant homographies...


                                    FileStorage fs;

                                    char hPath[256];
                                    char homogSuffix[64];

                                    //bool isInverted = false;

                                    //int firstIndex, secondIndex;

                                    //firstIndex = (numImagesInBaseFolder + iii - 1) % numImagesInBaseFolder;
                                    //secondIndex = (numImagesInBaseFolder + jjj - 1) % numImagesInBaseFolder;

                                    //printf("%s << Finding (%d -> %d) by using (%d -> %d)\n", __FUNCTION__, iii, jjj, firstIndex, secondIndex);

                                    sprintf(hPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(1).c_str());

                                    sprintf(homogSuffix, "H%04dtoH%04d.xml", iii, jjj);
                                    sprintf(aD.homogAddress, "%s/%s", hPath, homogSuffix);
                                    fs.open(aD.homogAddress, FileStorage::READ);
                                    fs["Homography"] >> H12;
                                    fs.release();

                                    // If this is the case, then the indices will have to be flipped etc..
                                    /*
                                    if (secondIndex < firstIndex) {
                                        isInverted = true;

                                        Mat H_input;

                                        sprintf(homogSuffix, "H%04dtoH%04d.xml", secondIndex, firstIndex);
                                        sprintf(aD.homogAddress, "%s/%s", hPath, homogSuffix);
                                        fs.open(aD.homogAddress, FileStorage::READ);
                                        fs["Homography"] >> H_input;
                                        fs.release();

                                        invert(H_input, H12);


                                    } else {
                                        sprintf(homogSuffix, "H%04dtoH%04d.xml", firstIndex, secondIndex);
                                        sprintf(aD.homogAddress, "%s/%s", hPath, homogSuffix);
                                        fs.open(aD.homogAddress, FileStorage::READ);
                                        fs["Homography"] >> H12;
                                        fs.release();
                                    }
                                    */


                                } else {
                                    //cout << ".";
                                    //cout.flush();
                                    vector<DMatch> filteredMatches;

                                    crossCheckMatching( hD.descriptorMatcher, hD.homographyDescriptors.at(iii), hD.homographyDescriptors.at(jjj), filteredMatches, 1 );

                                    Mat drawImg;

                                    int matchesFlag = DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;
                                    drawMatches( colImageVec.at(iii), hD.homographyKeypoints[iii], colImageVec.at(jjj), hD.homographyKeypoints[jjj], filteredMatches, drawImg, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), vector<char>(), matchesFlag );

                                    imshow("testWin", drawImg);
                                    waitKey( 50 );

                                    vector<int> queryIdxs( filteredMatches.size() ), trainIdxs( filteredMatches.size() );
                                    for( size_t i = 0; i < filteredMatches.size(); i++ )
                                    {
                                        queryIdxs[i] = filteredMatches[i].queryIdx;
                                        trainIdxs[i] = filteredMatches[i].trainIdx;
                                    }

                                    vector<Point2f> points1;
                                    KeyPoint::convert(hD.homographyKeypoints[iii], points1, queryIdxs);

                                    vector<Point2f> points2;
                                    KeyPoint::convert(hD.homographyKeypoints[jjj], points2, trainIdxs);


                                    H12 = findHomography( Mat(points1), Mat(points2), CV_RANSAC, hD.ransacReprojThreshold );

                                    // NOW HERE IS WHERE YOU IMPLEMENT eccBlock STUFF:

                                    Mat H12_32f(3, 3, CV_32FC1), H12_32f_default(3, 3, CV_32FC1);
                                    H12.assignTo(H12_32f, CV_32FC1);
                                    H12_32f.copyTo(H12_32f_default);

                                    //cout << "H12_32f = " << H12_32f  << endl;

                                    CvMat tmp123;
                                    tmp123 = (CvMat) H12_32f;

                                    eD.warp_matrix = &tmp123;

                                    //cout << "H12_32f = " << H12_32f  << endl;

                                    IplImage tmpImX1, tmpImX2;

                                    tmpImX1 = (IplImage) grayImageVec.at(jjj);
                                    tmpImX2 = (IplImage) grayImageVec.at(iii);

                                    eD.target_image = &tmpImX1;
                                    eD.template_image = &tmpImX2;

                                    if (eD.warp_mode == WARP_MODE_AFFINE) {
                                        eD.warp_matrix->rows = 2;
                                    }

                                    //printf("%s << Implementing ICA...\n", __FUNCTION__);


                                    CvMat dummyMat;

                                    CvMat * result_matrix = NULL;

                                    result_matrix = cvFindTransform (eD.template_image, eD.target_image, eD.warp_matrix, eD.warp_mode,
                                                        cvTermCriteria (cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                                                                        eD.number_of_iterations, eD.termination_eps));

                                    //printf("%s << algorithm implemented.\n", __FUNCTION__);

                                    //Mat warpedImage;

                                    /*
                                    imshow("wrp1", grayImageVec.at(jjj));
                                    waitKey(50);

                                    warpPerspective(grayImageVec.at(iii), warpedImage, H12_32f_default, grayImageVec.at(iii).size());
                                    printf("%s << Showing warped image #1\n", __FUNCTION__);
                                    imshow("wrp1", warpedImage);
                                    waitKey(50);
                                    */


                                    //printf("%s << assigning matrix...\n", __FUNCTION__);

                                    /*
                                    if (result_matrix->data == NULL) {
                                        printf("%s << result_matrix->data == NULL!\n", __FUNCTION__);
                                    }
                                    */

                                    //printf("%s << result_matrix\n", __FUNCTION__, result_matrix);

                                    Mat H_result(3, 3, CV_32FC1);

                                    // H_result.at<float>(0,0) = 0.0;

                                    if (result_matrix == NULL) {
                                        //printf("%s << ICA Failed. Retaining feature-based estimation.\n", __FUNCTION__);
                                    } else {
                                        //printf("%s << result_matrix.size() = (%d,%d)\n", __FUNCTION__, result_matrix->rows, result_matrix->cols);
                                        H_result = Mat(result_matrix, false);
                                        //printf("%s << matrix assigned.\n", __FUNCTION__);

                                        /*
                                        imshow("wrp2", grayImageVec.at(jjj));
                                        waitKey(50);
                                        */
                                        //warpPerspective(grayImageVec.at(iii), warpedImage, H_result, grayImageVec.at(iii).size());

                                        /*
                                        printf("%s << Showing warped image #2\n", __FUNCTION__);
                                        imshow("wrp2", warpedImage);
                                        waitKey(50);
                                        */

                                        //cout << "H12_32f_default = " << H12_32f_default << endl;

                                        //cout << "H_result = " << H_result << endl;

                                        bool replacedWithNew = false;

                                        replacedWithNew = retainMostAppropriateHomography(grayImageVec.at(iii), grayImageVec.at(jjj), H12_32f_default, H_result);

                                        if (replacedWithNew) {
                                            //printf("%s << Replaced match-based homography with ICA refinement.\n", __FUNCTION__);
                                        } else {
                                            //printf("%s << Did not replace match-based homography with ICA refinement.\n", __FUNCTION__);
                                            //cin.get();
                                        }
                                    }

                                    H12_32f_default.assignTo(H12, CV_64FC1);
                                }





                                //cout << "H12 = " << H12 << endl;


                                // ===============================================


                                //cout << "Homography[" << iii << "][" << jjj << "] = " << H12 << endl;

                                char homogSuffix[64];

                                sprintf(homogSuffix, "H%04dtoH%04d.xml", iii, jjj);

                                sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                //cout << "homog address = " << aD.homogAddress << endl;

                                FileStorage fs(aD.homogAddress, FileStorage::WRITE);

                                fs << "Homography" << H12;

                                fs.release();

                                imshow("im1", colImageVec.at(iii));
                                //waitKey();

                                /*
                                if (sss > 0) {
                                    printf("%s << Showing original image #2\n", __FUNCTION__);
                                    imshow("wrp", colImageVec.at(jjj));
                                    waitKey();
                                    Mat warpedImage;
                                    warpPerspective(colImageVec.at(iii), warpedImage, H12, colImageVec.at(iii).size());
                                    printf("%s << Showing warped image #1\n", __FUNCTION__);
                                    imshow("wrp", warpedImage);
                                    waitKey();
                                }
                                */



                                //cin.get();

                                // Then need to write homography out in the correct place...
                                // (may need to create folders)
                            }
                        }

                        //cout << "DONE." << endl;

                    }
                }

                printf("\n<<-----FEATURE DETECTION MODULE----->>\n\n");

                // =====================================================================================
                //                      FEATURE DETECTION
                // =====================================================================================
                for (int sss = 0; sss < subDirectories.size(); sss++) {

                    if (subDirectories.at(sss) != "") {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                    sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.resultsPath, "%s/results/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());

                    } else {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.resultsPath, "%s/results/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                    }

                    // For each detector
                    for (int ddd = 0; ddd < DD_DETECTORS_COUNT; ddd++) {

                        // Check if keypoints already exist
                        bool keypointsAlreadyExist = true;
                        char featuresSuffix[64];

                        int imagesToRead = min((unsigned int)MAX_IMAGES_PER_FOLDER, graySubsetImages.at(sss).size());

                        for (int iii = 0; iii < imagesToRead; iii++) {

                            sprintf(featuresSuffix, "%s-%04d.txt", (DD_DETECTOR_NAMES[ddd]).c_str(), iii);
                            sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);

                            if (!boost::filesystem::exists(aD.featuresAddress)) {
                                keypointsAlreadyExist = false;
                                break;
                            }
                        }

                        if ((!keypointsAlreadyExist) || (iD.regenerateFeatures) || (iD.regenerateAllResults)) {
                            if (DD_DETECTOR_NAMES[ddd] == "HES") {
                                printf("%s << HES detector not implemented.\n", __FUNCTION__);
                                break;
                            } else if (DD_DETECTOR_NAMES[ddd] == "MSER") {
                                //printf("%s << MSR detector not implemented.\n", __FUNCTION__);
                                //break;
                            }

                            printf("%s << Implementing %s detector...\n", __FUNCTION__, (DD_DETECTOR_NAMES[ddd]).c_str());

                            //cout << "Applying " << DD_DETECTOR_NAMES[ddd] << " detector to all images... ";

                            // For each image
                            for (int iii = 0; iii < imagesToRead; iii++) {

                                //cout << ".";
                                //cout.flush();
                                // DD_DETECTOR_NAMES

                                vector<KeyPoint> currPoints, reducedPoints;

                                if (DD_DETECTOR_NAMES[ddd] == "RAND") {
                                    randomDetection(graySubsetImages.at(sss).at(iii), currPoints, DEFAULT_MAX_FEATURES, false);
                                } else if (DD_DETECTOR_NAMES[ddd] == "FT0") {
                                    printf("%s << This detector is not currently implemented. Exiting.\n", __FUNCTION__);
                                    return -1;
                                    //trainedFASTDetection(graySubsetImages.at(sss).at(iii), currPoints, DEFAULT_MAX_FEATURES);
                                } else if (DD_DETECTOR_NAMES[ddd] == "SIFT") {

                                    double threshold = 0.04 / 4 / 2.0;
                                    double edgeThreshold = 10.0;

                                    do {
                                        allFeatureDetectors.at(ddd) = new SiftFeatureDetector(threshold, edgeThreshold);
                                        allFeatureDetectors.at(ddd) -> detect(graySubsetImages.at(sss).at(iii), currPoints);

                                        threshold /= 0.8;
                                        edgeThreshold /= 0.8;
                                        //printf("%s << Detected features = %d\n", __FUNCTION__, currPoints.size());
                                        //cin.get();
                                    } while (currPoints.size() > 350);

                                } else if (DD_DETECTOR_NAMES[ddd] == "SimpleBlob") {

                                    SimpleBlobDetector::Params blobParams;


                                    do {
                                        allFeatureDetectors.at(ddd) = new SimpleBlobDetector(blobParams);
                                        allFeatureDetectors.at(ddd) -> detect(graySubsetImages.at(sss).at(iii), currPoints);

                                        blobParams.minThreshold *= 0.8;
                                        blobParams.maxThreshold *= 1.25;
                                        printf("%s << Detected features = %d\n", __FUNCTION__, currPoints.size());
                                        //cin.get();
                                    } while (currPoints.size() < 300);

                                } else {
                                    allFeatureDetectors.at(ddd) -> detect(graySubsetImages.at(sss).at(iii), currPoints);

                                    printf("%s << Detected features = %d\n", __FUNCTION__, currPoints.size());
                                }

                                if (DD_DETECTOR_NAMES[ddd] == "SIFT") {



                                    for (unsigned int qwt = 0; qwt < currPoints.size(); qwt++) {
                                        //printf("%s << SIFT(%d) = %f\n", __FUNCTION__, qwt, currPoints.at(qwt).response);
                                    }

                                    //printf("%s << Detected features = %d\n", __FUNCTION__, currPoints.size());
                                    //cin.get();

                                    assignEigenVectorResponses(grayImageVec.at(iii), currPoints);

                                    //cin.get();

                                } else if ((DD_DETECTOR_NAMES[ddd] == "GFTT") || (DD_DETECTOR_NAMES[ddd] == "HARRIS")) {

                                    //assignChronologicalResponses(currPoints); // Should change this to something more mathematically sound...
                                    assignEigenVectorResponses(grayImageVec.at(iii), currPoints);

                                    if ((DD_DETECTOR_NAMES[ddd] == "GFTT") || (DD_DETECTOR_NAMES[ddd] == "HARRIS")) {
                                        assignMinimumRadii(currPoints);
                                    }
                                } else if (DD_DETECTOR_NAMES[ddd] == "MSER") {

                                    assignStabilityResponses(grayImageVec.at(iii), currPoints);

                                }





                                sortKeypoints(currPoints, ABSOLUTE_MAX_KEYPOINTS_TO_RETAIN);

                                if (currPoints.size() > 100) {
                                    reducedPoints.assign(currPoints.begin(), currPoints.begin() + 100);
                                } else if (currPoints.size() > 0) {
                                    reducedPoints.assign(currPoints.begin(), currPoints.begin() + currPoints.size()-1);
                                } else {
                                    // ...
                                }



                                allKeypoints[sss][ddd][iii].assign(currPoints.begin(), currPoints.end());

                                for (int zzz = 0; zzz < currPoints.size(); zzz++) {
                                    //printf("%s << currPoints.at(%d).response = %f\n", __FUNCTION__, zzz, currPoints.at(zzz).response);
                                }

                                //allFeatureDetectors.at(ddd) -> detect(grayImageVec.at(iii), allKeypoints[ddd][iii]);

                                //cout << "detector[" << ddd << "]; image[" << iii << "]; count = " << currPoints.size() << endl;

                                Mat drawImg;

                                //drawKeypoints(colImageVec.at(iii), currPoints, drawImg);

                                printf("%s << cS.size() = %d; cS.at(%d).size() = %d\n", __FUNCTION__, colSubsetImages.size(), sss, colSubsetImages.at(sss).size());

                                displayKeypoints(colSubsetImages.at(sss).at(iii), reducedPoints, drawImg, cv::Scalar(255,0,0), 0);

                                /*
                                if ((DD_DETECTOR_NAMES[ddd] == "SURF") && (sss == 0)) {
                                    imshow("SURF_0", drawImg);
                                    waitKey( 0 );
                                } else if ((DD_DETECTOR_NAMES[ddd] == "FAST") && (sss == 8)) {
                                    //imshow("FAST_8", drawImg);
                                    //waitKey( 0 );
                                } else {
                                    imshow("testWin", drawImg);
                                    waitKey( 50 );
                                }
                                */


                                imshow("testWin", drawImg);
                                waitKey( 50 );

                                // Spit out the results to a file
                                FileStorage fs;

                                char featuresSuffix[64];

                                sprintf(featuresSuffix, "%s-%04d.txt", (DD_DETECTOR_NAMES[ddd]).c_str(), iii);

                                sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);

                                //printf("%s << Writing features...\n", __FUNCTION__);
                                //printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                                writeKeypoints_MATLAB(aD.featuresAddress, currPoints);

                                //fs = FileStorage(aD.featuresAddress, FileStorage::WRITE);
                                //writeKeypoints(fs, currPoints, iii);
                                //fs.release();

                            }
                        } else {
                            // HAS TO READ IN KEYPOINTS NOW!!!!

                            for (int iii = 0; iii < imagesToRead; iii++) {
                                char featuresSuffix[64];

                                sprintf(featuresSuffix, "%s-%04d.txt", (DD_DETECTOR_NAMES[ddd]).c_str(), iii);

                                sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);

                                //printf("%s << Reading features...\n", __FUNCTION__);
                                //printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                                vector<KeyPoint> currPoints;

                                readKeypoints_MATLAB(aD.featuresAddress, currPoints);



                                //printf("%s << Features read.\n", __FUNCTION__);

                                sortKeypoints(currPoints);
                                allKeypoints[sss][ddd][iii].assign(currPoints.begin(), currPoints.end());

                                /*
                                Mat drawImg;

                                //drawKeypoints(colImageVec.at(iii), currPoints, drawImg);\


                                vector<KeyPoint> reducedPoints;

                                reducedPoints.assign(currPoints.begin(), currPoints.begin() + 200);

                                displayKeypoints(colImageVec.at(iii), reducedPoints, drawImg, cv::Scalar(255,0,0), 0);

                                imshow("testWin", drawImg);
                                waitKey( 0 );
                                */

                                //printf("%s << Keypoints sorted and assigned.\n", __FUNCTION__);


                            }
                        }

                    }

                    //cout << "DONE." << endl;

                }

                // =====================================================================================
                //                      INTER-LEVEL REPEATABILITY
                // =====================================================================================
                if (fD.subsets.at(subsetCounter) != "profile") {

                    // For base image, define it and other paths
                    vector<string> baseImages, inputImages;

                    sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(0).c_str());

                    countAndSortFiles(aD.imagePath, baseImages);

                    //printf("%s << aD.imagePath = %s\n", __FUNCTION__, aD.imagePath);
                    //printf("%s << baseImages.size() = %d\n", __FUNCTION__, baseImages.size());

                    //rawImageVec.clear();
                    //grayImageVec.clear();
                    //colImageVec.clear();

                    int baseImagesToRead = min((unsigned int)MAX_IMAGES_PER_FOLDER, baseImages.size());

                    //cout << "Processing images... (" << imagesToRead << ") ";


                    sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), "levels");

                    for (int sss = 1; sss < subDirectories.size(); sss++) {

                        inputImages.clear();


                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        countAndSortFiles(aD.imagePath, inputImages);

                        int imagesToRead = min((unsigned int)baseImagesToRead, inputImages.size());

                        printf("%s << aD.imagePath[%d] = %s\n", __FUNCTION__, sss, aD.imagePath);
                        printf("%s << inputImages to read from [%d] = %d\n", __FUNCTION__, sss, imagesToRead);

                        // Define image/other paths

                        sprintf(aD.resultsPath, "%s/results/%s/%s/%s/%s-BR", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);

                        sprintf(aD.matchResultsPath, "%s/match_results/%s/%s/%s/%s-BR", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        mkdir(aD.matchResultsPath, DEFAULT_MKDIR_PERMISSIONS);

                        // For each detector
                        for (int ddd = 0; ddd < DD_DETECTORS_COUNT; ddd++) {
                            // Check if keypoints already exist
                            bool resultsAlreadyExist = true;
                            char resultsSuffix[64], matchResultsSuffix[64];

                            sprintf(resultsSuffix, "%s.csv", DD_DETECTOR_NAMES[ddd].c_str());
                            sprintf(aD.resultsAddress, "%s/%s", aD.resultsPath, resultsSuffix);

                            if (!boost::filesystem::exists(aD.resultsAddress)) {
                                resultsAlreadyExist = false;
                                //break;
                            }

                            sprintf(matchResultsSuffix, "%s.csv", DD_DETECTOR_NAMES[ddd].c_str());
                            sprintf(aD.matchResultsAddress, "%s/%s", aD.matchResultsPath, matchResultsSuffix);

                            if (!boost::filesystem::exists(aD.matchResultsAddress)) {
                                resultsAlreadyExist = false;
                                //break;
                            }

                            if (resultsAlreadyExist) {
                                printf("%s << Repeatability results for %s detector already exist.\n", __FUNCTION__, DD_DETECTOR_NAMES[ddd].c_str());
                            }


                            if ((!resultsAlreadyExist) || (iD.regenerateDetectorResults) || (iD.regenerateAllResults)) {
                                // ...

                                //cout << "Calculating repeatability for " << DD_DETECTOR_NAMES[ddd] << " detector between levels... ";
                                printf("%s << Calculating repeatability results for %s detector...\n", __FUNCTION__, DD_DETECTOR_NAMES[ddd].c_str());

                                int minMaxCount = 9999;

                                //printf("%s << DEBUG %d\n", __FUNCTION__, 0);

                                for (int iii = 0; iii < imagesToRead; iii++) {
                                    // figure out typical response for that count
                                    int totalFeats = allKeypoints[sss][ddd][iii].size();

                                    printf("%s << totalFeats = %d\n", __FUNCTION__, totalFeats);

                                    if (totalFeats < minMaxCount) {
                                        minMaxCount = totalFeats;
                                    }

                                    totalFeats = allKeypoints[0][ddd][iii].size();

                                    if (totalFeats < minMaxCount) {
                                        minMaxCount = totalFeats;
                                    }

                                }

                                //printf("%s << minMaxCount = %d\n", __FUNCTION__, minMaxCount);

                                vector<int> averageCounts;
                                vector<float> averageScores, averageMatchScores;
                                vector<float> upperScores, upperMatchScores;
                                vector<float> lowerScores, lowerMatchScores;

                                // For each feature count level

                                int initialCount, numberOfReps = 1;

                                for (int fff = 0; fff < numberOfReps; fff++) {

                                    int aimedCount = 0;

                                    if (currentlyProfile) {
                                        aimedCount = DD_PROFILE_LIST[fff];
                                    } else {
                                        aimedCount = DEFAULT_FEATURE_COUNT;
                                    }

                                    //printf("%s << aimedCount = %d; minMaxCount = %d\n", __FUNCTION__, aimedCount, minMaxCount);

                                    if (aimedCount > minMaxCount) {
                                        printf("%s << Insufficient features for analysis. (%d / %d)\n", __FUNCTION__, minMaxCount, aimedCount);
                                        break;
                                    }


                                    //printf("%s << DEBUG fff[%d] %d\n", __FUNCTION__, fff, 0);

                                    int currAvCount = 0;

                                    //cout << ".";
                                    //cout.flush();

                                    double minAbsResponseBase = 0.0, minAbsResponseNew = 0.0;

                                    // For each image
                                    for (int iii = 0; iii < min((unsigned int)imagesToRead, baseImages.size()); iii++) {
                                        // figure out typical response for that count

                                        minAbsResponseBase += abs(allKeypoints[0][ddd][iii].at(aimedCount-1).response);

                                    }

                                    //printf("%s << DEBUG fff[%d] %d\n", __FUNCTION__, fff, 1);

                                    minAbsResponseBase /= min((unsigned int)imagesToRead, baseImages.size());

                                    // For each image
                                    for (int iii = 0; iii < min((unsigned int)imagesToRead, inputImages.size()); iii++) {
                                        // figure out typical response for that count

                                        minAbsResponseNew += abs(allKeypoints[sss][ddd][iii].at(aimedCount-1).response);

                                    }

                                    //printf("%s << DEBUG fff[%d] %d\n", __FUNCTION__, fff, 2);

                                    minAbsResponseNew /= min((unsigned int)imagesToRead, inputImages.size());

                                    //cout << "d[" << ddd << "]; fff[" << fff << "]; minAbsResponse = " << minAbsResponse << endl;

                                    vector<float> repeatabilityScores, matchabilityScores;

                                    vector<KeyPoint> pts1, pts2;

                                    int imagePairsCount = min((unsigned int)imagesToRead, inputImages.size());

                                    // For each pair of images (iii, jjj)
                                    for (int iii = 0; iii < imagePairsCount; iii++) {

                                        //printf("%s << DEBUG fff[%d] iii[%d] : %d\n", __FUNCTION__, fff, iii,  0);



                                        //printf("%s << DEBUG fff[%d] iii[%d] : %d\n", __FUNCTION__, fff, iii, 1);

                                        pts1.clear();

                                        //printf("%s << minAbsResponse = %f\n", __FUNCTION__, minAbsResponse);
                                        //printf("%s << allKeypoints.size() = %d; allKeypoints[%d].size() = %d; allKeypoints[%d][%d].size() = %d\n", __FUNCTION__, allKeypoints.size(), ddd, allKeypoints[ddd].size(), ddd, jjj, allKeypoints[ddd][jjj].size());

                                        //pts2.assign(allKeypoints[ddd][jjj].begin(), allKeypoints[ddd][jjj].begin() + fff-1);
                                        obtainSubset(allKeypoints[0][ddd][iii], pts1, minAbsResponseBase);

                                        pts2.clear();



                                        //pts1.assign(allKeypoints[ddd][iii].begin(), allKeypoints[ddd][iii].begin() + aimedCount-1);
                                        obtainSubset(allKeypoints[sss][ddd][iii], pts2, minAbsResponseNew);

                                        if (iii == 0) {
                                            currAvCount += pts1.size();
                                        }

                                        // fails on fast, at fff 250, iii 0, jjj 3

                                        //printf("%s << pts1.size() = %d; pts2.size() = %d; minAbsResponse = %f\n", __FUNCTION__, pts1.size(), pts2.size(), minAbsResponse);

                                        if (iii == 0) {
                                            currAvCount += pts2.size();
                                        }

                                        //printf("%s << DEBUG fff[%d] iii[%d] : %d\n", __FUNCTION__, fff, iii, 2);

                                        // Will need to read in up to 3 homographies!!
                                        char homogSuffix[64];
                                        sprintf(homogSuffix, "L%04dtoL%04d.xml", 0, sss);
                                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), "levels");
                                        sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                        // Read in major homography
                                        Mat H12, H12_inv, B;
                                        FileStorage fs(aD.homogAddress, FileStorage::READ);

                                        //printf("%s << aD.homogAddress = %s\n", __FUNCTION__, aD.homogAddress);

                                        fs["Homography"] >> B;
                                        fs.release();

                                        Mat X = Mat::eye(3, 3, CV_64F);
                                        Mat Y = Mat::eye(3, 3, CV_64F);
                                        Mat Xinv = Mat::eye(3, 3, CV_64F);
                                        Mat Yinv = Mat::eye(3, 3, CV_64F);
                                        Mat Binv = Mat::eye(3, 3, CV_64F);

                                        //printf("%s << DEBUG fff[%d] iii[%d] : %d\n", __FUNCTION__, fff, iii, 3);

                                        if (iii == 0) {
                                            //printf("%s << iii == 0\n", __FUNCTION__);
                                            H12 = B;
                                        } else {
                                            // Upload two other ones and multiply homographies together


                                            sprintf(homogSuffix, "H%04dtoH%04d.xml", 0, iii);

                                            sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(0).c_str());
                                            sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                            //printf("%s << aD.homogAddress1 = %s\n", __FUNCTION__, aD.homogAddress);

                                            FileStorage fs1(aD.homogAddress, FileStorage::READ);
                                            fs1["Homography"] >> X;
                                            fs1.release();

                                            //cout << "X = \n" << X << endl;

                                            sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                                            sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                            //printf("%s << aD.homogAddress2 = %s\n", __FUNCTION__, aD.homogAddress);

                                            FileStorage fs2(aD.homogAddress, FileStorage::READ);
                                            fs2["Homography"] >> Y;
                                            fs2.release();

                                            // This is kind-of guess at the moment:



                                            //printf("%s << X.size() = (%d, %d)\n", __FUNCTION__, X.rows, X.cols);

                                            invert(X, Xinv);
                                            invert(Y, Yinv);

                                            //printf("%s << About to attempt to calculate new homography...\n", __FUNCTION__);

                                            //printf("%s << Xinv = (%d, %d); B = (%d, %d); Y = (%d, %d)\n", __FUNCTION__, Xinv.rows, Xinv.cols, B.rows, B.cols, Y.rows, Y.cols);

                                            // was using H12 = Xinv * B * Y;

                                            H12 = Y * B * Xinv;

                                        }

                                        invert(H12, H12_inv);

                                        invert(B, Binv);

                                        //printf("%s << DEBUG fff[%d] iii[%d] : %d\n", __FUNCTION__, fff, iii, 4);

                                        // Determine repeatability (and sum to total)
                                        float repeatability, matchability;
                                        int correspCount;
                                        // need to get homography...
                                        //calculateRepeatability( img1, img2, H1to2, *keypoints1, *keypoints2, repeatability, correspCount );

                                        Mat thresholdedOverlapMask;

                                        calculateRepeatability( graySubsetImages.at(0).at(iii), graySubsetImages.at(sss).at(iii), H12, pts1, pts2, repeatability, correspCount, EVALUATION_OVERLAP_THRESHOLD, &thresholdedOverlapMask );

                                        if (correspCount < 0) {
                                            correspCount = 0;
                                            repeatability = 0.00;
                                            matchability = 0.00;
                                        } else {
                                            //calculateMatchability( graySubsetImages.at(0).at(iii), graySubsetImages.at(sss).at(iii), H12, pts1, pts2, matchability, correspCount, EVALUATION_OVERLAP_THRESHOLD, &thresholdedOverlapMask );
                                        }

                                        repeatability = max(repeatability, (float) 0.0);
                                        matchability = max(matchability, (float) 0.0);

                                        repeatabilityScores.push_back(repeatability);
                                        matchabilityScores.push_back(matchability);

                                        printf("%s << repeatability = %f\n", __FUNCTION__, repeatability);
                                        printf("%s << matchability = %f\n", __FUNCTION__, matchability);


                                        printf("%s << rep = %f; cCount = %d\n", __FUNCTION__, repeatability, correspCount);
                                        //cin.get();

                                        // Use H12 to test whether it relates the two images well..

                                        Mat warpedImage;

                                        if (0) {

                                        //if ((sss == 5) && (ddd == 0)) {
                                            printf("%s << Showing (0)(0).\n", __FUNCTION__);
                                            imshow("wrp", graySubsetImages.at(0).at(0));
                                            waitKey();

                                            printf("%s << Showing (0)(iii) -> (0)(0).\n", __FUNCTION__);
                                            warpPerspective(graySubsetImages.at(0).at(iii), warpedImage, Xinv, graySubsetImages.at(0).at(iii).size());
                                            imshow("wrp", warpedImage);
                                            waitKey();

                                            printf("%s << Showing (sss)(0) -> (0)(0).\n", __FUNCTION__);
                                            warpPerspective(graySubsetImages.at(sss).at(0), warpedImage, Binv, graySubsetImages.at(0).at(iii).size());
                                            imshow("wrp", warpedImage);
                                            waitKey();

                                            printf("%s << Showing (sss)(iii) -> (0)(0).\n", __FUNCTION__);
                                            Mat Q;
                                            Q = Binv * Yinv;
                                            warpPerspective(graySubsetImages.at(sss).at(iii), warpedImage, Q, graySubsetImages.at(0).at(iii).size());
                                            imshow("wrp", warpedImage);
                                            waitKey();

                                            printf("%s << Showing (sss)(iii).\n", __FUNCTION__);
                                            imshow("wrp", graySubsetImages.at(sss).at(iii));
                                            waitKey();

                                            printf("%s << Showing (0)(iii) -> (sss)(iii).\n", __FUNCTION__);
                                            warpPerspective(graySubsetImages.at(0).at(iii), warpedImage, H12, graySubsetImages.at(sss).at(iii).size());
                                            imshow("wrp", warpedImage);
                                            waitKey();

                                        }


                                    }

                                    currAvCount /= imagePairsCount;

                                    averageCounts.push_back(currAvCount);

                                    // Determine final repeatability

                                    float repeatSumm[3], matchSumm[3];    // mean, upp, low

                                    summarizeScores(repeatabilityScores, repeatSumm);
                                    summarizeScores(matchabilityScores, matchSumm);

                                    averageScores.push_back(repeatSumm[0]);
                                    upperScores.push_back(repeatSumm[1]);
                                    lowerScores.push_back(repeatSumm[2]);

                                    averageMatchScores.push_back(matchSumm[0]);
                                    upperMatchScores.push_back(matchSumm[1]);
                                    lowerMatchScores.push_back(matchSumm[2]);

                                    if ((fff % 5) == 0) {
                                        printf("%s << Average repeatability for count of [%d] = %f\n", __FUNCTION__, aimedCount, repeatSumm[0]);
                                        printf("%s << Average matchability for count of [%d] = %f\n", __FUNCTION__, aimedCount, matchSumm[0]);
                                    }

                                    //cout << "mean = " << repeatSumm[0] << "; upp = " << repeatSumm[1] << "; low = " << repeatSumm[2] << endl;

                                    //cin.get();
                                }

                                //cout << "DONE." << endl;

                                // Spit out repeatability results

                                printf("%s << Writing detector results...\n", __FUNCTION__);

                                if (averageScores.size() > 0) {
                                    FILE *file = fopen (aD.resultsAddress, "w");
                                    for (int zzz = 0; zzz < averageScores.size(); zzz++)
                                    {
                                        fprintf( file, "%d, %f, %f, %f\n", averageCounts.at(zzz), averageScores.at(zzz), upperScores.at(zzz), lowerScores.at(zzz));
                                    }

                                    fclose( file );
                                }

                                if (averageMatchScores.size() > 0) {
                                    FILE *file = fopen (aD.matchResultsAddress, "w");
                                    for (int zzz = 0; zzz < averageMatchScores.size(); zzz++)
                                    {
                                        fprintf( file, "%d, %f, %f, %f\n", averageCounts.at(zzz), averageMatchScores.at(zzz), upperMatchScores.at(zzz), lowerMatchScores.at(zzz));
                                    }

                                    fclose( file );
                                }

                                printf("%s << Detector results written.\n", __FUNCTION__);

                            }

                        }


                    }
                }

                // =====================================================================================
                //                      SAME-LEVEL REPEATABILITY
                // =====================================================================================

                printf("\n<<-----SAME-LEVEL REPEATABILITY MODULE----->>\n\n", __FUNCTION__);

                for (int sss = 0; sss < subDirectories.size(); sss++) {

                    if (subDirectories.at(sss) != "") {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.resultsPath, "%s/results/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.matchResultsPath, "%s/match_results/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());

                    } else {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.resultsPath, "%s/results/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.matchResultsPath, "%s/match_results/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                    }

                    mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);
                    mkdir(aD.matchResultsPath, DEFAULT_MKDIR_PERMISSIONS);

                    inputImages.clear();

                    countAndSortFiles(aD.imagePath, inputImages);

                    //cout << "About to perform repeatability calculations..." << endl;

                    unsigned int imagesToRead = min((unsigned int)MAX_IMAGES_PER_FOLDER, inputImages.size());

                    printf("%s << DD_DETECTORS_COUNT = %d\n", __FUNCTION__, DD_DETECTORS_COUNT);

                    //cin.get();

                    // For each detector
                    for (int ddd = 0; ddd < DD_DETECTORS_COUNT; ddd++) {

                        if (!strcmp((DD_DETECTOR_NAMES[ddd]).c_str(), "MSER")) {
                            //cin.get();
                        }



                        //cin.get();

                        // Check if results already exist, and if so skip this whole chunk!!

                        //printf("%s << DEBUG %d\n", __FUNCTION__, -99);

                        // Check if keypoints already exist
                        bool resultsAlreadyExist = true;
                        char resultsSuffix[64], matchResultsSuffix[64];

                        sprintf(resultsSuffix, "%s.csv", (DD_DETECTOR_NAMES[ddd]).c_str());
                        sprintf(aD.resultsAddress, "%s/%s", aD.resultsPath, resultsSuffix);

                        sprintf(matchResultsSuffix, "%s.csv", (DD_DETECTOR_NAMES[ddd]).c_str());
                        sprintf(aD.matchResultsAddress, "%s/%s", aD.matchResultsPath, matchResultsSuffix);

                        //printf("%s << DEBUG %d\n", __FUNCTION__, -98);

                        if (!boost::filesystem::exists(aD.resultsAddress)) {
                            resultsAlreadyExist = false;
                            // break;
                        }

                        if (!boost::filesystem::exists(aD.matchResultsAddress)) {
                            resultsAlreadyExist = false;
                        }

                        //printf("%s << DEBUG %d\n", __FUNCTION__, -97);

                        if (resultsAlreadyExist) {
                            printf("%s << Repeatability/matchability results for %s detector already exist.\n", __FUNCTION__, (DD_DETECTOR_NAMES[ddd]).c_str());
                        }

                        //cin.get();

                        //printf("%s << DEBUG %d\n", __FUNCTION__, -96);


                        if ((!resultsAlreadyExist) || (iD.regenerateDetectorResults) || (iD.regenerateAllResults)) {
                            printf("%s << Calculating repeatability for %s detector across all images...\n", __FUNCTION__, (DD_DETECTOR_NAMES[ddd]).c_str());

                            int minMaxCount = 9999;

                            //printf("%s << DEBUG %d\n", __FUNCTION__, 0);

                            for (int iii = 0; iii < imagesToRead; iii++) {
                                // figure out typical response for that count
                                int totalFeats = allKeypoints[sss][ddd][iii].size();

                                printf("%s << totalFeats (%d, %d, %d) = %d\n", __FUNCTION__, sss, ddd, iii, totalFeats);

                                if (totalFeats < minMaxCount) {
                                    minMaxCount = totalFeats;
                                }

                            }

                            printf("%s << DEBUG %d\n", __FUNCTION__, 0);

                            //printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                            vector<int> averageCounts;
                            vector<float> averageScores, averageMatchScores;
                            vector<float> upperScores, upperMatchScores;
                            vector<float> lowerScores, lowerMatchScores;

                            // For each feature count level

                            int initialCount, numberOfReps;

                            if (currentlyProfile) {
                                numberOfReps = DD_PROFILE_LEVELS;
                            } else {
                                numberOfReps = 1;

                            }

                            //printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                            //DD_PROFILE_LEVELS = 31;
                            //int DD_PROFILE_LIST[DD_PROFILE_LEVELS]

                            for (int fff = 0; fff < numberOfReps; fff++) {

                                printf("%s << DEBUG [%d], %d\n", __FUNCTION__, fff, 0);

                                int aimedCount = 0;

                                if (currentlyProfile) {
                                    aimedCount = DD_PROFILE_LIST[fff];
                                } else {
                                    aimedCount = DEFAULT_FEATURE_COUNT;
                                }

                                //printf("%s << DEBUG [%d], %d\n", __FUNCTION__, fff, 1);

                                printf("%s << aimedCount = %d; minMaxCount = %d\n", __FUNCTION__, aimedCount, minMaxCount);

                                if (aimedCount > minMaxCount) {
                                    printf("%s << Insufficient features for analysis. (%d / %d)\n", __FUNCTION__, minMaxCount, aimedCount);
                                    break;
                                }

                                //printf("%s << DEBUG [%d], %d\n", __FUNCTION__, fff, 2);


                                //printf("%s << DEBUG fff[%d] %d\n", __FUNCTION__, fff, 0);

                                int currAvCount = 0;

                                double minAbsResponse = 0.0;

                                // For each image
                                for (int iii = 0; iii < imagesToRead; iii++) {
                                    // figure out typical response for that count

                                    minAbsResponse += abs(allKeypoints[sss][ddd][iii].at(aimedCount-1).response);

                                }

                                //printf("%s << DEBUG [%d], %d\n", __FUNCTION__, fff, 3);

                                //printf("%s << DEBUG fff[%d] %d\n", __FUNCTION__, fff, 1);

                                minAbsResponse /= imagesToRead;

                                printf("%s << minAbsResponse = %f\n", __FUNCTION__, minAbsResponse);

                                //cout << "d[" << ddd << "]; fff[" << fff << "]; minAbsResponse = " << minAbsResponse << endl;

                                vector<float> repeatabilityScores, matchabilityScores, matchingScores;

                                vector<KeyPoint> pts1, pts2;

                                //printf("%s << DEBUG [%d], %d\n", __FUNCTION__, fff, 4);

                                // For each pair of images (iii, jjj)
                                for (int iii = 0; iii < imagesToRead-1; iii++) {

                                    printf("%s << DEBUG [%d][%d], %d\n", __FUNCTION__, fff, iii, 0);

                                    //printf("%s << DEBUG fff[%d] iii[%d] : %d\n", __FUNCTION__, fff, iii,  0);

                                    pts1.clear();



                                    //pts1.assign(allKeypoints[ddd][iii].begin(), allKeypoints[ddd][iii].begin() + aimedCount-1);
                                    obtainSubset(allKeypoints[sss][ddd][iii], pts1, minAbsResponse);

                                    if (iii == 0) {
                                        currAvCount += pts1.size();
                                    }

                                    //printf("%s << DEBUG fff[%d] iii[%d] : %d\n", __FUNCTION__, fff, iii, 1);

                                    printf("%s << DEBUG [%d][%d], %d\n", __FUNCTION__, fff, iii, 1);



                                    for (int jjj = iii+1; jjj < imagesToRead; jjj++) {

                                        printf("%s << DEBUG fff[%d] iii[%d] jjj[%d] : %d\n", __FUNCTION__, fff, iii, jjj, 0);




                                        pts2.clear();

                                        //printf("%s << minAbsResponse = %f\n", __FUNCTION__, minAbsResponse);
                                        //printf("%s << allKeypoints.size() = %d; allKeypoints[%d].size() = %d; allKeypoints[%d][%d].size() = %d\n", __FUNCTION__, allKeypoints.size(), ddd, allKeypoints[ddd].size(), ddd, jjj, allKeypoints[ddd][jjj].size());

                                        //pts2.assign(allKeypoints[ddd][jjj].begin(), allKeypoints[ddd][jjj].begin() + fff-1);
                                        obtainSubset(allKeypoints[sss][ddd][jjj], pts2, minAbsResponse);

                                        // fails on fast, at fff 250, iii 0, jjj 3

                                        //printf("%s << pts1.size() = %d; pts2.size() = %d; minAbsResponse = %f\n", __FUNCTION__, pts1.size(), pts2.size(), minAbsResponse);

                                        if (iii == 0) {
                                            currAvCount += pts2.size();
                                        }

                                        printf("%s << DEBUG fff[%d] iii[%d] jjj[%d] : %d\n", __FUNCTION__, fff, iii, jjj, 1);


                                        // Read in homographies
                                        char homogSuffix[64];
                                        sprintf(homogSuffix, "H%04dtoH%04d.xml", iii, jjj);
                                        sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);
                                        //cout << "homog address = " << aD.homogAddress << endl;

                                        Mat H12;
                                        FileStorage fs(aD.homogAddress, FileStorage::READ);

                                        fs["Homography"] >> H12;

                                        fs.release();

                                        //printf("%s << DEBUG fff[%d] iii[%d] jjj[%d] : %d\n", __FUNCTION__, fff, iii, jjj, 2);

                                        cout << "Homography = " << H12 << endl;


                                        // Determine repeatability (and sum to total)
                                        float repeatability, matching, matchability;
                                        int correspCount;
                                        // need to get homography...
                                        //calculateRepeatability( img1, img2, H1to2, *keypoints1, *keypoints2, repeatability, correspCount );

                                        Mat H12_inv;

                                        invert(H12, H12_inv);

                                        Mat thresholdedOverlapMask;

                                        calculateRepeatability( graySubsetImages.at(sss).at(iii), graySubsetImages.at(sss).at(jjj), H12, pts1, pts2, repeatability, correspCount, EVALUATION_OVERLAP_THRESHOLD, &thresholdedOverlapMask );

                                        //calculateMatchability( graySubsetImages.at(sss).at(iii), graySubsetImages.at(sss).at(jjj), H12, pts1, pts2, matchability, correspCount, EVALUATION_OVERLAP_THRESHOLD, &thresholdedOverlapMask );

                                        //printf("%s << DEBUG fff[%d] iii[%d] jjj[%d] : %d\n", __FUNCTION__, fff, iii, jjj, 3);

                                        // Need to correct if -1 is returned, which means no overlap at all...
                                        repeatability = max(repeatability, (float) 0.0);
                                        //cout << "repeatability = " << repeatability << endl;

                                        matchability = max(matchability, (float) 0.0);

                                        repeatabilityScores.push_back(repeatability);
                                        matchabilityScores.push_back(matchability);

                                        printf("%s << repeatability = %f\n", __FUNCTION__, repeatability);
                                        printf("%s << matchability = %f\n", __FUNCTION__, matchability);
                                        //cin.get();


                                        //printf("%s << correspondenceCount = %d\n", __FUNCTION__, calcQuality[di][ci].correspondenceCount);
                                        // calcQuality[di][ci].repeatability = rep == -1 ? rep : 100.f*rep;
                                    }

                                }

                                currAvCount /= imagesToRead;

                                averageCounts.push_back(currAvCount);

                                // Determine final repeatability

                                float repeatSumm[3], matchSumm[3];    // mean, upp, low

                                summarizeScores(repeatabilityScores, repeatSumm);
                                summarizeScores(matchabilityScores, matchSumm);

                                averageScores.push_back(repeatSumm[0]);
                                upperScores.push_back(repeatSumm[1]);
                                lowerScores.push_back(repeatSumm[2]);

                                averageMatchScores.push_back(matchSumm[0]);
                                upperMatchScores.push_back(matchSumm[1]);
                                lowerMatchScores.push_back(matchSumm[2]);

                                if ((fff % 5) == 0) {
                                    printf("%s << Average repeatability for count of [%d] = %f\n", __FUNCTION__, aimedCount, repeatSumm[0]);
                                    printf("%s << Average matchability for count of [%d] = %f\n", __FUNCTION__, aimedCount, matchSumm[0]);
                                }



                                //cout << "mean = " << repeatSumm[0] << "; upp = " << repeatSumm[1] << "; low = " << repeatSumm[2] << endl;

                                //cin.get();
                            }

                            printf("%s << Writing detector results...\n", __FUNCTION__);

                            //cout << "DONE." << endl;

                            // Spit out repeatability results

                            if (averageScores.size() > 0) {
                                FILE *file = fopen (aD.resultsAddress, "w");
                                for (int zzz = 0; zzz < averageScores.size(); zzz++)
                                {
                                    fprintf( file, "%d, %f, %f, %f\n", averageCounts.at(zzz), averageScores.at(zzz), upperScores.at(zzz), lowerScores.at(zzz));
                                }

                                fclose( file );

                            }

                            //printf("%s << Writing match results (%s)...\n", __FUNCTION__, aD.matchResultsAddress);

                            if (averageMatchScores.size() > 0) {
                                FILE *file = fopen (aD.matchResultsAddress, "w");
                                for (int zzz = 0; zzz < averageMatchScores.size(); zzz++)
                                {
                                    fprintf( file, "%d, %f, %f, %f\n", averageCounts.at(zzz), averageMatchScores.at(zzz), upperMatchScores.at(zzz), lowerMatchScores.at(zzz));
                                }

                                fclose( file );
                            }

                            printf("%s << Detector results written.\n", __FUNCTION__);

                        }


                        if (!strcmp((DD_DETECTOR_NAMES[ddd]).c_str(), "MSER")) {
                            //cin.get();
                        }


                    }

                }

                // =====================================================================================
                //                      FEATURE DESCRIPTION
                // =====================================================================================
                printf("\n<<-----FEATURE DESCRIPTION MODULE----->>\n\n");

                vector<vector<vector<Mat> > > allDescriptors, allCompDescriptors;
                vector<vector<vector<vector<KeyPoint> > > > allDescriptorKeypoints;



                for (int sss = 0; sss < subDirectories.size(); sss++) {

                    vector<vector<Mat> > dummy1;
                    allDescriptors.push_back(dummy1);
                    allCompDescriptors.push_back(dummy1);

                    vector<vector<vector<KeyPoint> > > dummy_1;
                    allDescriptorKeypoints.push_back(dummy_1);



                    if (subDirectories.at(sss) != "") {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.descriptorsPath, "%s/descriptors/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.descResultsPath, "%s/descresults/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());

                    } else {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.descriptorsPath, "%s/descriptors/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.descResultsPath, "%s/descresults/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                    }

                    // For each descriptor
                    for (int ddd = 0; ddd < DD_DESCRIPTORS_COUNT; ddd++) {

                        //Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create((DD_DESCRIPTORS_NAMES[ddd]).c_str());


                        printf("%s << Implementing descriptor %s\n", __FUNCTION__, (DD_DESCRIPTORS_NAMES[ddd]).c_str());



                        //extractor->create((DD_DESCRIPTORS_NAMES[ddd]).c_str());
                        //extractor->create("BRIEF");

                        //printf("%s << extractor created.\n", __FUNCTION__);

                        //cin.get();

                        vector<Mat> dummy2;
                        allDescriptors.at(sss).push_back(dummy2);
                        allCompDescriptors.at(sss).push_back(dummy2);

                        vector<vector<KeyPoint> > dummy_2;
                        allDescriptorKeypoints.at(sss).push_back(dummy_2);

                        bool descriptorsAlreadyExist = true;
                        char descriptorsSuffix[64], compDescriptorsSuffix[64];

                        //printf("%s << DEBUG %d\n", __FUNCTION__, 0);

                        // First check if descriptor already exists
                        for (int iii = 0; iii < graySubsetImages.at(sss).size(); iii++) {

                            sprintf(descriptorsSuffix, "%s-%04d.txt", (DD_DESCRIPTORS_NAMES[ddd]).c_str(), iii);
                            sprintf(aD.descriptorsAddress, "%s/%s", aD.descriptorsPath, descriptorsSuffix);

                            sprintf(compDescriptorsSuffix, "%s-%04d-TWARP.txt", (DD_DESCRIPTORS_NAMES[ddd]).c_str(), iii);
                            sprintf(aD.compdescriptorsAddress, "%s/%s", aD.descriptorsPath, compDescriptorsSuffix);

                            if (!boost::filesystem::exists(aD.descriptorsAddress)) {
                                descriptorsAlreadyExist = false;
                                break;
                            }
                        }



                        //printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                        char keypointsSuffix[256];
                        bool featuresAlreadyExist = true;

                        // Now must check to see if necessary (SURF) keypoints exists.
                        for (int iii = 0; iii < graySubsetImages.at(sss).size(); iii++) {

                            sprintf(keypointsSuffix, "%s-%04d.txt", (DETECTOR_FOR_PRECISION_RECALL).c_str(), iii);
                            sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, keypointsSuffix);

                            printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                            if (!boost::filesystem::exists(aD.featuresAddress)) {
                                featuresAlreadyExist = false;
                                break;
                            }
                        }

                        //printf("%s << ELEPHANT %d\n", __FUNCTION__, 0);
                        //cin.get();

                        //printf("%s << DEBUG %d\n", __FUNCTION__, 2);

                        if (featuresAlreadyExist) {

                            //printf("%s << ELEPHANT %d\n", __FUNCTION__, 1);
                            //cin.get();

                            bool descriptorAvailableInOpenCV = true;

                            if ((DD_DESCRIPTORS_NAMES[ddd] != "FREAK") && (DD_DESCRIPTORS_NAMES[ddd] != "ASURF") && (DD_DESCRIPTORS_NAMES[ddd] != "USURF") && (DD_DESCRIPTORS_NAMES[ddd] != "RSURF") && (DD_DESCRIPTORS_NAMES[ddd] != "SURF") && (DD_DESCRIPTORS_NAMES[ddd] != "SIFT") && (DD_DESCRIPTORS_NAMES[ddd] != "ORB") && (DD_DESCRIPTORS_NAMES[ddd] != "BRIEF") && (DD_DESCRIPTORS_NAMES[ddd] != "RAND")) {
                                descriptorAvailableInOpenCV = false;
                            }

                            if (((!descriptorsAlreadyExist) || (iD.regenerateDescriptors) || (iD.regenerateAllResults)) && (descriptorAvailableInOpenCV)) {

                                //cout << "Applying " << DD_DESCRIPTORS_NAMES[ddd] << " descriptor to all images... ";
                                printf("%s << Applying %s descriptor to all images.\n", __FUNCTION__, (DD_DESCRIPTORS_NAMES[ddd]).c_str());



                                //printf("%s << DEVELOPMENT EXIT.\n", __FUNCTION__);
                                //return 0;

                                //printf("%s << grayImageVec.size() = %d\n", __FUNCTION__, grayImageVec.size());

                                // For each image
                                for (int iii = 0; iii < graySubsetImages.at(sss).size(); iii++) {

                                    Mat currDescriptors, compDescriptors;

                                    //cout << ".";
                                    //cout.flush();
                                    // DD_DETECTOR_NAMES

                                    vector<KeyPoint> currPoints;

                                    // READ IN KEYPOINTS

                                    char featuresSuffix[64];

                                    sprintf(featuresSuffix, "%s-%04d.txt", (DETECTOR_FOR_PRECISION_RECALL).c_str(), iii);

                                    sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);

                                    //printf("%s << Reading features...\n", __FUNCTION__);
                                    //printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                                    //cin.get();

                                    readKeypoints_MATLAB(aD.featuresAddress, currPoints);

                                    printf("%s << currPoints.size() = %d\n", __FUNCTION__, currPoints.size());

                                    //printf("%s << keypoints read.\n", __FUNCTION__);

                                    // Reduce number of keypoints used for descriptors/descriptor experiment
                                    if (currPoints.size() > MAX_POINTS_FOR_DESCRIPTORS) {
                                        currPoints.erase(currPoints.begin()+MAX_POINTS_FOR_DESCRIPTORS, currPoints.end());
                                    }

                                    printf("%s << currPoints.size() = %d\n", __FUNCTION__, currPoints.size());


                                    //printf("%s << keypoints.size() = %d\n", __FUNCTION__, currPoints.size());

                                    for (int poi = 0; poi < currPoints.size(); poi++) {
                                        //displayKeypointInfo(currPoints.at(poi));
                                    }

                                    // MUST GENERATE DESCRIPTORS (Put them in currDescriptors)

                                    //imshow("A", grayImageVec.at(iii));
                                    //waitKey();
                                    //printf("%s << x = %d; z = (%d, %d, %d)\n", __FUNCTION__, currPoints.size(), currDescriptors.rows, currDescriptors.cols, currDescriptors.type());

                                    // This should contain only descriptors that can be implemented in C++/OpenCV:
                                    if ((DD_DESCRIPTORS_NAMES[ddd] == "FREAK") || (DD_DESCRIPTORS_NAMES[ddd] == "SURF") || (DD_DESCRIPTORS_NAMES[ddd] == "ASURF") || (DD_DESCRIPTORS_NAMES[ddd] == "SIFT") || (DD_DESCRIPTORS_NAMES[ddd] == "ORB") || (DD_DESCRIPTORS_NAMES[ddd] == "BRIEF")) {

                                        //printf("%s << Here. (%s)\n", __FUNCTION__, DD_DESCRIPTORS_NAMES[ddd].c_str());
                                        Ptr<DescriptorExtractor> extractor;


                                        vector<KeyPoint> copyPoints;
                                        copyPoints.insert(copyPoints.end(), currPoints.begin(), currPoints.end());
                                        unsigned int desired_points = currPoints.size();


                                        if (DD_DESCRIPTORS_NAMES[ddd] == "ASURF") {
                                            extractor = DescriptorExtractor::create("SURF");
                                            Mat dst;
                                            straightCLAHE(graySubsetImages.at(sss).at(iii), dst, 0.5);



                                            printf("%s << dst = (%d, %d, %d, %d)\n", __FUNCTION__, dst.rows, dst.cols, dst.depth(), dst.type());

                                            extractor->compute(dst, copyPoints, currDescriptors);

                                            printf("%s << GUH!!\n", __FUNCTION__);

                                        } else {


                                            if (DD_DESCRIPTORS_NAMES[ddd] == "SURF") {
                                                cout << "Creating SURF descriptor extractor in main()..." << endl;
                                                extractor = new SurfDescriptorExtractor(4, 2, true);
                                            } else {
                                                extractor = DescriptorExtractor::create((DD_DESCRIPTORS_NAMES[ddd]).c_str());
                                            }

                                            extractor->compute(graySubsetImages.at(sss).at(iii), copyPoints, currDescriptors);
                                        }



                                        printf("%s << extracted (%d:%d / %d) descriptors...\n", __FUNCTION__, currDescriptors.rows, currDescriptors.cols, currPoints.size());
                                        //cin.get();
                                        if (currDescriptors.rows != currPoints.size()) {

                                            printf("%s << replacing empty descriptors...\n", __FUNCTION__);
                                            // Replacing empty descriptors

                                            Mat newDescriptors = Mat::zeros(currPoints.size(), currDescriptors.cols, currDescriptors.type());

                                            srand ( time(NULL) );

                                            for (unsigned int jty = 0; jty < currPoints.size(); jty++) {
                                                vector<KeyPoint> tmpVec;
                                                tmpVec.push_back(currPoints.at(jty));
                                                Mat tmpMat;
                                                extractor->compute(graySubsetImages.at(sss).at(iii), tmpVec, tmpMat);

                                                //printf("%s << type = %d (CV_8UC1 = %d)\n", __FUNCTION__, currDescriptors.type(), CV_8UC1);

                                                if (tmpMat.rows > 0) {
                                                    printf("%s << Extracted descriptor for feature (%d) : (%d)\n", __FUNCTION__, jty, tmpMat.cols);

                                                    for (unsigned int dsa = 0; dsa < tmpMat.cols; dsa++) {
                                                        if (currDescriptors.type() == CV_8UC1) {
                                                            //printf("%s << type = CV_8UC1\n", __FUNCTION__);
                                                            newDescriptors.at<unsigned char>(jty,dsa) = tmpMat.at<unsigned char>(0,dsa);
                                                        } else if (currDescriptors.type() == CV_64FC1) {
                                                            //printf("%s << type = CV_64FC1\n", __FUNCTION__);
                                                            newDescriptors.at<double>(jty,dsa) = tmpMat.at<double>(0,dsa);
                                                            printf("%s << type = %d\n", __FUNCTION__, CV_64FC1);
                                                        } else if (currDescriptors.type() == CV_32FC1) {
                                                            //printf("%s << type = CV_64FC1\n", __FUNCTION__);
                                                            newDescriptors.at<float>(jty,dsa) = tmpMat.at<float>(0,dsa);
                                                            //printf("%s << type = %d\n", __FUNCTION__, CV_64FC1);
                                                        } else {
                                                            //printf("%s << type unknown = %d (%d)\n", __FUNCTION__, currDescriptors.type(), CV_64FC1);
                                                            //cin.get();
                                                        }

                                                        //cin.get();
                                                    }


                                                } else {
                                                    //printf("%s << Failed to extract descriptor for feature (%d)\n", __FUNCTION__, jty);

                                                    for (unsigned int dsa = 0; dsa < currDescriptors.cols; dsa++) {
                                                        if (currDescriptors.type() == CV_8UC1) {
                                                            newDescriptors.at<unsigned char>(jty,dsa) = rand() % 255;
                                                        } else if (currDescriptors.type() == CV_64FC1) {
                                                            //printf("%s << type = CV_64FC1\n", __FUNCTION__);
                                                            newDescriptors.at<double>(jty,dsa) = double(rand() % 1000) * 0.001;
                                                        } else if  (currDescriptors.type() == CV_32FC1) {
                                                            //printf("%s << type = CV_64FC1\n", __FUNCTION__);
                                                            newDescriptors.at<float>(jty,dsa) = float(rand() % 1000) * 0.001;
                                                        }
                                                    }
                                                }

                                            }

                                            currDescriptors.release();
                                            newDescriptors.copyTo(currDescriptors);

                                            //printf("%s << corrected (%d / %d) descriptors...\n", __FUNCTION__, currDescriptors.rows, currPoints.size());
                                            //cin.get();
                                        }



                                        if (iD.fusionMode) {

                                            vector<KeyPoint> fusedCopyPoints;
                                            fusedCopyPoints.insert(fusedCopyPoints.end(), currPoints.begin(), currPoints.end());
                                            unsigned int desired_points = currPoints.size();

                                            Mat warpedImage;
                                            warpedCompImages.at(iii).copyTo(warpedImage);

                                            extractor->compute(warpedImage, fusedCopyPoints, compDescriptors);
                                            printf("%s << extracted (%d:%d / %d) descriptors...\n", __FUNCTION__, compDescriptors.rows, compDescriptors.cols, currPoints.size());
                                            //cin.get();
                                            if (compDescriptors.rows < currPoints.size()) {

                                                printf("%s << replacing empty descriptors...\n", __FUNCTION__);
                                                //cin.get();
                                                // Replacing empty descriptors

                                                Mat newDescriptors = Mat::zeros(currPoints.size(), compDescriptors.cols, compDescriptors.type());

                                                for (unsigned int jty = 0; jty < currPoints.size(); jty++) {
                                                    vector<KeyPoint> tmpVec;
                                                    tmpVec.push_back(currPoints.at(jty));
                                                    Mat tmpMat;
                                                    extractor->compute(warpedImage, tmpVec, tmpMat);

                                                    //printf("%s << type = %d (CV_8UC1 = %d)\n", __FUNCTION__, compDescriptors.type(), CV_8UC1);

                                                    if (tmpMat.rows > 0) {
                                                        //printf("%s << Extracted descriptor for feature (%d) : (%d)\n", __FUNCTION__, jty, tmpMat.cols);

                                                        for (unsigned int dsa = 0; dsa < tmpMat.cols; dsa++) {
                                                            if (compDescriptors.type() == CV_8UC1) {
                                                                //printf("%s << type = CV_8UC1\n", __FUNCTION__);
                                                                newDescriptors.at<unsigned char>(jty,dsa) = tmpMat.at<unsigned char>(0,dsa);
                                                            }
                                                        }

                                                    } else {
                                                        //printf("%s << Failed to extract descriptor for feature (%d)\n", __FUNCTION__, jty);

                                                        for (unsigned int dsa = 0; dsa < compDescriptors.cols; dsa++) {
                                                            if (compDescriptors.type() == CV_8UC1) {
                                                                newDescriptors.at<unsigned char>(jty,dsa) = rand() % 255;
                                                            }
                                                        }
                                                    }

                                                }

                                                compDescriptors.release();
                                                newDescriptors.copyTo(compDescriptors);

                                                //printf("%s << corrected (%d / %d) descriptors...\n", __FUNCTION__, currDescriptors.rows, currPoints.size());
                                                //cin.get();
                                            }


                                        }

                                        //allFeatureDescriptors.at(ddd)->compute(grayImageVec.at(iii), currPoints, currDescriptors);
                                    } else if (DD_DESCRIPTORS_NAMES[ddd] == "USURF") {
                                        // Apply Upright SURF
                                        vector<KeyPoint> tmpPoints;

                                        srand ( (unsigned int)(time(NULL)) );

                                        // Remove all orientations from keypoints...
                                        for (int zaq = 0; zaq < currPoints.size(); zaq++) {
                                            tmpPoints.push_back(currPoints.at(zaq));
                                            //printf("%s << tmpPoints.at(%d).angle = %f\n", __FUNCTION__, zaq, tmpPoints.at(zaq).angle);
                                            tmpPoints.at(zaq).angle = -1;
                                            //cin.get();
                                        }

                                        Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SURF");
                                        extractor->compute(graySubsetImages.at(sss).at(iii), tmpPoints, currDescriptors);

                                    } else if (DD_DESCRIPTORS_NAMES[ddd] == "RSURF") {
                                        // Apply Upright SURF
                                        vector<KeyPoint> tmpPoints;

                                        srand ( (unsigned int)(time(NULL)) );

                                        // Remove all orientations from keypoints...
                                        for (int zaq = 0; zaq < currPoints.size(); zaq++) {
                                            tmpPoints.push_back(currPoints.at(zaq));
                                            //printf("%s << tmpPoints.at(%d).angle = %f\n", __FUNCTION__, zaq, tmpPoints.at(zaq).angle);
                                            tmpPoints.at(zaq).angle = float(rand() % 360);
                                            //printf("%s << tmpPoints.at(%d).angle = %f\n", __FUNCTION__, zaq, tmpPoints.at(zaq).angle);
                                            //cin.get();
                                        }

                                        Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SURF");
                                        extractor->compute(graySubsetImages.at(sss).at(iii), tmpPoints, currDescriptors);

                                    } else if (DD_DESCRIPTORS_NAMES[ddd] == "RAND") {
                                        // apply random description...
                                        //printf("%s << Or here..?\n", __FUNCTION__);
                                        applyRandomDescription(currPoints, currDescriptors);
                                    }

                                    printf("%s << currPoints.size() = %d; currDescriptors.rows = %d\n", __FUNCTION__, currPoints.size(), currDescriptors.rows);
                                    //cin.get();


                                    //printf("%s << descriptors extracted.\n", __FUNCTION__);

                                    // Spit out the results to a file
                                    FileStorage fs;

                                    char descriptorsSuffix[64];

                                    sprintf(descriptorsSuffix, "%s-%04d.txt", (DD_DESCRIPTORS_NAMES[ddd]).c_str(), iii);

                                    sprintf(aD.descriptorsAddress, "%s/%s", aD.descriptorsPath, descriptorsSuffix);

                                    sprintf(compDescriptorsSuffix, "%s-%04d-TWARP.txt", (DD_DESCRIPTORS_NAMES[ddd]).c_str(), iii);

                                    sprintf(aD.compdescriptorsAddress, "%s/%s", aD.descriptorsPath, compDescriptorsSuffix);

                                    printf("%s << Writing descriptors...\n", __FUNCTION__);
                                    //printf("%s << aD.descriptorsAddress = %s\n", __FUNCTION__, aD.descriptorsAddress);

                                    writeDescriptors_MATLAB(aD.descriptorsAddress, currPoints, currDescriptors);

                                    if (iD.fusionMode) {
                                        writeDescriptors_MATLAB(aD.compdescriptorsAddress, currPoints, compDescriptors);

                                        printf("%s << aD.descriptorsAddress = %s\n", __FUNCTION__, aD.descriptorsAddress);
                                        printf("%s << aD.compdescriptorsAddress = %s\n", __FUNCTION__, aD.compdescriptorsAddress);
                                        //cin.get();
                                        // aD.compdescriptorsAddress
                                    }

                                    printf("%s << descriptors written.\n", __FUNCTION__);

                                    //fs = FileStorage(aD.featuresAddress, FileStorage::WRITE);
                                    //writeKeypoints(fs, currPoints, iii);
                                    //fs.release();

                                    if (currPoints.size() < MAX_POINTS_FOR_DESCRIPTORS) {
                                        printf("%s << currPoints.size() too low (%d / %d)\n", __FUNCTION__, currPoints.size(), MAX_POINTS_FOR_DESCRIPTORS);
                                        cin.get();
                                    }

                                    allDescriptors.at(sss).at(ddd).push_back(currDescriptors);
                                    allCompDescriptors.at(sss).at(ddd).push_back(compDescriptors);
                                    allDescriptorKeypoints.at(sss).at(ddd).push_back(currPoints);

                                    //printf("%s << vector added.\n", __FUNCTION__);

                                }
                            } else {
                                // HAS TO READ IN DESCRIPTORS NOW!!!!

                                //printf("%s << about to try and read in descriptors..\n", __FUNCTION__);

                                for (int iii = 0; iii < graySubsetImages.at(sss).size(); iii++) {

                                    Mat currDescriptors;
                                    vector<KeyPoint> currentPoints;
                                    char descriptorsSuffix[64];

                                    sprintf(descriptorsSuffix, "%s-%04d.txt", (DD_DESCRIPTORS_NAMES[ddd]).c_str(), iii);

                                    sprintf(aD.descriptorsAddress, "%s/%s", aD.descriptorsPath, descriptorsSuffix);

                                    readDescriptors_MATLAB(aD.descriptorsAddress, currentPoints, currDescriptors, MAX_POINTS_FOR_DESCRIPTORS);

                                    allDescriptors.at(sss).at(ddd).push_back(currDescriptors);


                                    if (iD.fusionMode) {
                                        Mat currCompDescriptors;
                                        vector<KeyPoint> currentPoints;
                                        char compDescriptorsSuffix[64];

                                        //sprintf(compDescriptorsSuffix, "%s-%04d-TWARP.txt", (DD_DESCRIPTORS_NAMES[ddd]).c_str(), iii);
                                        //sprintf(aD.compdescriptorsAddress, "%s/%s", aD.descriptorsPath, compDescriptorsSuffix);

                                        sprintf(compDescriptorsSuffix, "%s-%04d-TWARP.txt", (DD_DESCRIPTORS_NAMES[ddd]).c_str(), iii);

                                        sprintf(aD.compdescriptorsAddress, "%s/%s", aD.descriptorsPath, compDescriptorsSuffix);

                                        //printf("%s << aD.compdescriptorsAddress = %s\n", __FUNCTION__, aD.compdescriptorsAddress);
                                        //cin.get();

                                        readDescriptors_MATLAB(aD.compdescriptorsAddress, currentPoints, currCompDescriptors, MAX_POINTS_FOR_DESCRIPTORS);

                                        allCompDescriptors.at(sss).at(ddd).push_back(currCompDescriptors);
                                    }













                                    allDescriptorKeypoints.at(sss).at(ddd).push_back(currentPoints);


                                }
                            }

                            //if (sss == 7) {
                            if (0) {
                                if ((allDescriptors.at(sss).at(ddd).at(0).rows > 0) && (allDescriptors.at(sss).at(ddd).at(1).rows > 0)) {
                                    printf("%s << Attempting initial matching for visualisation purposes...\n", __FUNCTION__);

                                    // This will actually depend on descriptor type...
                                    Ptr<DescriptorMatcher> descriptorMatcher;

                                    if (allDescriptors.at(sss).at(ddd).at(0).type() == CV_32FC1) {
                                        descriptorMatcher = DescriptorMatcher::create("BruteForce");
                                    } else {
                                        descriptorMatcher = DescriptorMatcher::create("BruteForce-Hamming");
                                    }

                                    vector<DMatch> filteredMatches;
                                    crossCheckMatching( descriptorMatcher, allDescriptors.at(sss).at(ddd).at(0), allDescriptors.at(sss).at(ddd).at(1), filteredMatches, 1 );

                                    vector<KeyPoint> keyPoints1, keyPoints2;

                                    char featuresSuffix[64];
                                    sprintf(featuresSuffix, "%s-%04d.txt", (DETECTOR_FOR_PRECISION_RECALL).c_str(), 0);
                                    sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);
                                    readKeypoints_MATLAB(aD.featuresAddress, keyPoints1);
                                    sprintf(featuresSuffix, "%s-%04d.txt", (DETECTOR_FOR_PRECISION_RECALL).c_str(), 1);
                                    sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);
                                    readKeypoints_MATLAB(aD.featuresAddress, keyPoints2);

                                    // For visualisation purposes, attempt a crude match using BruteForce
                                    Mat drawImg;
                                    int matchesFlag = DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;

                                    drawMatches( graySubsetImages.at(sss).at(0), keyPoints1, graySubsetImages.at(sss).at(1), keyPoints2, filteredMatches, drawImg, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), vector<char>(), matchesFlag );

                                    imshow("testWin", drawImg);
                                    waitKey();

                                    printf("%s << Visualisation matching complete.\n", __FUNCTION__);
                                }
                            }




                        }



                    }

                    //cout << "DONE." << endl;

                }

                printf("\n<<-----SL DESCRIPTOR EVAL MODULE----->>\n\n");


                // =====================================================================================
                //                      SAME-LEVEL DESCRIPTOR ANALYSIS
                // =====================================================================================
                for (int sss = 0; sss < subDirectories.size(); sss++) {

                    if (subDirectories.at(sss) != "") {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());

                        sprintf(aD.descriptorsPath, "%s/descriptors/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.descResultsPath, "%s/descresults/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());



                    } else {
                        sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.descriptorsPath, "%s/descriptors/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());
                        sprintf(aD.descResultsPath, "%s/descresults/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());


                    }

                    cout << "About to perform descriptor calculations..." << endl;

                    // For each descriptor
                    for (int ddd = 0; ddd < DD_DESCRIPTORS_COUNT; ddd++) {

                        // If descriptor data doesnt exists for this particular descriptor
                        if (allDescriptors.at(sss).at(ddd).size() == 0) {
                            break;
                        }

                        printf("%s << DEBUG %d: %s\n", __FUNCTION__, 0, DD_DESCRIPTORS_NAMES[ddd].c_str());

                        // Check if results already exist, and if so skip this whole chunk!!
                        // Check if keypoints already exist
                        bool resultsAlreadyExist = true;
                        char resultsSuffix[64];

                        if (0) {
                            sprintf(resultsSuffix, "%s-%s.csv", DD_DESCRIPTORS_NAMES[ddd].c_str(), "NND");
                        } else {
                            sprintf(resultsSuffix, "%s.csv", DD_DESCRIPTORS_NAMES[ddd].c_str());
                        }


                        sprintf(aD.resultsAddress, "%s/%s", aD.descResultsPath, resultsSuffix);

                        printf("%s << aD.resultsAddress = %s\n", __FUNCTION__, aD.resultsAddress);

                        if (iD.fusionMode) {
                            char compResultsSuffix[64];
                            sprintf(compResultsSuffix, "%s-%s-%f.csv", DD_DESCRIPTORS_NAMES[ddd].c_str(), "TWARP", WEIGHTING);
                            sprintf(aD.compResultsAddress, "%s/%s", aD.descResultsPath, compResultsSuffix);
                        }

                        if (!boost::filesystem::exists(aD.resultsAddress)) {
                            resultsAlreadyExist = false;
                        }

                        printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                        if (resultsAlreadyExist) {
                            printf("%s << Precision-recall results for descriptor [%s] already exist.\n", __FUNCTION__, DD_DESCRIPTORS_NAMES[ddd].c_str());
                        }

                        printf("%s << About to check if needs to be calculated...\n", __FUNCTION__);

                        if ((!resultsAlreadyExist) || (iD.regenerateDescriptorResults) || (iD.regenerateAllResults)) {
                            cout << "Calculating precision-recall for " << DD_DESCRIPTORS_NAMES[ddd] << " descriptor across all images... ";

                            // For each pair of images, call a function which returns a 2x64 matrix of recall vs (1-precision) results

                            int pairsTested = 0;

                            vector<vector<Point2f> > totalPrecRecall, totalFusionPrecRecall; // hopefully initializes at zero!

                            vector<KeyPoint> pts1, pts2;

                            bool featuresAvailable = true;

                            if (currentlyProfile) {
                                printMatches = true;
                            } else {
                                printMatches = false;

                            }

                            for (int iii = 0; iii < min((int)graySubsetImages.at(sss).size(), MAX_IMAGES_FOR_SINGLE_DESCRIPTOR_TEST)-1; iii++) {

                                printf("%s << DEBUG %d\n", __FUNCTION__, -1);

                                pts1.clear();

                                char featuresSuffix[64];
                                sprintf(featuresSuffix, "%s-%04d.txt", (DETECTOR_FOR_PRECISION_RECALL).c_str(), iii);

                                sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);

                                if (!boost::filesystem::exists(aD.featuresAddress)) {
                                    printf("%s << Features unavailable...\n", __FUNCTION__);
                                    cin.get();
                                    featuresAvailable = false;
                                    break;
                                }

                                printf("%s << Reading features (1)...\n", __FUNCTION__);
                                printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                                readKeypoints_MATLAB(aD.featuresAddress, pts1);





                                printf("%s << Features now read.\n", __FUNCTION__);

                                if (pts1.size() > MAX_POINTS_FOR_DESCRIPTORS) {
                                    pts1.erase(pts1.begin()+MAX_POINTS_FOR_DESCRIPTORS, pts1.end());
                                }

                                printf("%s << pts1.size() = %d\n", __FUNCTION__, pts1.size());

                                if (pts1.size() == 0) {
                                    featuresAvailable = false;
                                    printf("%s << ERROR! Insufficient keypoints, returning...\n", __FUNCTION__);
                                    return -1;
                                }

                                printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                                for (int jjj = iii+1; jjj < min((int)graySubsetImages.at(sss).size(), MAX_IMAGES_FOR_SINGLE_DESCRIPTOR_TEST); jjj++) {

                                    printf("%s << DEBUG[%d] %d\n", __FUNCTION__, jjj, 0);

                                    pts2.clear();

                                    pairsTested++;

                                    // Read in homographies
                                    char homogSuffix[64];
                                    sprintf(homogSuffix, "H%04dtoH%04d.xml", iii, jjj);
                                    sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);
                                    //cout << "homog address = " << aD.homogAddress << endl;

                                    Mat H12;
                                    FileStorage fs(aD.homogAddress, FileStorage::READ);

                                    fs["Homography"] >> H12;

                                    fs.release();

                                    printf("%s << DEBUG[%d] %d\n", __FUNCTION__, jjj, 1);

                                    vector<Point2f> precRecall;
                                     vector<Point2f> fusionPrecRecall;
                                    // READ IN KEYPOINTS

                                    sprintf(featuresSuffix, "%s-%04d.txt", (DETECTOR_FOR_PRECISION_RECALL).c_str(), jjj);

                                    sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);

                                    if (!boost::filesystem::exists(aD.featuresAddress)) {
                                        featuresAvailable = false;
                                        printf("%s << ERROR! Features unavailable, returning...\n", __FUNCTION__);
                                        return -1;
                                    }

                                    printf("%s << DEBUG[%d] %d\n", __FUNCTION__, jjj, 2);

                                    //printf("%s << Reading features...\n", __FUNCTION__);
                                    //printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                                    readKeypoints_MATLAB(aD.featuresAddress, pts2);



                                    //printf("%s << Features2 now read.\n", __FUNCTION__);

                                    //printf("%s << pts2.size() = %d; MPFD = %d\n", __FUNCTION__, pts2.size(), MAX_POINTS_FOR_DESCRIPTORS);

                                    if (pts2.size() > MAX_POINTS_FOR_DESCRIPTORS) {
                                        pts2.erase(pts2.begin()+MAX_POINTS_FOR_DESCRIPTORS, pts2.end());
                                    }

                                    printf("%s << pts2[%d].size() = %d\n", __FUNCTION__, jjj, pts2.size());

                                    if (pts2.size() == 0) {
                                        featuresAvailable = false;
                                        printf("%s << Insufficient keypoints...\n", __FUNCTION__);
                                        cin.get();
                                        break;
                                    }

                                    printf("%s << DEBUG[%d] %d\n", __FUNCTION__, jjj, 4);

                                    //printf("%s << allDescriptors.size() #1 = %d\n", __FUNCTION__, allDescriptors.size());
                                    //printf("%s << allDescriptors.size() #2 = %d\n", __FUNCTION__, allDescriptors.at(sss).size());
                                    //printf("%s << allDescriptors.size() #3 = %d\n", __FUNCTION__, allDescriptors.at(sss).at(ddd).size());

                                    //printf("%s << desc1.size() = (%d, %d)\n", __FUNCTION__, allDescriptors.at(sss).at(ddd).at(iii).rows, allDescriptors.at(sss).at(ddd).at(iii).cols);
                                    //printf("%s << desc2.size() = (%d, %d)\n", __FUNCTION__, allDescriptors.at(sss).at(ddd).at(jjj).rows, allDescriptors.at(sss).at(ddd).at(jjj).cols);

                                    if ((allDescriptors.at(sss).at(ddd).at(iii).rows > 0) && (allDescriptors.at(sss).at(ddd).at(jjj).rows > 0)) {

                                        float repeatability;
                                        int correspCount;

                                            int fusionCorrespCount;
                                        calculateRepeatability( graySubsetImages.at(sss).at(iii), graySubsetImages.at(sss).at(jjj), H12, pts1, pts2, repeatability, correspCount, EVALUATION_OVERLAP_THRESHOLD );

                                        if (0) {
                                        //if ((sss == 5) && (ddd == 0)) {

                                            Mat desc1, desc2;

                                            printf("%s << Attempting initial matching for visualisation purposes...\n", __FUNCTION__);

                                            // This will actually depend on descriptor type...
                                            Ptr<DescriptorMatcher> descriptorMatcher;

                                            if (allDescriptors.at(sss).at(ddd).at(0).type() == CV_32FC1) {
                                                descriptorMatcher = DescriptorMatcher::create("BruteForce");
                                            } else {
                                                descriptorMatcher = DescriptorMatcher::create("BruteForce-Hamming");
                                            }

                                            vector<DMatch> filteredMatches;
                                            crossCheckMatching( descriptorMatcher, allDescriptors.at(sss).at(ddd).at(iii), allDescriptors.at(sss).at(ddd).at(jjj), filteredMatches, 1 );


                                            // For visualisation purposes, attempt a crude match using BruteForce
                                            Mat drawImg;
                                            int matchesFlag = DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;

                                            drawMatches( graySubsetImages.at(sss).at(iii), pts1, graySubsetImages.at(sss).at(jjj), pts2, filteredMatches, drawImg, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), vector<char>(), matchesFlag );

                                            printf("%s << filteredMatches.size() = %d; (correspCount = %d)\n", __FUNCTION__, filteredMatches.size(), correspCount);

                                            imshow("testWin", drawImg);
                                            waitKey( 0 );

                                            printf("%s << Visualisation matching complete.\n", __FUNCTION__);

                                        }


                                        printf("%s << %d, %d, %d, %d\n", __FUNCTION__, allDescriptorKeypoints.at(sss).at(ddd).at(iii).size(), allDescriptorKeypoints.at(sss).at(ddd).at(jjj).size(), allDescriptors.at(sss).at(ddd).at(iii).rows, allDescriptors.at(sss).at(ddd).at(jjj).rows);

                                        printf("%s << About to calculate PR...\n", __FUNCTION__);

                                        cout << "H12 = " << H12 << endl;

                                        printf("%s << (%d, %d) / (%d, %d)\n", __FUNCTION__, graySubsetImages.at(sss).at(iii).rows, graySubsetImages.at(sss).at(iii).cols, graySubsetImages.at(sss).at(jjj).rows, graySubsetImages.at(sss).at(jjj).cols);


                                        for (int abc = 0; abc < allDescriptorKeypoints.at(sss).at(ddd).at(iii).size(); abc++) {
                                            if (allDescriptorKeypoints.at(sss).at(ddd).at(iii).at(abc).size == 0.0) {
                                                printf("%s << radius(1)(%d) == 0!\n", __FUNCTION__, abc);
                                                cin.get();
                                            }
                                        }

                                        for (int abc = 0; abc < allDescriptorKeypoints.at(sss).at(ddd).at(jjj).size(); abc++) {
                                            if (allDescriptorKeypoints.at(sss).at(ddd).at(jjj).at(abc).size == 0.0) {
                                                printf("%s << radius(2)(%d) == 0!\n", __FUNCTION__, abc);
                                                cin.get();
                                            }
                                        }

                                        // Assign matches filename
                                        sprintf(matchesFilename, "%s/%s-%s-%s-%d-%d.txt", matchesParent.c_str(), fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), iii, jjj);

                                        if (printMatches) {
                                            printf("%s << Printing matches to: %s\n", __FUNCTION__, matchesFilename);
                                        } else {
                                            printf("%s << Not printing matches to: %s\n", __FUNCTION__, matchesFilename);
                                            //cin.get();
                                        }

                                        printf("%s << APproaching..\n", __FUNCTION__);
                                        printf("%s << pts1.size() = %d; pts2.size() = %d\n", __FUNCTION__, allDescriptorKeypoints.at(sss).at(ddd).at(iii).size(), allDescriptorKeypoints.at(sss).at(ddd).at(jjj).size());

                                        //calculatePrecisionRecall( grayImageVec.at(iii), grayImageVec.at(jjj), H12, pts1, pts2, allDescriptors.at(sss).at(ddd).at(iii), allDescriptors.at(sss).at(ddd).at(jjj), precRecall, EVALUATION_OVERLAP_THRESHOLD );
                                        calculatePrecisionRecall( graySubsetImages.at(sss).at(iii),
                                                                 graySubsetImages.at(sss).at(jjj),
                                                                 H12,
                                                                 allDescriptorKeypoints.at(sss).at(ddd).at(iii),
                                                                 allDescriptorKeypoints.at(sss).at(ddd).at(jjj),
                                                                 allDescriptors.at(sss).at(ddd).at(iii),
                                                                 allDescriptors.at(sss).at(ddd).at(jjj),
                                                                 precRecall,
                                                                 correspCount,
                                                                 EVALUATION_OVERLAP_THRESHOLD );

                                        if (iD.fusionMode) {

                                            printf("%s << About to calculate entropy...\n", __FUNCTION__);

                                            double visEntropy = calculateEntropy(graySubsetImages.at(sss).at(iii), graySubsetImages.at(sss).at(jjj), allDescriptorKeypoints.at(sss).at(ddd).at(iii), allDescriptorKeypoints.at(sss).at(ddd).at(jjj));
                                            double thermEntropy = calculateEntropy(warpedCompImages.at(iii), warpedCompImages.at(jjj), allDescriptorKeypoints.at(sss).at(ddd).at(iii), allDescriptorKeypoints.at(sss).at(ddd).at(jjj));
                                            double entropy = visEntropy / thermEntropy;

                                            printf("%s << visEntropy = (%f); thermEntropy = (%f); entropy = (%f)\n", __FUNCTION__, visEntropy, thermEntropy, entropy);
                                            //cin.get();
                                            printf("%s << About to fuse: (%d, %d, %d, %d, %d) / (%d, %d)\n", __FUNCTION__, graySubsetImages.at(sss).size(), warpedCompImages.size(), allDescriptorKeypoints.at(sss).at(ddd).size(), allDescriptors.at(sss).at(ddd).size(), allCompDescriptors.at(sss).at(ddd).size(), iii, jjj);

                                            if (0) {
                                                cScheme colorization;
                                                colorization.load_standard(100, 1); //
                                                double params_x[2] = { 0.2, 0.8 };

                                                Mat fusedIm, showIm;

                                                colorization.fuse_image(warpedCompImages.at(iii), graySubsetImages.at(sss).at(iii), fusedIm, params_x);

                                                vector<KeyPoint> kpSubset;

                                                unsigned int kpAdded = 0;
                                                unsigned int addIndex = 0;

                                                while (kpAdded < 50) {
                                                    if (allDescriptorKeypoints.at(sss).at(ddd).at(iii).at(addIndex).pt.y > 100.0) {
                                                        kpSubset.push_back(allDescriptorKeypoints.at(sss).at(ddd).at(iii).at(addIndex));
                                                        kpAdded++;
                                                    }
                                                    addIndex++;
                                                }

                                                displayKeypoints(fusedIm, kpSubset, showIm, cv::Scalar(0,0,0), 1);

                                                imwrite("/home/steve/Desktop/sample_im.jpg", showIm);

                                                imshow("fusedIm", showIm);
                                                waitKey(0);
                                            }

                                            calculateFusionPR(graySubsetImages.at(sss).at(iii),
                                                              graySubsetImages.at(sss).at(jjj),
                                                              warpedCompImages.at(iii),
                                                              warpedCompImages.at(jjj),
                                                              H12,
                                                              allDescriptorKeypoints.at(sss).at(ddd).at(iii),
                                                              allDescriptorKeypoints.at(sss).at(ddd).at(jjj),
                                                              allDescriptors.at(sss).at(ddd).at(iii),
                                                              allDescriptors.at(sss).at(ddd).at(jjj),
                                                              allCompDescriptors.at(sss).at(ddd).at(iii),
                                                              allCompDescriptors.at(sss).at(ddd).at(jjj),
                                                              fusionPrecRecall,
                                                              fusionCorrespCount,
                                                              EVALUATION_OVERLAP_THRESHOLD,
                                                              WEIGHTING,
                                                              entropy  );

                                              smoothAndSubsamplePrecisionRecallPoints(fusionPrecRecall, 0.001);

                                            double fusionPrScore = calculatePrecisionRecallScore(fusionPrecRecall);

                                            printf("%s << fusionPrecRecall.size() = %d\n", __FUNCTION__, fusionPrecRecall.size());

                                            printf("%s << fusionPrScore = %f\n", __FUNCTION__, fusionPrScore);


                                        }

                                        smoothAndSubsamplePrecisionRecallPoints(precRecall, 0.001);

                                        double prScore = calculatePrecisionRecallScore(precRecall);

                                        printf("%s << Calculation complete. PR-Score = %f\n", __FUNCTION__, prScore);

                                        //cin.get();

                                        //cin.get();

                                        if ((sss == 5) && (ddd == 0)) {
                                            // cin.get();
                                        }
                                    }

                                    printf("%s << DEBUG[%d] %d\n", __FUNCTION__, jjj, 5);


                                    //printf("%s << results (precRecall.size() = %d):\n", __FUNCTION__, precRecall.size());

                                    for (int qwe = 0; qwe < precRecall.size(); qwe++) {
                                        //printf("%s << (%d) = (%f, %f)\n", __FUNCTION__, qwe, precRecall.at(qwe).x, precRecall.at(qwe).y);
                                    }

                                    //cin.get();

                                    // might want to check to make sure all vectors are the same lengths...
                                    totalPrecRecall.push_back(precRecall);
                                    totalFusionPrecRecall.push_back(fusionPrecRecall);

                                }

                                printf("%s << DEBUG[%d] X6\n", __FUNCTION__, 6);
                            }

                            printf("%s << DEBUG[%d] X7\n", __FUNCTION__, 7);

                            if (featuresAvailable) {

                                printf("%s << DEBUG[%d] X7a\n", __FUNCTION__, 7);


                                // if vectors are not all corresponding / the same lengths, might want to have a more sophisticated method
                                // for coming up with an "averaged" result
                                vector<Point2f> finalRecall, finalFusionRecall;
                                combineRecallVectors(totalPrecRecall, finalRecall);

                                combineRecallVectors(totalFusionPrecRecall, finalFusionRecall);

                                // Spit out prec/recall results

                                printf("%s << writing out descriptor recall results...\n", __FUNCTION__);

                                printf("%s << writing out descriptor recall results...\n", __FUNCTION__);
                                printf("%s << aD.resultsAddress = %s\n", __FUNCTION__, aD.resultsAddress);
                                printf("%s << aD.compResultsAddress = %s\n", __FUNCTION__, aD.compResultsAddress);

                                if (finalRecall.size() > 0) {
                                    writeDescriptorResults(aD.resultsAddress, finalRecall);

                                    if (iD.fusionMode) {
                                        writeDescriptorResults(aD.compResultsAddress, finalFusionRecall);
                                    }
                                }

                                //cin.get();



                                //cout << "DONE." << endl;
                            }

                            printf("%s << DEBUG[%d] X8\n", __FUNCTION__, 7);


                        }

                        printf("%s << DEBUG[%d] X8\n", __FUNCTION__, 8);

                    }

                    printf("%s << DEBUG[%d] X9\n", __FUNCTION__, 9);

                }

                printf("\n<<-----BR DESCRIPTOR EVAL MODULE----->>\n\n");

                // =====================================================================================
                //                      BASE-REF DESCRIPTOR ANALYSIS
                // =====================================================================================
                if (fD.subsets.at(subsetCounter) != "profile") {

                    // For base image, define it and other paths
                    vector<string> baseImages, inputImages;

                    sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(0).c_str());

                    countAndSortFiles(aD.imagePath, baseImages);

                    printf("%s << aD.imagePath = %s\n", __FUNCTION__, aD.imagePath);
                    printf("%s << baseImages.size() = %d\n", __FUNCTION__, baseImages.size());

                    cout << "Processing images... (" << inputImages.size() << ") ";

                    sprintf(aD.baseFeaturesPath, "%s/keypoints/%s/%s/%s/0000", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str());


                    sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), "levels");

                    for (int sss = 1; sss < subDirectories.size(); sss++) {

                        sprintf(aD.imagePath, "%s/images/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.featuresPath, "%s/keypoints/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.descriptorsPath, "%s/descriptors/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                        sprintf(aD.descResultsPath, "%s/descresults/%s/%s/%s/%s-BR", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());

                        mkdir(aD.descResultsPath, DEFAULT_MKDIR_PERMISSIONS);


                        inputImages.clear();
                        countAndSortFiles(aD.imagePath, inputImages);

                        cout << "About to perform descriptor calculations..." << endl;

                        // For each descriptor
                        for (int ddd = 0; ddd < DD_DESCRIPTORS_COUNT; ddd++) {

                            printf("%s << DEBUG %d: %s\n", __FUNCTION__, 0, DD_DESCRIPTORS_NAMES[ddd].c_str());

                            // Check if results already exist, and if so skip this whole chunk!!
                            // Check if keypoints already exist
                            bool resultsAlreadyExist = true;
                            char resultsSuffix[64];

                            sprintf(resultsSuffix, "%s.csv", DD_DESCRIPTORS_NAMES[ddd].c_str());
                            sprintf(aD.resultsAddress, "%s/%s", aD.descResultsPath, resultsSuffix);

                            printf("%s << aD.resultsAddress = %s\n", __FUNCTION__, aD.resultsAddress);

                            if (!boost::filesystem::exists(aD.resultsAddress)) {
                                resultsAlreadyExist = false;
                            }

                            printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                            if (resultsAlreadyExist) {
                                printf("%s << Precision-recall results for detector [%d] already exist.\n", __FUNCTION__, ddd);
                            }

                            printf("%s << About to check if needs to be calculated...\n", __FUNCTION__);

                            if ((!resultsAlreadyExist) || (iD.regenerateDescriptorResults) || (iD.regenerateAllResults)) {
                                cout << "Calculating (base-ref) precision-recall for " << DD_DESCRIPTORS_NAMES[ddd] << " descriptor across all images... ";

                                // For each pair of images, call a function which returns a 2x64 matrix of recall vs (1-precision) results

                                int pairsTested = 0;

                                vector<vector<Point2f> > totalPrecRecall; // hopefully initializes at zero!



                                int numberOfImages;

                                numberOfImages = min((int)inputImages.size(), (int)baseImages.size());
                                numberOfImages = min(numberOfImages, MAX_IMAGES_FOR_SINGLE_DESCRIPTOR_TEST-1);

                                // Will need to read in up to 3 homographies!!
                                char homogSuffix[64];
                                sprintf(homogSuffix, "L%04dtoL%04d.xml", 0, sss);
                                sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), "levels");
                                sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                // Read in major homography
                                Mat H12, B, H12_inv;
                                FileStorage fs(aD.homogAddress, FileStorage::READ);

                                printf("%s << aD.homogAddress = %s\n", __FUNCTION__, aD.homogAddress);

                                fs["Homography"] >> B;
                                fs.release();



                                bool featuresAvailable = true;

                                for (int iii = 0; iii < numberOfImages; iii++) {

                                    vector<KeyPoint> pts1, pts2;

                                    char featuresSuffix[64];
                                    sprintf(featuresSuffix, "%s-%04d.txt", (DETECTOR_FOR_PRECISION_RECALL).c_str(), iii);

                                    // ===== READ IN FEATURES FROM A PAIR OF IMAGES


                                    sprintf(aD.featuresAddress, "%s/%s", aD.baseFeaturesPath, featuresSuffix);

                                    if (!boost::filesystem::exists(aD.featuresAddress)) {
                                        featuresAvailable = false;
                                        break;
                                    }


                                    printf("%s << Reading features (1)...\n", __FUNCTION__);
                                    printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                                    readKeypoints_MATLAB(aD.featuresAddress, pts1);

                                    printf("%s << features.size() = %d\n", __FUNCTION__, pts1.size());

                                    if (pts1.size() == 0) {
                                        featuresAvailable = false;
                                        break;
                                    }

                                    if (pts1.size() > MAX_POINTS_FOR_DESCRIPTORS) {
                                        pts1.erase(pts1.begin()+MAX_POINTS_FOR_DESCRIPTORS, pts1.end());
                                    }

                                    sprintf(aD.featuresAddress, "%s/%s", aD.featuresPath, featuresSuffix);

                                    if (!boost::filesystem::exists(aD.featuresAddress)) {
                                        featuresAvailable = false;
                                        break;
                                    }

                                    printf("%s << Reading features (2)...\n", __FUNCTION__);
                                    printf("%s << aD.featuresAddress = %s\n", __FUNCTION__, aD.featuresAddress);

                                    readKeypoints_MATLAB(aD.featuresAddress, pts2);

                                    printf("%s << features.size() = %d\n", __FUNCTION__, pts2.size());

                                    if (pts2.size() == 0) {
                                        featuresAvailable = false;
                                        break;
                                    }

                                    if (pts2.size() > MAX_POINTS_FOR_DESCRIPTORS) {
                                        pts2.erase(pts2.begin()+MAX_POINTS_FOR_DESCRIPTORS, pts2.end());
                                    }

                                    printf("%s << Features2 now read.\n", __FUNCTION__);

                                    printf("%s << pts2.size() = %d; MPFD = %d\n", __FUNCTION__, pts2.size(), MAX_POINTS_FOR_DESCRIPTORS);

                                    printf("%s << Features now read.\n", __FUNCTION__);

                                    // ===== DETERMINE HOMOGRAPHY RELATING IMAGES
                                    if (iii == 0) {
                                        printf("%s << iii == 0\n", __FUNCTION__);
                                        H12 = B;
                                    } else {
                                        // Upload two other ones and multiply homographies together
                                        Mat X = Mat::eye(3, 3, CV_64F);
                                        Mat Y = Mat::eye(3, 3, CV_64F);

                                        sprintf(homogSuffix, "H%04dtoH%04d.xml", 0, iii);

                                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(0).c_str());
                                        sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                        printf("%s << aD.homogAddress1 = %s\n", __FUNCTION__, aD.homogAddress);

                                        FileStorage fs1(aD.homogAddress, FileStorage::READ);
                                        fs1["Homography"] >> X;
                                        fs1.release();

                                        //cout << "X = \n" << X << endl;

                                        sprintf(aD.homogPath, "%s/homographies/%s/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), fD.modalities.at(modalityCounter).c_str(), fD.subsets.at(subsetCounter).c_str(), subDirectories.at(sss).c_str());
                                        sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                                        printf("%s << aD.homogAddress2 = %s\n", __FUNCTION__, aD.homogAddress);

                                        FileStorage fs2(aD.homogAddress, FileStorage::READ);
                                        fs2["Homography"] >> Y;
                                        fs2.release();

                                        // This is kind-of guess at the moment:

                                        Mat Xinv, Yinv;

                                        //printf("%s << X.size() = (%d, %d)\n", __FUNCTION__, X.rows, X.cols);

                                        invert(X, Xinv);
                                        invert(Y, Yinv);

                                        printf("%s << About to attempt to calculate new homography...\n", __FUNCTION__);

                                        printf("%s << Xinv = (%d, %d); B = (%d, %d); Y = (%d, %d)\n", __FUNCTION__, Xinv.rows, Xinv.cols, B.rows, B.cols, Y.rows, Y.cols);

                                        H12 = Y * B * Xinv;

                                    }

                                    invert(H12, H12_inv);


                                    pairsTested++;
                                    vector<Point2f> precRecall;

                                    printf("%s << About to calculate precision recall...\n", __FUNCTION__);

                                    printf("%s << allDescriptors.size() = %d\n", __FUNCTION__, allDescriptors.size());
                                    printf("%s << allDescriptors.at(%d).size() = %d\n", __FUNCTION__, sss, allDescriptors.at(sss).size());
                                    printf("%s << allDescriptors.at(%d).at(%d).size() = %d\n", __FUNCTION__, sss, ddd, allDescriptors.at(sss).at(ddd).size());

                                    //printf("%s << desc1.size() = (%d, %d)\n", __FUNCTION__, allDescriptors.at(sss).at(ddd).at(0).rows, allDescriptors.at(sss).at(ddd).at(0).cols);
                                    //printf("%s << desc2.size() = (%d, %d)\n", __FUNCTION__, allDescriptors.at(sss).at(ddd).at(iii).rows, allDescriptors.at(sss).at(ddd).at(iii).cols);

                                    if ((allDescriptors.at(0).at(ddd).size() != 0) && (allDescriptors.at(sss).at(ddd).size() != 0)) {

                                        // check homography...

                                        if (0) {
                                        //if ((sss == 5) && (ddd == 0)) {
                                            printf("%s << Showing original image #2\n", __FUNCTION__);
                                            imshow("wrp", graySubsetImages.at(sss).at(iii));
                                            waitKey();
                                            Mat warpedImage;
                                            warpPerspective(graySubsetImages.at(0).at(iii), warpedImage, H12, graySubsetImages.at(0).at(iii).size());
                                            printf("%s << Showing warped image #1\n", __FUNCTION__);
                                            imshow("wrp", warpedImage);
                                            waitKey();
                                        }

                                        printf("%s << pts1.size() = %d; pts2.size() = %d; desc1.size() = %d; desc2.size() = %d\n", __FUNCTION__, pts1.size(), pts2.size(), allDescriptors.at(0).at(ddd).at(iii).rows, allDescriptors.at(sss).at(ddd).at(iii).rows);

                                        //if (sss == 2) {
                                        //if (ddd == 0) {
                                        if (0) { // if ((sss == 9) && (ddd == 0)) {

                                            printf("%s << Attempting initial matching for visualisation purposes...\n", __FUNCTION__);

                                            // This will actually depend on descriptor type...
                                            Ptr<DescriptorMatcher> descriptorMatcher;

                                            if (allDescriptors.at(sss).at(ddd).at(0).type() == CV_32FC1) {
                                                descriptorMatcher = DescriptorMatcher::create("BruteForce");
                                            } else {
                                                descriptorMatcher = DescriptorMatcher::create("BruteForce-Hamming");
                                            }

                                            vector<DMatch> filteredMatches;
                                            crossCheckMatching( descriptorMatcher, allDescriptors.at(0).at(ddd).at(iii), allDescriptors.at(sss).at(ddd).at(iii), filteredMatches, 1 );

                                            vector<uchar> validityMask;

                                            vector<KeyPoint> sPts1, sPts2;
                                            vector<Point2f> vPts1, vPts2;

                                            if (filteredMatches.size() < 4) {
                                                printf("%s << ERROR! Insufficient matches (%d)\n", __FUNCTION__, filteredMatches.size());
                                            }

                                            for (unsigned int tre = 0; tre < filteredMatches.size(); tre++) {
                                                sPts1.push_back(pts1.at(filteredMatches.at(tre).queryIdx));
                                                vPts1.push_back(sPts1.at(tre).pt);
                                                sPts2.push_back(pts2.at(filteredMatches.at(tre).trainIdx));
                                                vPts2.push_back(sPts2.at(tre).pt);
                                            }

                                            printf("%s << (%d)(%d)(%d)\n", __FUNCTION__, filteredMatches.size(), sPts1.size(), sPts2.size());

                                            Mat H12X = findHomography( Mat(vPts1), Mat(vPts2), validityMask, CV_RANSAC, 16.0 );

                                            printf("%s << Homography calculated.\n", __FUNCTION__);

                                            for (int tre = validityMask.size()-1; tre >= 0; tre--) {
                                                if (validityMask.at(tre) == 0) {
                                                    sPts1.erase(sPts1.begin() + tre);
                                                    sPts2.erase(sPts2.begin() + tre);
                                                }
                                            }

                                            printf("%s << Points reduced.\n", __FUNCTION__);

                                            vector<DMatch> newMatches;

                                            for (unsigned int tre = 0; tre < sPts1.size(); tre++) {
                                                DMatch alpha__(tre, tre, 0, (double(tre + 1)));
                                                newMatches.push_back(alpha__);
                                            }

                                            // For visualisation purposes, attempt a crude match using BruteForce
                                            Mat drawImg;
                                            int matchesFlag = DrawMatchesFlags::DRAW_RICH_KEYPOINTS; // DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;

                                            //drawMatches( graySubsetImages.at(0).at(iii), pts1, graySubsetImages.at(sss).at(iii), pts2, filteredMatches, drawImg, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), vector<char>(), matchesFlag );

                                            displayMatchesNeatly(graySubsetImages.at(0).at(iii), graySubsetImages.at(sss).at(iii), sPts1, sPts2, newMatches, drawImg);

                                            printf("%s << filteredMatches.size() = %d\n", __FUNCTION__, filteredMatches.size());

                                            imwrite("/home/steve/Desktop/sample_im.png", drawImg);

                                            imshow("testWin", drawImg);
                                            waitKey( 0 );

                                            printf("%s << Visualisation matching complete.\n", __FUNCTION__);

                                        }

                                        // break; // hmmm, not sure about this...

                                        if ((allDescriptors.at(0).at(ddd).at(iii).rows > 0) && (allDescriptors.at(sss).at(ddd).at(iii).rows > 0)) {
                                            Mat eyeMat = Mat::eye(3,3, CV_64F);

                                            float repeatability;
                                            int correspCount;
                                            calculateRepeatability( graySubsetImages.at(0).at(iii), graySubsetImages.at(sss).at(iii), H12, pts1, pts2, repeatability, correspCount, EVALUATION_OVERLAP_THRESHOLD );

                                            //calculatePrecisionRecall( grayImageVec.at(0), grayImageVec.at(iii), H12_inv, pts1, pts2, allDescriptors.at(sss).at(ddd).at(0), allDescriptors.at(sss).at(ddd).at(iii), precRecall, EVALUATION_OVERLAP_THRESHOLD );
                                            //calculatePrecisionRecall( graySubsetImages.at(0).at(0), graySubsetImages.at(sss).at(iii), H12_inv, pts1, pts2, allDescriptors.at(sss).at(ddd).at(0), allDescriptors.at(sss).at(ddd).at(iii), precRecall, EVALUATION_OVERLAP_THRESHOLD );
                                            //calculatePrecisionRecall( graySubsetImages.at(0).at(iii), graySubsetImages.at(sss).at(iii), H12, pts1, pts2, allDescriptors.at(0).at(ddd).at(iii), allDescriptors.at(sss).at(ddd).at(iii), precRecall, EVALUATION_OVERLAP_THRESHOLD );
                                            calculatePrecisionRecall( graySubsetImages.at(0).at(iii), graySubsetImages.at(sss).at(iii), H12, allDescriptorKeypoints.at(0).at(ddd).at(iii), allDescriptorKeypoints.at(sss).at(ddd).at(iii), allDescriptors.at(0).at(ddd).at(iii), allDescriptors.at(sss).at(ddd).at(iii), precRecall, correspCount, EVALUATION_OVERLAP_THRESHOLD );

                                            smoothAndSubsamplePrecisionRecallPoints(precRecall, 0.001);

                                            //calculateRepeatability( graySubsetImages.at(0).at(iii), graySubsetImages.at(sss).at(iii), H12_inv, pts1, pts2, repeatability, correspCount, EVALUATION_OVERLAP_THRESHOLD );
                                            if ((sss == 5) && (ddd == 0)) {
                                             //cin.get();
                                            }
                                        }
                                    }





                                    totalPrecRecall.push_back(precRecall);

                                }

                                if (featuresAvailable) {

                                    // if vectors are not all corresponding / the same lengths, might want to have a more sophisticated method
                                    // for coming up with an "averaged" result
                                    vector<Point2f> finalRecall;
                                    combineRecallVectors(totalPrecRecall, finalRecall);

                                    // Spit out prec/recall results

                                    printf("%s << writing out descriptor recall results (2)...\n", __FUNCTION__);

                                    printf("%s << aD.resultsAddress = %s\n", __FUNCTION__, aD.resultsAddress);
                                    printf("%s << finalRecall.size() = %d\n", __FUNCTION__, finalRecall.size());

                                    if (finalRecall.size() > 0) {
                                        writeDescriptorResults(aD.resultsAddress, finalRecall);
                                    }




                                    //cout << "DONE." << endl;
                                }



                            }



                        }

                    }
                }

            }
        }

        // =====================================================================================
        //                      INTER-MODALITY HOMOGRAPHIES
        // =====================================================================================
        if (iD.testAcrossModalities) {

            printf("\n<<-----INTER-MODALITY: READING IMAGES----->>\n\n");

            // Read in images...
            vector<vector<Mat> > crossModalityImages;

            int imagesToRead = std::min((unsigned int)MAX_IMAGES_PER_FOLDER, inputImages.size());

            for (int aaa = 0; aaa < 2; aaa++) {

                vector<string> inputImages;

                vector<Mat> modalityImages;

                if (aaa == 0) {
                    sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "visible", "profile");
                } else if (aaa == 1) {
                    sprintf(aD.imagePath, "%s/images/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "thermal", "profile");
                }

                countAndSortFiles(aD.imagePath, inputImages);

                for (int iii = 0; iii < imagesToRead; iii++) {

                    sprintf(aD.imageAddress, "%s/%s", aD.imagePath, inputImages.at(iii).c_str());

                    Mat currImg, grayImg, colImg, drawImg;

                    printf("%s << aD.imageAddress = %s\n", __FUNCTION__, aD.imageAddress);

                    currImg = imread(aD.imageAddress, -1);
                    imshow("testWin", currImg);
                    waitKey( 50 ); // 50

                    if (currImg.channels() == 3) {
                        currImg.copyTo(colImg);
                        cvtColor(currImg, grayImg, cv::COLOR_RGB2GRAY);

                    } else {
                        currImg.copyTo(grayImg);
                        cvtColor(currImg, colImg, cv::COLOR_GRAY2RGB);
                    }

                    modalityImages.push_back(grayImg);

                }

                crossModalityImages.push_back(modalityImages);
            }

            printf("\n<<-----INTER-MODALITY: GENERATING HOMOGRAPHIES----->>\n\n");

            sprintf(aD.homogPath, "%s/homographies/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str());
            mkdir(aD.homogPath, DEFAULT_MKDIR_PERMISSIONS);

            sprintf(aD.homogPath, "%s/homographies/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality");
            mkdir(aD.homogPath, DEFAULT_MKDIR_PERMISSIONS);

            sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality", "visible-thermal");
            mkdir(aD.homogPath, DEFAULT_MKDIR_PERMISSIONS);

            bool modalityHomographiesAlreadyExist = true;

            char homogSuffix[256];

            sprintf(homogSuffix, "V%04dtoT%04d.xml", 0, 0);
            sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

            if (!boost::filesystem::exists(aD.homogAddress)) {
                modalityHomographiesAlreadyExist = false;
            }

            vector<Mat> homographyDescriptors;

            vector<vector<KeyPoint> > modalityHomogKeypoints[2];
            vector<Mat> modalityHomogDescriptors[2];

            int maxImagesForModality = min(crossModalityImages.at(0).size(), crossModalityImages.at(1).size());

            if ((!modalityHomographiesAlreadyExist) || (iD.regenerateHomographies) || (iD.regenerateAllResults)) {

                // First get x-modality homography

                ::leftAnnotationPoints.clear();
                ::rightAnnotationPoints.clear();

                ::imageWidth = crossModalityImages.at(0).at(0).cols;

                printf("%s << DEBUG %d\n", __FUNCTION__, 0);

                crossModalityImages.at(0).at(0).copyTo(testImage_1);
                crossModalityImages.at(1).at(0).copyTo(testImage_2);

                makeAnnotationPair(::annotationImage, crossModalityImages.at(0).at(0), crossModalityImages.at(1).at(0));

                //makeAnnotationPair(::annotationImage, colSubsetImages.at(0).at(0), colSubsetImages.at(sss).at(0));

                imshow("testWin", ::annotationImage);

                setMouseCallback( "testWin", onMouse, 0 );

                waitKey( );


                if (leftAnnotationPoints.size() > rightAnnotationPoints.size()) {
                    leftAnnotationPoints.pop_back();
                } else if (rightAnnotationPoints.size() > leftAnnotationPoints.size()) {
                    rightAnnotationPoints.pop_back();
                }

                vector<Point2f> points1, points2;

                Mat H12;

                points1.assign(leftAnnotationPoints.begin(), leftAnnotationPoints.end());
                points2.assign(rightAnnotationPoints.begin(), rightAnnotationPoints.end());

                for (int abc = 0; abc < points1.size(); abc++) {
                    printf("%s << (%f, %f) to (%f, %f)\n", __FUNCTION__, points1.at(abc).x, points1.at(abc).y, points2.at(abc).x, points2.at(abc).y);
                }

                vector<uchar> validityMask;
                H12.release();

                H12 = findHomography( Mat(points1), Mat(points2), validityMask, CV_RANSAC, hD.ransacReprojThreshold );

                cout << "H12 = " << H12 << endl;

                //cout << "homog address = " << aD.homogAddress << endl;

                FileStorage fs(aD.homogAddress, FileStorage::WRITE);

                fs << "Homography" << H12;

                fs.release();

                sprintf(homogSuffix, "V%04dtoT%04d.xml", 0, 0);
                sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);


                for (int iii = 1; iii < maxImagesForModality; iii++) {

                    // Read in visible profile homography
                    Mat H_vis;


                    sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "visible", "profile");
                    sprintf(homogSuffix, "H%04dtoH%04d.xml", 0, iii);
                    sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                    printf("%s << aD.homogAddress = %s\n", __FUNCTION__, aD.homogAddress);

                    FileStorage fsA(aD.homogAddress, FileStorage::READ);
                    fsA["Homography"] >> H_vis;
                    fsA.release();



                    printf("%s << DEBUG %d\n", __FUNCTION__, 0);



                    // Read in thermal profile homography
                    Mat H_therm;

                    sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "thermal", "profile");
                    sprintf(homogSuffix, "H%04dtoH%04d.xml", 0, iii);
                    sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);


                    FileStorage fsB(aD.homogAddress, FileStorage::READ);
                    fsB["Homography"] >> H_therm;
                    fsB.release();

                    cout << "H_therm = " << H_therm << endl;

                    printf("%s << DEBUG %d\n", __FUNCTION__, 1);



                    Mat Hnew;

                    // Calculate homography using H12 and profile homographies...

                    Mat H_vis_inv, H_therm_inv, H12_inv;



                                //printf("%s << X.size() = (%d, %d)\n", __FUNCTION__, X.rows, X.cols);

                    invert(H_vis, H_vis_inv);
                    invert(H_therm, H_therm_inv);
                    invert(H12, H12_inv);

                    cout << "H_vis_inv = " << H_vis_inv << endl;

                    cout << "H12 = " << H12 << endl;



                    printf("%s << About to perform equation...\n", __FUNCTION__);

                    //Hnew = H_vis_inv * H12 * H_therm;
                    //Hnew = H_vis * H12 * H_therm;
                    //Hnew = H_vis * H12_inv * H_therm_inv;
                    Hnew = H_therm * H12 * H_vis_inv;

                    printf("%s << Equation performed...\n", __FUNCTION__);


                    sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality", "visible-thermal");
                    sprintf(homogSuffix, "V%04dtoT%04d.xml", iii, iii);
                    sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                    printf("%s << aD.homogAddress = %s\n", __FUNCTION__, aD.homogAddress);

                    FileStorage fsC(aD.homogAddress, FileStorage::WRITE);
                    fsC << "Homography" << Hnew;
                    fsC.release();

                    imshow("wrp1", crossModalityImages.at(1).at(iii));
                    waitKey(0);
                    Mat warpedImage;
                    warpPerspective(crossModalityImages.at(0).at(iii), warpedImage, Hnew, crossModalityImages.at(0).at(iii).size());
                    //printf("%s << Showing warped image #1\n", __FUNCTION__);
                    imshow("wrp1", warpedImage);
                    waitKey(0);



                }



            }

            printf("\n<<-----INTER-MODALITY: DETECTOR REPEATABILITY----->>\n\n");

            sprintf(aD.resultsPath, "%s/results/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str());
            mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);

            sprintf(aD.resultsPath, "%s/results/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality");
            mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);

            sprintf(aD.resultsPath, "%s/results/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality", "visible-thermal");
            mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);

            sprintf(aD.resultsPath, "%s/match_results/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str());
            mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);

            sprintf(aD.resultsPath, "%s/match_results/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality");
            mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);

            sprintf(aD.resultsPath, "%s/match_results/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality", "visible-thermal");
            mkdir(aD.resultsPath, DEFAULT_MKDIR_PERMISSIONS);

            printf("%s << DD_DETECTORS_COUNT = %d\n", __FUNCTION__, DD_DETECTORS_COUNT);

            //cin.get();

            // For each detector
            for (int ddd = 0; ddd < DD_DETECTORS_COUNT; ddd++) {

                bool resultsAlreadyExist = true;
                char resultsSuffix[64], matchResultsSuffix[64];

                sprintf(resultsSuffix, "%s.csv", (DD_DETECTOR_NAMES[ddd]).c_str());
                sprintf(aD.resultsAddress, "%s/%s", aD.resultsPath, resultsSuffix);

                sprintf(matchResultsSuffix, "%s.csv", (DD_DETECTOR_NAMES[ddd]).c_str());
                sprintf(aD.matchResultsAddress, "%s/%s", aD.matchResultsPath, matchResultsSuffix);

                //printf("%s << DEBUG %d\n", __FUNCTION__, -98);

                if (!boost::filesystem::exists(aD.resultsAddress)) {
                    resultsAlreadyExist = false;
                    // break;
                }

                if (!boost::filesystem::exists(aD.matchResultsAddress)) {
                    resultsAlreadyExist = false;
                    // break;
                }

                //printf("%s << DEBUG %d\n", __FUNCTION__, -97);

                if (resultsAlreadyExist) {
                    printf("%s << Repeatability results for %s detector already exist.\n", __FUNCTION__, (DD_DETECTOR_NAMES[ddd]).c_str());
                }

                //cin.get();

                //printf("%s << DEBUG %d\n", __FUNCTION__, -96);


                if ((!resultsAlreadyExist) || (iD.regenerateDetectorResults) || (iD.regenerateAllResults)) {
                    printf("%s << Calculating repeatability for %s detector across modalities...\n", __FUNCTION__, (DD_DETECTOR_NAMES[ddd]).c_str());

                    int minMaxCount = 9999;

                    char visibleKeypointsPath[256], thermalKeypointsPath[256];

                    char keypointsSuffix[64];

                    // Establish addresses of keypoints files for both modalities

                    sprintf(visibleKeypointsPath, "%s/keypoints/%s/%s/profile", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "visible");
                    sprintf(thermalKeypointsPath, "%s/keypoints/%s/%s/profile", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "thermal");

                    vector<vector<KeyPoint> > visiblePtVector, thermalPtVector;

                    for (int iii = 0; iii < maxImagesForModality; iii++) {

                        vector<KeyPoint> visPoints, thermPoints;

                        char keypointsAddress[256];

                        sprintf(keypointsSuffix, "%s-%04d.txt", (DD_DETECTOR_NAMES[ddd]).c_str(), iii);

                        sprintf(keypointsAddress, "%s/%s", visibleKeypointsPath, keypointsSuffix);

                        printf("%s << keypointsAddress[visible] = %s\n", __FUNCTION__, keypointsAddress);

                        readKeypoints_MATLAB(keypointsAddress, visPoints);
                        sortKeypoints(visPoints);

                        visiblePtVector.push_back(visPoints);

                        if (visPoints.size() < minMaxCount) {
                            minMaxCount = visPoints.size();
                        }

                        sprintf(keypointsAddress, "%s/%s", thermalKeypointsPath, keypointsSuffix);

                        printf("%s << keypointsAddress[thermal] = %s\n", __FUNCTION__, keypointsAddress);

                        readKeypoints_MATLAB(keypointsAddress, thermPoints);
                        sortKeypoints(thermPoints);

                        thermalPtVector.push_back(thermPoints);

                        if (thermPoints.size() < minMaxCount) {
                            minMaxCount = thermPoints.size();
                        }

                    }

                    printf("%s << minMaxCount = %d\n", __FUNCTION__, minMaxCount);

                    vector<int> averageCounts;
                    vector<float> averageScores, averageMatchScores;
                    vector<float> upperScores, upperMatchScores;
                    vector<float> lowerScores, lowerMatchScores;

                    int initialCount, numberOfReps;
                    numberOfReps = DD_PROFILE_LEVELS;

                    for (int fff = 0; fff < numberOfReps; fff++) {

                        int aimedCount = DD_PROFILE_LIST[fff];

                        if (aimedCount > minMaxCount) {
                            printf("%s << Insufficient features for analysis. (%d / %d)\n", __FUNCTION__, minMaxCount, aimedCount);
                            break;
                        }

                        double minVisAbsResponse = 0.0, minThermAbsResponse = 0.0;

                        // For each image
                        for (int iii = 0; iii < maxImagesForModality; iii++) {
                            // figure out typical response for that count

                            minVisAbsResponse += abs(visiblePtVector.at(iii).at(aimedCount-1).response);
                            minThermAbsResponse += abs(thermalPtVector.at(iii).at(aimedCount-1).response);

                        }


                        minVisAbsResponse /= maxImagesForModality;
                        minThermAbsResponse /= maxImagesForModality;

                        int currAvCountVis = 0, currAvCountTherm = 0;


                        vector<float> repeatabilityScores, matchabilityScores;


                        for (int iii = 0; iii < maxImagesForModality; iii++) {

                            vector<KeyPoint> pts1, pts2;

                            obtainSubset(visiblePtVector.at(iii), pts1, minVisAbsResponse);
                            obtainSubset(thermalPtVector.at(iii), pts2, minThermAbsResponse);

                            currAvCountVis += pts1.size();
                            currAvCountTherm += pts2.size();

                            char homogSuffix[64];
                            sprintf(homogSuffix, "V%04dtoT%04d.xml", iii, iii);
                            sprintf(aD.homogPath, "%s/homographies/%s/%s/%s", iD.dataPath, fD.datasets.at(datasetCounter).c_str(), "x-modality", "visible-thermal");
                            sprintf(aD.homogAddress, "%s/%s", aD.homogPath, homogSuffix);

                            Mat HX;

                            FileStorage fsX(aD.homogAddress, FileStorage::READ);
                            fsX["Homography"] >> HX;
                            fsX.release();

                            // Determine repeatability (and sum to total)
                            float repeatability, matchability;
                            int correspCount;

                            Mat thresholdedOverlapMask;

                            calculateRepeatability( crossModalityImages.at(0).at(iii), crossModalityImages.at(1).at(iii), HX, pts1, pts2, repeatability, correspCount, EVALUATION_OVERLAP_THRESHOLD, &thresholdedOverlapMask );
                            //calculateMatchability( crossModalityImages.at(0).at(iii), crossModalityImages.at(1).at(iii), HX, pts1, pts2, matchability, correspCount, EVALUATION_OVERLAP_THRESHOLD, thresholdedOverlapMask );

                            printf("%s << Matchability calculated...\n", __FUNCTION__);
                            //printf("%s << DEBUG fff[%d] iii[%d] jjj[%d] : %d\n", __FUNCTION__, fff, iii, jjj, 3);

                                // Need to correct if -1 is returned, which means no overlap at all...
                            repeatability = max(repeatability, (float) 0.0);
                            matchability = max(matchability, (float) 0.0);
                                //cout << "repeatability = " << repeatability << endl;

                            repeatabilityScores.push_back(repeatability);
                            matchabilityScores.push_back(matchability);

                        }

                        currAvCountVis /= maxImagesForModality;
                        currAvCountTherm /= maxImagesForModality;

                        int currAvCountBoth = (currAvCountVis + currAvCountTherm) / 2.0;

                        averageCounts.push_back(currAvCountBoth);

                        // Determine final repeatability

                        float repeatSumm[3], matchSumm[3];    // mean, upp, low

                        summarizeScores(repeatabilityScores, repeatSumm);
                        summarizeScores(matchabilityScores, matchSumm);

                        averageScores.push_back(repeatSumm[0]);
                        upperScores.push_back(repeatSumm[1]);
                        lowerScores.push_back(repeatSumm[2]);

                        averageMatchScores.push_back(matchSumm[0]);
                        upperMatchScores.push_back(matchSumm[1]);
                        lowerMatchScores.push_back(matchSumm[2]);

                        if ((fff % 5) == 0) {
                            printf("%s << Average repeatability for count of [%d] = %f\n", __FUNCTION__, aimedCount, repeatSumm[0]);
                            printf("%s << Average matchability for count of [%d] = %f\n", __FUNCTION__, aimedCount, matchSumm[0]);
                        }


                    }

                    // Spit out repeatability results

                    printf("%s << Printing results (if sufficient).\n", __FUNCTION__);

                    if (averageScores.size() > 0) {
                        FILE *file = fopen (aD.resultsAddress, "w");
                        for (int zzz = 0; zzz < averageScores.size(); zzz++)
                        {
                            fprintf( file, "%d, %f, %f, %f\n", averageCounts.at(zzz), averageScores.at(zzz), upperScores.at(zzz), lowerScores.at(zzz));
                        }

                        fclose( file );
                    }

                    if (averageMatchScores.size() > 0) {
                        FILE *file = fopen (aD.matchResultsAddress, "w");
                        for (int zzz = 0; zzz < averageMatchScores.size(); zzz++)
                        {
                            fprintf( file, "%d, %f, %f, %f\n", averageCounts.at(zzz), averageMatchScores.at(zzz), upperMatchScores.at(zzz), lowerMatchScores.at(zzz));
                        }

                        fclose( file );
                    }

                    printf("%s << Results printed.\n", __FUNCTION__);


                }


            }

        }




    }

}

void onMouse( int event, int x, int y, int, void* ) {

    Point2f seed = Point(x,y);

    //printf("%s << Click co-ordinates: (%d, %d)\n", __FUNCTION__, x, y);

    if (event == CV_EVENT_LBUTTONDOWN) {
        printf("%s << Left button pressed!\n", __FUNCTION__);

        if ((x > 0) && (x < imageWidth) && (y > 0) && (y < imageHeight)) {
            if (leftAnnotationPoints.size() <= rightAnnotationPoints.size()) {
                leftAnnotationPoints.push_back(seed);
            }

        } else if ((x > imageWidth) && (x < 2*imageWidth) && (y > 0) && (y < imageHeight)) {
            if (rightAnnotationPoints.size() <= leftAnnotationPoints.size()) {
                rightAnnotationPoints.push_back(Point2f(seed.x - imageWidth, seed.y));
            }
        }


    } else if (event == CV_EVENT_RBUTTONDOWN) {
        printf("%s << Right button pressed!\n", __FUNCTION__);

        if ((x > 0) && (x < imageWidth) && (y > 0) && (y < imageHeight)) {
            if ((leftAnnotationPoints.size() > 0) && (leftAnnotationPoints.size() >= rightAnnotationPoints.size())) {
                leftAnnotationPoints.pop_back();
            }

        } else if ((x > imageWidth) && (x < 2*imageWidth) && (y > 0) && (y < imageHeight)) {
            if ((rightAnnotationPoints.size() > 0) && (rightAnnotationPoints.size() >= leftAnnotationPoints.size())) {
                rightAnnotationPoints.pop_back();
            }

        }
    }

    Mat drawingImg;

    //printf("%s << (%d)\n", __FUNCTION__, ::annotationImage.channels());

    //cvtColor(::annotationImage, drawingImg, cv::COLOR_GRAY2RGB);
    (::annotationImage).copyTo(drawingImg);

    for (int iii = 0; iii < max(leftAnnotationPoints.size(), rightAnnotationPoints.size()); iii++) {
        if (leftAnnotationPoints.size() > iii) {
            circle(drawingImg, leftAnnotationPoints.at(iii), 5, cv::Scalar(255, 255*((iii/2) % 2), 255*(((iii+1)/2) % 2)), 2);
        }

        if (rightAnnotationPoints.size() > iii) {
            circle(drawingImg, Point2f(rightAnnotationPoints.at(iii).x + imageWidth, rightAnnotationPoints.at(iii).y), 5, cv::Scalar(255, 255*((iii/2) % 2), 255*(((iii+1)/2) % 2)), 2);
        }
    }

    if ((leftAnnotationPoints.size() >= 4) && (rightAnnotationPoints.size() >= 4)) {
        unsigned int ptsToUseForHomog = min(leftAnnotationPoints.size(), rightAnnotationPoints.size());

        vector<Point2f> leftTestPts, rightTestPts;

        for (unsigned int iii = 0; iii < ptsToUseForHomog; iii++) {
            leftTestPts.push_back(leftAnnotationPoints.at(iii));
            rightTestPts.push_back(rightAnnotationPoints.at(iii));

        }

        vector<uchar> validityMask;
        Mat H12_test;

        H12_test = findHomography( Mat(leftTestPts), Mat(rightTestPts), validityMask, CV_RANSAC, 1.0 );

        Mat warpedIm, debugIm;

        printf("%s << Warping... (%d,%d,%d) vs (%d,%d,%d)\n", __FUNCTION__, testImage_1.rows, testImage_1.cols, testImage_1.type(), 0, 0, 0);

        warpPerspective(testImage_1, warpedIm, H12_test, testImage_2.size());

        printf("%s << Warped!\n", __FUNCTION__);

        debugIm = Mat::zeros(testImage_1.size(), CV_8UC1);

        // printf("%s << sizes = (%d, %d) (%d, %d) (%d, %d)\n", __FUNCTION__, testImage_1.rows, testImage_1.cols, debugIm.rows, debugIm.cols, warpedIm.rows, warpedIm.cols);

        for (unsigned int iii = 0; iii < debugIm.rows; iii++) {
            for (unsigned int jjj = 0; jjj < debugIm.cols; jjj++) {
                //debugIm.at<Vec3b>(iii,jjj)[0] = warpedIm.at<unsigned char>(iii,jjj);
                //debugIm.at<Vec3b>(iii,jjj)[1] = testImage_2.at<unsigned char>(iii,jjj);

                if ((((iii % debugIm.rows/32) % 2) > 0) && (((jjj % debugIm.cols/32) % 2) == 0)) {
                    debugIm.at<unsigned char>(iii,jjj) = warpedIm.at<unsigned char>(iii,jjj); // - testImage_2.at<unsigned char>(iii,jjj);
                } else if ((((iii % debugIm.rows/32) % 2) == 0) && (((jjj % debugIm.cols/32) % 2) > 0)) {
                    debugIm.at<unsigned char>(iii,jjj) = warpedIm.at<unsigned char>(iii,jjj); // - testImage_2.at<unsigned char>(iii,jjj);
                } else if ((((iii % debugIm.rows/32) % 2) == 0) && (((jjj % debugIm.cols/32) % 2) == 0)) {
                    debugIm.at<unsigned char>(iii,jjj) = testImage_2.at<unsigned char>(iii,jjj); // - warpedIm.at<unsigned char>(iii,jjj);
                }  else if ((((iii % debugIm.rows/32) % 2) > 0) && (((jjj % debugIm.cols/32) % 2) > 0)) {
                    debugIm.at<unsigned char>(iii,jjj) = testImage_2.at<unsigned char>(iii,jjj); // - warpedIm.at<unsigned char>(iii,jjj);
                }

                if (iii < debugIm.rows/32)

                if (warpedIm.at<unsigned char>(iii,jjj) > testImage_2.at<unsigned char>(iii,jjj)) {
                    //debugIm.at<unsigned char>(iii,jjj) = warpedIm.at<unsigned char>(iii,jjj); // - testImage_2.at<unsigned char>(iii,jjj);
                } else {
                    //debugIm.at<unsigned char>(iii,jjj) = testImage_2.at<unsigned char>(iii,jjj); // - warpedIm.at<unsigned char>(iii,jjj);
                }


            }
        }

        if (globswitch) {
            //imshow("fusedWin", warpedIm);
            //globswitch = false;
        } else {
            //imshow("fusedWin", testImage_2);
            //globswitch = true;
        }

        imshow("fusedWin", debugIm);
        // waitKey(1);

    }

    imshow("testWin", drawingImg);
    // waitKey(1);
#endif
}
