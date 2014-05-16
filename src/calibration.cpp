/*! \file	calibration.cpp
 *  \brief	Definitions for generic geometric calibration.
*/

#include "calibration.hpp"

void generateRandomIndexArray(int * randomArray, int maxElements, int maxVal)
{

    srand ( (unsigned int)(time(NULL)) );

    vector<int> validValuesVector, randomSelection;

    for (int iii = 0; iii < maxVal; iii++)
    {
        validValuesVector.push_back(iii);
    }

    for (int iii = 0; iii < maxElements; iii++)
    {
        int currIndex = rand() % validValuesVector.size();

        randomSelection.push_back(validValuesVector.at(currIndex));
        validValuesVector.erase(validValuesVector.begin() + currIndex);
    }

    sort(randomSelection.begin(), randomSelection.end());

    for (unsigned int iii = 0; iii < randomSelection.size(); iii++)
    {
        randomArray[iii] = randomSelection.at(iii);
    }
}

mserPatch::mserPatch()
{

}

mserPatch::mserPatch(vector<Point>& inputHull, const Mat& image)
{
    double dbx, dby;
    int x, y, division = 0;
    meanIntensity = 0.0;

    copyContour(inputHull, hull);

    // get moments
    momentSet = moments(Mat(hull));

    // get centroid
    dbx = (momentSet.m10/momentSet.m00);
    dby = (momentSet.m01/momentSet.m00);
    x = (int) dbx;
    y = (int) dby;
    centroid = Point(x,y);
    centroid2f = Point2f(float(dbx), float(dby));

    // get area
    area = contourArea(Mat(hull));

    int searchDist = 15;

    // get mean pixel intensity
    for (int i = x-searchDist; i < x+searchDist+1; i++)
    {
        for (int j = y-searchDist; j < y+searchDist+1; j++)
        {

            if (pointPolygonTest(Mat(hull), Point2f(float(i),float(j)), false) > 0.0)
            {
                //printf("%s << (j, i) = (%d, %d)\n", __FUNCTION__, j, i);
                meanIntensity = meanIntensity + double(image.at<Vec3b>(j,i)[0]);
                division++;
            }
        }
    }

    meanIntensity /= division;

    // get variance
    varIntensity = 0.0;

    for (int i = x-searchDist; i < x+searchDist+1; i++)
    {
        for (int j = y-searchDist; j < y+searchDist+1; j++)
        {

            if (pointPolygonTest(Mat(hull), Point2f(float(i),float(j)), false) > 0.0)
            {
                //printf("%s << (j, i) = (%d, %d)\n", __FUNCTION__, j, i);
                varIntensity += pow((meanIntensity - double(image.at<Vec3b>(j,i)[0])), 2);
            }
        }
    }

    varIntensity /= division;

    //printf("%s << variance = %f\n", __FUNCTION__, varIntensity);

    // get mean pixel intensity
    /*
    for (int i = x-7; i < x+8; i++) {
        if ((i < 0) || (i >= image.cols)) {
            i++;
        } else {
            for (int j = y-7; j < y+8; j++) {
                if ((j < 0) || (j >= image.rows)) {
                    j++;
                } else {
                    //printf("center: %d, %d\n", x, y);
    				//printf("%s << j = %d; i = %d\n", __FUNCTION__, j, i);
                    meanIntensity = meanIntensity + double(image.at<Vec3b>(j,i)[0]);
                    //printf("in loop: %d, %d\n", i, j);
                    //cin.get();
                    division++;
                }

            }
        }
    }
    */



    // get variance of pixel intensity
}

bool findMaskCorners(const Mat& image, Size patternSize, vector<Point2f>& corners, int detector, int mserDelta, float max_var, float min_diversity, double area_threshold)
{
        Mat grayIm;
        if (image.channels() > 1) {
                cvtColor(image, grayIm, CV_RGB2GRAY);
        } else {
                grayIm = Mat(image);
        }
        //cout << "ALPHA" << endl;
     return findPatternCorners(grayIm, patternSize, corners, 1, detector, mserDelta, max_var, min_diversity, area_threshold);
}


bool checkAcutance()
{
    bool retVal = true;

    return retVal;
}

void determinePatchDistribution(Size patternSize, int mode, int &rows, int &cols, int &quant)
{
    if (mode == 0)
    {
        rows = patternSize.height + 1;
        cols = patternSize.width + 1;
        quant = patternSize.width*patternSize.height/2;
    }
    else
    {
        rows = patternSize.height / 2;
        cols = patternSize.width / 2;
        quant = patternSize.width*patternSize.height/4;
    }

}

void findAllPatches(const Mat& image, Size patternSize, vector<vector<Point> >& msers, int delta, float max_var, float min_diversity, double area_threshold)
{

    //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 0);

    int64 t = getTickCount();

    int imWidth = image.size().width;
    int imHeight = image.size().height;
    int maxArea, minArea;

    maxArea = 2 * (imWidth / (patternSize.width-1)) * (imHeight / (patternSize.height-1));
    minArea = MINIMUM_MSER_AREA;

    int X = patternSize.width/2, Y = patternSize.height/2;

    Mat displayMat(image);
    Mat mask = Mat::ones(image.rows, image.cols, CV_8U);

    //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 1);

    // 15
    MSER mserExtractor(delta, minArea, maxArea, max_var, min_diversity, 200, area_threshold, 0.003, 5); // delta = 8, max var = 0.1
    /*
    cvMSERParams(int delta = 5, int min_area = 60, int max_area = 14400, \n"
        "    float max_variation = .25, float min_diversity = .2, \n"
        "    int max_evolution = 200, double area_threshold = 1.01, \n"
        "    double min_margin = .003, \n"
        "    int edge_blur_size = 5)
    */

    //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 2);

    // Copy image but into greyscale
    Mat imGrey;
    if (image.channels() > 1)
    {
        cvtColor(image, imGrey, CV_RGB2GRAY);
    }
    else
    {
        image.copyTo(imGrey);
    }

    // Blur image a bit...
    int kernelSize = min(imWidth, imHeight) / (5*max(patternSize.width, patternSize.height));
    kernelSize = kernelSize + 1 + (kernelSize % 2);
    //printf("%s << kernelSize = %d\n", __FUNCTION__, kernelSize);
    GaussianBlur(imGrey, imGrey, Size(kernelSize,kernelSize), 0,0);

    //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 3);

    //printf("%s << imGrey.size() = (%d,%d)\n", __FUNCTION__, imGrey.rows, imGrey.cols);

    //imshow("blurred", imGrey);
    //waitKey(0);


    // Processing of greyscale version
    // thresholding?


    // Extract MSER features
    mserExtractor(imGrey, msers, mask);

    //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 4);

    // Clean up MSER features by putting them in a convex hull
    for (unsigned int i = 0; i < msers.size(); i++)
    {
        convexHull(Mat(msers[i]), msers[i]);
    }

    //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 5);

    if (DEBUG_MODE > 1)
    {
        printf("%s << MSERs found: %d\n", __FUNCTION__, msers.size());
    }

    if (DEBUG_MODE > 0)
    {
        t = getTickCount() - t;
        printf("%s << Algorithm duration: %fms\n", __FUNCTION__, t*1000/getTickFrequency());
    }

    // Extract SURF features
    /*
    SURF surfExtractor();

    vector<KeyPoint> surfs;
    surfExtractor(imGrey, mask, surfs);
    */

    if (DEBUG_MODE > 1)
    {
        printf("%s << Total patches found = %d\n", __FUNCTION__, msers.size());
    }
}

void randomCulling(vector<std::string> &inputList, int maxSearch)
{

    srand ( (unsigned int)(time(NULL)) );

    int deletionIndex = 0;

    while (int(inputList.size()) > maxSearch)
    {
        deletionIndex = rand() % inputList.size();
        inputList.erase(inputList.begin() + deletionIndex);
    }
}

void randomCulling(vector<std::string> &inputList, int maxSearch, vector<vector<Point2f> >& patterns)
{

    srand ( (unsigned int)(time(NULL)) );

    int deletionIndex = 0;

    while (int(inputList.size()) > maxSearch)
    {
        deletionIndex = rand() % inputList.size();
        inputList.erase(inputList.begin() + deletionIndex);
        patterns.erase(patterns.begin() + deletionIndex);
    }
}

void randomCulling(vector<string>& inputList, int maxSearch, vector<vector<vector<Point2f> > >& patterns)
{
    srand ( (unsigned int)(time(NULL)) );

    int deletionIndex = 0;

    printf("%s << inputList.size() = %d / %d\n", __FUNCTION__, inputList.size(), maxSearch);

    while (int(inputList.size()) > maxSearch)
    {
        deletionIndex = rand() % inputList.size();
        inputList.erase(inputList.begin() + deletionIndex);

        for (unsigned int i = 0; i < patterns.size(); i++)
        {
            patterns.at(i).erase(patterns.at(i).begin() + deletionIndex);
        }

    }

    //printf("%s << patterns.at(i).size() = %d / %d\n", __FUNCTION__, patterns.at(0).size(), maxSearch);
    //printf("%s << Waiting for key...\n", __FUNCTION__);
    //cin.get();
}

void debugDisplayPatches(const Mat& image, vector<vector<Point> >& msers)
{

    Scalar color(0, 0, 255);
    Mat dispMat;

    image.copyTo(dispMat);

    drawContours(dispMat, msers, -1, color, 1);

    if (image.cols > 640)
    {
        Mat dispMat2;
        resize(dispMat, dispMat2, Size(0,0), 0.5, 0.5);
        imshow("mainWin", dispMat2);
    }
    else
    {
        imshow("mainWin", dispMat);
    }

    waitKey(0);
}

void determineFindablePatches(Size patternSize, int mode, int *XVec, int *YVec)
{
    int X, Y;

    if (mode == 0)
    {

        Y = patternSize.height + 1;

        // XVec tells you how many 'black' patches to look for between extremes in that row

        for (int i = 0; i < Y; i++)
        {
            if ((i % 2) > 0)      // odd
            {
                XVec[i] = int(ceil(((double(patternSize.width) + 1)/2)-0.01)-1);
            }
            else            // even
            {
                XVec[i] = int(floor(((double(patternSize.width) + 1)/2)+0.01)-1);
            }

        }

        // YVec tells you how many 'black' patches to look for between extremes on the LHS and RHS
        YVec[0] = int(floor(((patternSize.height + 1)/2)+0.01)-1);

        if (patternSize.width % 2)      // odd
        {
            if (patternSize.height % 2)     // odd
            {
                YVec[1] = int(floor(((patternSize.height + 1)/2)+0.01)-1);
            }
            else
            {
                YVec[1] = int(floor(((patternSize.height + 1)/2)+0.01));
            }

        }
        else            // even
        {
            YVec[1] = int(floor(((patternSize.height + 1)/2)+0.01)-1);
        }

    }
    else
    {
        X = patternSize.width/2;
        Y = patternSize.height/2;

        for (int i = 0; i < Y; i++)
        {
            if (i % 2)
            {
                XVec[i] = patternSize.width/2 - 2;
            }
            else
            {
                XVec[i] = patternSize.width/2 - 2;
            }

        }

        YVec[0] = Y-2;
        YVec[1] = Y-2;
    }

    if (DEBUG_MODE > 3)
    {
        printf("%s << Total number of rows: %d\n", __FUNCTION__, Y);
        printf("%s << Number of findable patches in LHS: %d\n", __FUNCTION__, YVec[0]);
        printf("%s << Number of findable patches in RHS: %d\n", __FUNCTION__, YVec[1]);

        for (int i = 0; i < Y; i++)
        {
            printf("%s << Number of findable patches in row[%d]: %d\n", __FUNCTION__, i, XVec[i]);
        }
    }
}

void findCornerPatches(Size imageSize, Size patternSize, int mode, int *XVec, int *YVec, vector<Point2f>& patchCentres, vector<Point2f>& remainingPatches)
{

    Point2f center;
    float angle;
    Size2f size;
    vector<vector<Point> > sideCenters(1);
    vector<vector<Point> > fourCorners(1);
    vector<Point> orderedCorners;
    vector<vector<Point> > orderedForDisplay;
    double x,y,X,Y;
    //vector<Point> wrapperHull;

    //convexHull(Mat(remainingPatches), wrapperHull);

    RotatedRect wrapperRectangle;

    wrapperRectangle = fitEllipse(Mat(remainingPatches));

    center = wrapperRectangle.center;
    angle = wrapperRectangle.angle;
    size = wrapperRectangle.size;

    //printf("%s << Wrapper Rectangle params: center = [%f, %f]; angle = %f; size = [%f, %f]\n", __FUNCTION__, center.x, center.y, angle, size.width, size.height);

    //vertices.push_back(Point2f(center.x - size.height*cos(angle), center.y - size.width*sin(angle)));
    //vertices.push_back(Point2f(center.x + size.height*cos(angle), center.y + size.width*sin(angle)));

    x = center.x;
    y = center.y;
    X = (size.width / 2) * 1.2;     // dilate the elipse otherwise it might
    Y = (size.height / 2) * 1.2;

    //sideCenters.at(0).push_back(Point(x-X*cos((3.14/180)*angle),y+X*sin((3.14/180)*angle)));
    //sideCenters.at(0).push_back(Point(x, y));
    /*
    sideCenters.at(0).push_back(Point(x-Y*sin((3.14/180)*angle),y+Y*cos((3.14/180)*angle)));
    sideCenters.at(0).push_back(Point(x+X*cos((3.14/180)*angle),y+X*sin((3.14/180)*angle)));
    sideCenters.at(0).push_back(Point(x+Y*sin((3.14/180)*angle),y-Y*cos((3.14/180)*angle)));
    sideCenters.at(0).push_back(Point(x-X*cos((3.14/180)*angle),y-X*sin((3.14/180)*angle)));
    */

    fourCorners.at(0).push_back(Point(int(x-X*cos((3.14/180)*angle)-Y*sin((3.14/180)*angle)), int(y-X*sin((3.14/180)*angle)+Y*cos((3.14/180)*angle))));
    fourCorners.at(0).push_back(Point(int(x+Y*sin((3.14/180)*angle)-X*cos((3.14/180)*angle)), int(y-Y*cos((3.14/180)*angle)-X*sin((3.14/180)*angle))));
    fourCorners.at(0).push_back(Point(int(x+X*cos((3.14/180)*angle)+Y*sin((3.14/180)*angle)), int(y+X*sin((3.14/180)*angle)-Y*cos((3.14/180)*angle))));
    fourCorners.at(0).push_back(Point(int(x-Y*sin((3.14/180)*angle)+X*cos((3.14/180)*angle)), int(y+Y*cos((3.14/180)*angle)+X*sin((3.14/180)*angle))));


    for (int i = 0; i < 4; i++)
    {
        //printf("x = %d, y = %d\n", fourCorners.at(0).at(i).x, fourCorners.at(0).at(i).y);
    }

    //waitKey(0);

    Mat imCpy(imageSize, CV_8UC3);
    imCpy.setTo(0);
    Scalar color( rand()&255, rand()&255, rand()&255 );

    ellipse(imCpy, wrapperRectangle, color);

    color = Scalar(rand()&255, rand()&255, rand()&255);


    //drawContours(imCpy, fourCorners, -1, color, 3);
    //rectangle(imCpy, vertices.at(0), vertices.at(1), color, 2);
    //imshow("testWin", imCpy);
    //waitKey(0);


    // Now re-order these points according to which one is closest to each image corner:

    double distMin[4] = {99999999, 99999999, 99999999, 99999999};
    double dist;
    int distIndex[4] = {0, 0, 0, 0};
    float extremesX[4] = {0.0, float(imageSize.width), float(imageSize.width), 0.0}; // TL, TR, BR, BL
    float extremesY[4] = {0.0, 0.0, float(imageSize.height), float(imageSize.height)};
    Point tmpPoint;
    double minDist;
    int minIndex = 0;
    double sumDist;
    //int bestConfiguration;

    // For each of the four configurations
    minDist = 9e9;

    for (int k = 0; k < 4; k++)
    {
        orderedCorners.clear();
        sumDist = 0;
        // For each of the remaining rectangle corners
        for (int i = 0; i < 4; i++)
        {
            tmpPoint = Point(int(extremesX[i]), int(extremesY[i]));
            dist = distBetweenPts(fourCorners.at(0).at((k+i)%4), tmpPoint);
            //printf("dist[%d,%d] = %f\n", k, i, dist);

            sumDist += dist;
        }

        //printf("sumDist[%d] = %f\n", k, sumDist);

        if (sumDist < minDist)
        {
            minDist = sumDist;
            minIndex = k;
        }
    }

    //printf("minIndex = %d\n", minIndex);

    // now minIndex represents the best configuration
    orderedCorners.push_back(fourCorners.at(0).at((minIndex)%4));
    orderedCorners.push_back(fourCorners.at(0).at((minIndex+1)%4));
    orderedCorners.push_back(fourCorners.at(0).at((minIndex+3)%4));
    orderedCorners.push_back(fourCorners.at(0).at((minIndex+2)%4));

    for (int i = 0; i < 4; i++)
    {
        //printf("x = %d, y = %d\n", orderedCorners.at(i).x, orderedCorners.at(i).y);
    }

    //waitKey(0);

    /*
    orderedForDisplay.push_back(orderedCorners);
    drawContours(imCpy, orderedForDisplay, -1, color, 3);
    //rectangle(imCpy, vertices.at(0), vertices.at(1), color, 2);
    imshow("testWin", imCpy);
    waitKey(40);
    */

    vector<Point> pointPair;

    // go thru remaining points and add the ones that are closest to these
    for (int k = 0; k < 4; k++)
    {
        minDist = 9e9;
        for (unsigned int i = 0; i < remainingPatches.size(); i++)
        {

            //printf("%s << remainingPatches.at(%d) = (%f, %f)\n", __FUNCTION__, i, remainingPatches.at(i).x, remainingPatches.at(i).y);

            pointPair.clear();
            pointPair.push_back(orderedCorners.at(k));
            pointPair.push_back(remainingPatches.at(i));
            dist = arcLength(Mat(pointPair), false); // total distance to extremes

            //printf("distance for point [%d] to corner [%d] = %f\n", i, k, dist);

            if (dist < minDist)
            {
                //printf("minimum found.\n");
                minDist = dist;
                minIndex = i;
            }

        }

        //printf("Best point for %d: %d\n", k, minIndex);
        //circle(imCpy, corners.at(minIndex), 8, color, 3, 8, 0);
        transferElement(patchCentres, remainingPatches, minIndex);
    }

    if (DEBUG_MODE > 3)
    {
        for (unsigned int i = 0; i < patchCentres.size(); i++)
        {
            printf("%s << patchCentres.at(%d) = (%f, %f)\n", __FUNCTION__, i, patchCentres.at(i).x, patchCentres.at(i).y);
        }

        printf("%s << remainingPatches.size() = %d\n", __FUNCTION__, remainingPatches.size());
    }

    return;

    // TODO:
    // Depending on the pattern dimensions, not all corners will be occupied by black squares (for chessboard).
    // However, at least 2 will definitely be. If it's not a single black patch in a corner, it will be
    // 2 black patches in a corner

    // Real or virtual corners are now stored in cornerPoints
    // Could also try to look at which patches have an insufficient number of near-neighbours to be interior

    // Once corners and virtual corners are found, figure out which corner of the image to attach them to
    // based on a minimization of distances between the four pattern corners and the four image corners

    // This function should create virtual corners as necessary

    // Correct order of points
    //double imgCornerDistances[remainingPatches.size()][4];
    //double distMin[4] = {99999999, 99999999, 99999999, 99999999};
    //double dist;
    //int distIndex[4] = {0, 0, 0, 0};
    //float extremesX[4] = {0, imageSize.width, 0, imageSize.width};
    //float extremesY[4] = {0, 0, imageSize.height, imageSize.height};

    if (DEBUG_MODE > 3)
    {
        printf("%s << imageSize.width = %d; imageSize.height = %d\n", __FUNCTION__, imageSize.width, imageSize.height);
    }
    //int cornerIndex[4] = {0, X-1, X*Y-X, X*Y-1};                    // this should depend on pattern size
    //int cornerIndex[4] = {0, 1, 2, 3};


    Point intermediatePt;

    Point tempPt;

    //printf("%s << DEBUG 000\n", __FUNCTION__);

    if (DEBUG_MODE > 3)
    {
        printf("%s << remainingPatches.size() = %d\n", __FUNCTION__, remainingPatches.size());
    }

    // For all 4 corners...
    for (int k = 0; k < 4; k++)
    {
        minDist = 99999999;
        // Search all remaining patches
        for (unsigned int i = 0; i < remainingPatches.size(); i++)
        {

            //printf("%s << remainingPatches.at(%d) = (%f, %f)\n", __FUNCTION__, i, remainingPatches.at(i).x, remainingPatches.at(i).y);

            pointPair.clear();
            pointPair.push_back(Point(int(extremesX[k]), int(extremesY[k])));
            pointPair.push_back(remainingPatches.at(i));
            dist = arcLength(Mat(pointPair), false); // total distance to extremes

            //printf("distance for point [%d] to corner [%d] = %f\n", i, k, dist);

            if (dist < minDist)
            {
                //printf("minimum found.\n");
                minDist = dist;
                minIndex = i;
            }

        }

        //printf("Best point for %d: %d\n", k, minIndex);
        //circle(imCpy, corners.at(minIndex), 8, color, 3, 8, 0);
        transferElement(patchCentres, remainingPatches, minIndex);

        //imshow("MSERwin", imCpy);
        //waitKey(0);
    }

    if (DEBUG_MODE > 3)
    {
        for (unsigned int i = 0; i < patchCentres.size(); i++)
        {
            printf("%s << patchCentres.at(%d) = (%f, %f)\n", __FUNCTION__, i, patchCentres.at(i).x, patchCentres.at(i).y);
        }

        printf("%s << remainingPatches.size() = %d\n", __FUNCTION__, remainingPatches.size());
    }


}

void findEdgePatches(Size patternSize, int mode, int *XVec, int *YVec, vector<Point2f>& patchCentres, vector<Point2f>& remainingPatches)
{
    int minDist, minIndex, sortedIndex = int(patchCentres.size());
    double dist;
    vector<Point2f> patchString;  // to store some patches before you splice stuff inside
    Point2f interPoint;

    //printf("sizes: %d, %d, %d\n", patchCentres.size(), patchString.size(), remainingPatches.size());

    // LHS -> Get all the edge patches
    //printf("%s << YVec[0] = %d\n", __FUNCTION__, YVec[0]);
    for (int k = 0; k < YVec[0]; k++)
    {
        // go through each remaining point
        minDist = 99999999;

        for (unsigned int i = 0; i < remainingPatches.size(); i++)
        {
            // get distance to line
            dist = perpDist(patchCentres.at(0), patchCentres.at(2), remainingPatches.at(i));
            //printf("dist (from line) [%d] = %f\n", j, dist);
            if (dist < minDist)
            {
                minDist = int(dist);
                minIndex = i;
            }
        }
        //printf("minDist (from line) [%d] = %f\n", minIndex, minDist);

        // Transfer the found patch
        transferElement(patchString, remainingPatches, minIndex);
    }

    //printf("BLEEERGH!!!\n");

    //printf("sizes: %d, %d, %d\n", patchCentres.size(), patchString.size(), remainingPatches.size());

    // LHS -> Get the edge patches in order within the string
    for (int k = 0; k < YVec[0]; k++)
    {
        // go through each remaining point
        minDist = 99999999;

        for (int i = k; i < YVec[0]; i++)
        {
            // get distance to top left point
            dist = distBetweenPts2f(patchCentres.at(0), patchString.at(i));
            //printf("dist (from line) [%d] = %f\n", j, dist);
            if (dist < minDist)
            {
                minDist = int(dist);
                minIndex = i;
            }
        }
        //printf("minDist (from line) [%d] = %f\n", minIndex, minDist);

        // switch to end of these points
        swapElements(patchString, k, minIndex);


    }

    //printf("sizes: %d, %d, %d\n", patchCentres.size(), patchString.size(), remainingPatches.size());

    // Insert patches from string and any intermediate patches
    if (mode == 0)
    {
        // Between top left corner and first patch (should be conditional)
        interPoint = meanPoint(patchCentres.at(0), patchString.at(0));
        patchCentres.push_back(interPoint);

        // between all internal-edge patches
        for (int k = 0; k < YVec[0]-1; k++)
        {
            //insertIntermediateElement(patchCentres, 4+2*k-1, 4+2*k, 4+2*k);
            interPoint = meanPoint(patchString.at(0), patchString.at(1));
            transferElement(patchCentres, patchString, 0);
            patchCentres.push_back(interPoint);

        }

        //printf("size() = %d\n", patchString.size());

        // between last on LHS and final point (should also be conditional)
        interPoint = meanPoint(patchCentres.at(2), patchString.at(0));
        transferElement(patchCentres, patchString, 0);
        patchCentres.push_back(interPoint);

    }
    else
    {
        for (int k = 0; k < YVec[0]; k++)
        {
            //printf("A sizes: %d, %d, %d\n", patchCentres.size(), patchString.size(), remainingPatches.size());

            transferElement(patchCentres, patchString, 0);
        }

    }

    //printf("sizes: %d, %d, %d\n", patchCentres.size(), patchString.size(), remainingPatches.size());

    sortedIndex += YVec[0];

    //printf("SNARF!!!\n");

    // RHS -> Get all the edge patches
    //printf("%s << YVec[1] = %d\n", __FUNCTION__, YVec[1]);
    for (int k = 0; k < YVec[1]; k++)
    {
        // go through each remaining point
        minDist = 99999999;

        for (unsigned int i = 0; i < remainingPatches.size(); i++)
        {
            // get distance to line
            dist = perpDist(patchCentres.at(1), patchCentres.at(3), remainingPatches.at(i));
            //printf("dist (from line) [%d] = %f\n", j, dist);
            if (dist < minDist)
            {
                minDist = int(dist);
                minIndex = i;
            }
        }
        //printf("minDist (from line) [%d] = %f\n", minIndex, minDist);

        // switch to start of vector
        transferElement(patchString, remainingPatches, minIndex);
    }

    //printf("SCHMIMF!!!\n");

    // RHS -> Get the edge patches in order
    for (int k = 0; k < YVec[1]; k++)
    {
        // go through each remaining point
        minDist = 99999999;

        for (int i = k; i < YVec[1]; i++)
        {

            //printf("k = %d; i = %d, size = %d\n", k, i, patchString.size());

            // get distance to top left point
            dist = distBetweenPts2f(patchCentres.at(1), patchString.at(i));
            //printf("dist (from line) [%d] = %f\n", j, dist);
            if (dist < minDist)
            {
                minDist = int(dist);
                minIndex = i;
            }
        }
        //printf("minDist (from line) [%d] = %f\n", minIndex, minDist);

        // switch to end of these points
        swapElements(patchString, k, minIndex);
    }

    //printf("PREFROO!!!!\n");

    if (mode == 0)
    {
        // Between top left corner and first patch (should be conditional)
        interPoint = meanPoint(patchCentres.at(1), patchString.at(0));
        patchCentres.push_back(interPoint);

        // between all internal-edge patches
        for (int k = 0; k < YVec[1]-1; k++)
        {
            //insertIntermediateElement(patchCentres, 4+2*k-1, 4+2*k, 4+2*k);
            interPoint = meanPoint(patchString.at(0), patchString.at(1));
            transferElement(patchCentres, patchString, 0);
            patchCentres.push_back(interPoint);

        }

        //printf("size() = %d\n", patchString.size());

        // between last on LHS and final point (should also be conditional)
        interPoint = meanPoint(patchCentres.at(3), patchString.at(0));
        transferElement(patchCentres, patchString, 0);
        patchCentres.push_back(interPoint);

    }
    else
    {
        for (int k = 0; k < YVec[1]; k++)
        {
            //printf("A sizes: %d, %d, %d\n", patchCentres.size(), patchString.size(), remainingPatches.size());

            transferElement(patchCentres, patchString, 0);
        }

    }

    sortedIndex += YVec[1];

    //printf("PWOGGG!!!\n");

    if (DEBUG_MODE > 3)
    {

        printf("%s << patchCentres.size() = %d\n", __FUNCTION__, patchCentres.size());
        printf("%s << remainingPatches.size() = %d\n", __FUNCTION__, remainingPatches.size());

        for (unsigned int i = 0; i < patchCentres.size(); i++)
        {
            printf("%s << patchCentres.at(%d) = (%f, %f)\n", __FUNCTION__, i, patchCentres.at(i).x, patchCentres.at(i).y);
        }


    }
}

void findInteriorPatches(Size patternSize, int mode, int *XVec, int *YVec, vector<Point2f>& patchCentres, vector<Point2f>& remainingPatches)
{
    // Temporary:

    int minDist, minIndex = 0, sortedIndex = int(patchCentres.size());
    double dist;

    vector<Point2f> patchString;  // to store some patches before you splice stuff inside
    Point2f interPoint;

    int nRows;

    if (mode == 0)
    {
        nRows = patternSize.height + 1;
    }
    else
    {
        nRows = patternSize.height / 2;
    }

    //printf("%s << nRows = %d\n", __FUNCTION__, nRows);

    // For each row:
    for (int r = 0; r < nRows; r++)
    {

        patchString.clear();

        //printf("%s << XVec[%d] = %d\n", __FUNCTION__, r, XVec[r]);

        // For each patch that is present in that row
        for (int k = 0; k < XVec[r]; k++)
        {
            minDist = 99999999;

            //printf("%s << row[%d] k = %d, XVec[r] = %d\n", __FUNCTION__, r, k, XVec[r]);

            for (unsigned int i = 0; i < remainingPatches.size(); i++)
            {
                if (r == 0)     // First Row
                {
                    dist = perpDist(patchCentres.at(0), patchCentres.at(1), remainingPatches.at(i));
                }
                else if (r == (nRows-1))      // Last Row
                {
                    dist = perpDist(patchCentres.at(2), patchCentres.at(3), remainingPatches.at(i));
                }
                else        // Other Rows
                {
                    dist = perpDist(patchCentres.at(4+r-1), patchCentres.at(4+nRows-2+r-1), remainingPatches.at(i));
                }

                if (dist < minDist)
                {
                    minDist = int(dist);
                    minIndex = i;
                }

            }

            transferElement(patchString, remainingPatches, minIndex);
        }

        //printf("BLEEERGH!!! [%d]\n", r);

        // try to sort the patchString elements
        for (int k = 0; k < XVec[r]; k++)
        {
            minDist = 99999999;

            for (int i = k; i < XVec[r]; i++)
            {
                if (r == 0)     // First Row
                {
                    dist = distBetweenPts2f(patchCentres.at(0), patchString.at(i));
                }
                else if (r == (nRows-1))      // Last Row
                {
                    dist = distBetweenPts2f(patchCentres.at(2), patchString.at(i));
                }
                else        // Other Rows
                {
                    dist = distBetweenPts2f(patchCentres.at(4+r-1), patchString.at(i));
                }

                //printf("%s << row[%d] k = %d, i = %d, dist = %f\n", __FUNCTION__, r, k, i, dist);

                if (dist < minDist)
                {
                    minDist = int(dist);
                    minIndex = i;
                }
            }

            //printf("%s << moving %d to %d\n", __FUNCTION__, minIndex, k);

            swapElements(patchString, k, minIndex);
        }

        for (int k = 0; k < XVec[r]; k++)
        {
            minDist = 99999999;

            dist = distBetweenPts2f(patchCentres.at(0), patchString.at(k));

            //printf("%s << dist[%d] = %f\n", __FUNCTION__, k, dist);
            //printf("%s << at(%d,%d)\n", __FUNCTION__, patchString.at(k).x, patchString.at(k).y);
        }

        //sortedIndex += XVec[r];

        if (mode == 0)
        {
            // Between left point and first patch (should be conditional)
            if (r == 0)     // First Row
            {
                interPoint = meanPoint(patchCentres.at(0), patchString.at(0));
                patchCentres.push_back(interPoint);
            }
            else if (r == nRows-1)      // Last Row
            {
                interPoint = meanPoint(patchCentres.at(2), patchString.at(0));
                patchCentres.push_back(interPoint);

            }
            else        // Other Rows
            {
                if (r % 2)
                {

                }
                else
                {
                    interPoint = meanPoint(patchCentres.at(4+r-1), patchString.at(0));
                    patchCentres.push_back(interPoint);
                }

            }


            for (int k = 0; k < XVec[r]-1; k++)
            {
                interPoint = meanPoint(patchString.at(0), patchString.at(1));
                transferElement(patchCentres, patchString, 0);
                patchCentres.push_back(interPoint);

            }

            // between all internal-edge patches

            //printf("size() = %d\n", patchString.size());

            // between last on LHS and final point (should also be conditional)
            if (r == 0)     // First Row
            {
                interPoint = meanPoint(patchCentres.at(1), patchString.at(0));
                transferElement(patchCentres, patchString, 0);
                patchCentres.push_back(interPoint);
            }
            else if (r == (nRows-1))      // Last Row
            {
                interPoint = meanPoint(patchCentres.at(3), patchString.at(0));
                transferElement(patchCentres, patchString, 0);
                patchCentres.push_back(interPoint);
            }
            else        // Other Rows
            {
                if (r % 2)
                {
                    transferElement(patchCentres, patchString, 0);
                }
                else
                {
                    interPoint = meanPoint(patchCentres.at(4+r-1+nRows-2), patchString.at(0));
                    transferElement(patchCentres, patchString, 0);
                    patchCentres.push_back(interPoint);
                }

            }

        }
        else
        {
            for (int k = 0; k < XVec[r]; k++)
            {
                //printf("A sizes: %d, %d, %d\n", patchCentres.size(), patchString.size(), remainingPatches.size());
                transferElement(patchCentres, patchString, 0);
            }

        }
    }



    /*
    while (remainingPatches.size() > 0) {
        transferElement(patchCentres, remainingPatches, 0);
    }
    */

    if (DEBUG_MODE > 3)
    {

        printf("%s << patchCentres.size() = %d\n", __FUNCTION__, patchCentres.size());
        printf("%s << remainingPatches.size() = %d\n", __FUNCTION__, remainingPatches.size());

        for (unsigned int i = 0; i < patchCentres.size(); i++)
        {
            //printf("%s << patchCentres.at(%d) = (%d, %d)\n", __FUNCTION__, i, patchCentres.at(i).x, patchCentres.at(i).y);
        }


    }

}

void sortPatches(Size imageSize, Size patternSize, vector<Point2f>& patchCentres, int mode)
{
    // TODO:
    // See the corresponding sections.

    int desiredPatchCount, X, Y;

    if (mode == 0)
    {
        desiredPatchCount = (patternSize.width + 1)*(patternSize.height + 1);
    }
    else
    {
        desiredPatchCount = (patternSize.width / 2)*(patternSize.height / 2);
    }

    if (DEBUG_MODE > 3)
    {
        printf("%s << Looking for [%d] patches..\n", __FUNCTION__, desiredPatchCount);
        //printf("%s << Configuring for mode = %d\n", __FUNCTION__, mode);
    }

    // Determine the number of "findable" patches in each row and at edge columns
    int *XVec;
    int YVec[2];

    // XVec tells you how many 'black' patches to look for between extremes in that row
    if (mode == 0)
    {
        Y = patternSize.height + 1;
        XVec = new int[Y];
    }
    else
    {
        X = patternSize.width/2;
        Y = patternSize.height/2;
        XVec = new int[Y];
    }

    determineFindablePatches(patternSize, mode, XVec, YVec);

    // Copy input patchCentres to remainingPatches vector, and clear patchCentres
    vector<Point2f> remainingPatches;
    remainingPatches.clear();
    patchCentres.swap(remainingPatches);

    // Find the 4 corners of the pattern (adding virtual points as necessary)
    findCornerPatches(imageSize, patternSize, mode, XVec, YVec, patchCentres, remainingPatches);

    // Find the LHS edge points and RHS edge points (adding virtual points as necessary)
    findEdgePatches(patternSize, mode, XVec, YVec, patchCentres, remainingPatches);

    // Find (or simulate) interior patches
    findInteriorPatches(patternSize, mode, XVec, YVec, patchCentres, remainingPatches);

    // Check that there are no remaining patches and that there are the correct no. of patches
    if (remainingPatches.size() > 0)
    {
        printf("%s << ERROR. There are remaining patches when there shouldn't be.\n", __FUNCTION__);

        /*
        patchCentres.clear();
        return;
        */
    }

    if (patchCentres.size() != desiredPatchCount)
    {
        printf("%s << ERROR. An incorrect number (%d vs %d) of patches have been allocated.\n", __FUNCTION__, patchCentres.size(), desiredPatchCount);

        //patchCentres.clear();

        // temp for testing:
        while (patchCentres.size() < (unsigned int)(desiredPatchCount))
        {
            patchCentres.push_back(Point(0,0));
        }

        delete[] XVec;

        return;
    }

    // Remap order of patches to match OpenCV format
    //printf("GlERP!!!\n");
    reorderPatches(patternSize, mode, XVec, YVec, patchCentres);

    delete[] XVec;
}

void reorderPatches(Size patternSize, int mode, int *XVec, int *YVec, vector<Point2f>& patchCentres)
{
    // TODO:
    // Nothing much.

    //printf("GWanGLE!!!\n");

    //int X = patternSize.width/2, Y = patternSize.height/2;
    vector<Point2f> reSortedCorners;

    int nRows, nCols, nOffset;

    //printf("FlAng!!!\n");

    if (mode == 0)
    {
        nRows = patternSize.height + 1;
        nCols = patternSize.width + 1;
    }
    else
    {
        nRows = patternSize.height / 2;
        nCols = patternSize.width / 2;
    }

    //printf("Pring!!!\n");

    nOffset = 4+2*(nRows-2);

    //printf("%s << nRows = %d; nCols = %d; nOffset = %d\n", __FUNCTION__, nRows, nCols, nOffset);

    // For each row
    for (int r = 0; r < nRows; r++)
    {
        // First element of row
        if (r == 0)     // if first row
        {
            reSortedCorners.push_back(patchCentres.at(0));
        }
        else if (r == (nRows-1))        // if last row
        {
            reSortedCorners.push_back(patchCentres.at(2));
        }
        else        // if other row
        {
            reSortedCorners.push_back(patchCentres.at(4+r-1));
        }

        // In between
        for (int i = 0; i < nCols-2; i++)
        {
            //printf("%s << row [%d] current Index = %d\n", __FUNCTION__, r, nOffset + r*(nCols-2) + i);
            reSortedCorners.push_back(patchCentres.at(nOffset + r*(nCols-2) + i));
        }


        // Last element of row
        if (r == 0)     // if first row
        {
            reSortedCorners.push_back(patchCentres.at(1));
        }
        else if (r == (nRows-1))        // if last row
        {
            reSortedCorners.push_back(patchCentres.at(3));
        }
        else        // if other row
        {
            reSortedCorners.push_back(patchCentres.at(4+nRows-2+r-1));
        }
    }

    reSortedCorners.swap(patchCentres);

    if (DEBUG_MODE > 3)
    {

        printf("%s << patchCentres.size() = %d\n", __FUNCTION__, patchCentres.size());

        for (unsigned int i = 0; i < patchCentres.size(); i++)
        {
            //printf("%s << patchCentres.at(%d) = (%d, %d)\n", __FUNCTION__, i, patchCentres.at(i).x, patchCentres.at(i).y);
        }


    }

}

void debugDisplayPattern(const Mat& image, Size patternSize, Mat& corners, bool mode, double delay)
{

    Mat tmpMat, dispMat;

    mode = false;

    if (image.channels() > 1)
    {
        cvtColor(image, tmpMat, CV_RGB2GRAY);
    }
    else
    {
        image.copyTo(tmpMat);
    }

    cvtColor(tmpMat, dispMat, CV_GRAY2RGB);



    drawChessboardCorners(dispMat, patternSize, corners, mode);

    if (image.cols > 640)
    {
        Mat dispMat2;
        resize(dispMat, dispMat2, Size(0,0), 0.5, 0.5);
        imshow("mainWin", dispMat2);
    }
    else
    {
        imshow("mainWin", dispMat);
    }
    waitKey(int(delay));
}

bool findPatternCorners(const Mat& image, Size patternSize, vector<Point2f>& corners, int mode, int detector, int mserDelta, float max_var, float min_div, double area_threshold)
{
    // mode 0: MSER chessboard finder
    // mode 1: MSER mask finder

    if (!checkAcutance())
    {
        return false;
    }

    int patchCols, patchRows, desiredPatchQuantity;
    determinePatchDistribution(patternSize, mode, patchRows, patchCols, desiredPatchQuantity);

    vector<vector<Point> > msers;
    //cout << "BETA" << endl;
    findAllPatches(image, patternSize, msers, mserDelta, max_var, min_div, area_threshold);
        //cout << "GAMMA" << endl;
    if (DEBUG_MODE > 2)
    {
        debugDisplayPatches(image, msers);
    }

    if (int(msers.size()) < desiredPatchQuantity)
    {
        corners.clear();
        if (DEBUG_MODE > 1)
        {
            printf("%s << Insufficient patches found. Returning.\n", __FUNCTION__);
        }

        return false;
    }



    vector<Point2f> patchCentres2f;
    bool found = refinePatches(image, patternSize, msers, patchCentres2f, mode);


    if (DEBUG_MODE > 1)
    {
        printf("%s << Patches found after refinement = %d\n", __FUNCTION__, msers.size());
    }

    if (DEBUG_MODE > 2)
    {
        debugDisplayPatches(image, msers);
    }



    // If patches still not found...
    if (!found)
    {
        corners.clear();

        if (DEBUG_MODE > 1)
        {
            printf("%s << Correct number of patches not found. Returning.\n", __FUNCTION__);
        }

        return false;
    }


    found = patternInFrame(image.size(), patchCentres2f);

    // If patches still not found...
    if (!found)
    {
        corners.clear();

        if (DEBUG_MODE > 1)
        {
            printf("%s << Some patch centres out of range...\n", __FUNCTION__);
        }

        return false;
    }

    sortPatches(image.size(), patternSize, patchCentres2f, mode);

    if (DEBUG_MODE > 2)
    {
        Mat patchCentres_(patchCentres2f);
        debugDisplayPattern(image, cvSize(patchCols, patchRows), patchCentres_);
    }

    found = verifyPatches(image.size(), patternSize, patchCentres2f, mode, MIN_CORNER_SEPARATION, MAX_CORNER_SEPARATION);

    if (!found)
    {
        corners.clear();

        if (DEBUG_MODE > 1)
        {
            printf("%s << Pattern verification failed. Returning.\n", __FUNCTION__);
        }

        return false;
    }

    // Correct patch centres (using histogram equalisation, single-frame calibration)
    correctPatchCentres(image, patternSize, patchCentres2f, mode);

    if (DEBUG_MODE > 2)
    {
        Mat patchCentres_(patchCentres2f);
        debugDisplayPattern(image, cvSize(patchCols, patchRows), patchCentres_);
    }

    Mat homography;
    found = findPatchCorners(image, patternSize, homography, corners, patchCentres2f, mode, detector);

    // refineCornerPositions
    //fixFourCorners(image, corners, patternSize);

    if (!found)
    {
        corners.clear();

        if (DEBUG_MODE > 1)
        {
            printf("%s << Patch corner search failed. Returning.\n", __FUNCTION__);
        }

        return false;
    }

    found = patternInFrame(image.size(), patchCentres2f);

    // If patches still not found...
    if (!found)
    {
        corners.clear();

        if (DEBUG_MODE > 1)
        {
            printf("%s << Some corners out of range...\n", __FUNCTION__);
        }

        return false;
    }

    if (DEBUG_MODE > 2) {
        Mat cornersMat(corners);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersMat, true, 1.0);
    }

    return found;
}


void interpolateCornerLocations2(const Mat& image, int mode, Size patternSize, vector<Point2f>& vCentres, vector<Point2f>& vCorners)
{

    if (DEBUG_MODE > 2)
    {
        printf("%s << Entered function...\n", __FUNCTION__);
    }

    double minDimension = findMinimumSeparation(vCentres);

    int correctionDistance = int(minDimension) / 4;

    //printf("%s << minDimension = %f, correctionDistance = %d\n", __FUNCTION__, minDimension, correctionDistance);

    Mat imGrey;

    if (image.channels() > 1)
    {
        cvtColor(image, imGrey, CV_RGB2GRAY);
    }
    else
    {
        image.copyTo(imGrey);
    }

    int X, Y;

    X = patternSize.width/2;
    Y = patternSize.height/2;

    vector<Point2f> fullCentroidGrid;

    // Sets up all centroid grid points
    for (int i = 0; i < Y; i++)
    {
        for (int j = 0; j < X; j++)
        {
            Point2f pt;

            pt = Point2f(float(j), float(i));
            fullCentroidGrid.push_back(pt);

        }
    }

    vector<Point2f> fullCornerGrid;

    // Sets up all corner grid points
    for (int i = 0; i < Y; i++)
    {
        for (int j = 0; j < X; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                Point2f pt;

                if (k == 0)
                {
                    pt = Point2f(float(j-0.25), float(i-0.25));
                }
                else if (k == 1)
                {
                    pt = Point2f(float(j+0.25), float(i-0.25));
                }
                else if (k == 2)
                {
                    pt = Point2f(float(j-0.25), float(i+0.25));
                }
                else if (k == 3)
                {
                    pt = Point2f(float(j+0.25), float(i+0.25));
                }

                fullCornerGrid.push_back(pt);
            }

        }
    }

    vector<Point2f> patchNeighbourhood;
    vector<Point2f> patchArrangement;
    vector<Point2f> newCorners, bestCorners;
    vector<Point2f> cornerArrangement;
    Mat cornerLocs(2, 2, CV_32FC2);
    //Point2f *neighbourhoodArray;
    //Point2f* arrangementArray;

    Mat homography;

    // Depending on the location of the patch, a different number of neighbouring patches
    // are used to determine an approximate homography
    int neighbourhoodCount = 0;

    int index = 0;

    // Initial estimate of all corners
    for (int i = 0; i < Y; i++)
    {
        for (int j = 0; j < X; j++)
        {

            if (0)   //((i == 0) || (i == Y-1) || (j == 0) || (j == X-1)) {
            {
                for (int k = 0; k < 4; k++)
                {
                    newCorners.push_back(Point2f(0.0, 0.0));    // skip corners and edges
                    index++;
                }

            }
            else
            {
                patchNeighbourhood.clear();
                patchArrangement.clear();

                neighbourhoodCount = 0;

                // For each neighbourhood patch
                for (int k = max(0, i-1); k < min(Y, i+2); k++)             // try to go 1 up and down
                {
                    for (int l = max(0, j-1); l < min(X, j+2); l++)         // try to go 1 left and right
                    {
                        // Add patch to patch vector
                        patchNeighbourhood.push_back(vCentres.at(k*X+l));
                        // Add co-ordinate to arbitrary map vector
                        patchArrangement.push_back(Point2f((float)l, (float)k));

                        neighbourhoodCount++;
                    }
                }

                //neighbourhoodArray = new Point2f[neighbourhoodCount];
                //arrangementArray = new Point2f[neighbourhoodCount];

                /*
                for (int k = 0; k < neighbourhoodCount; k++) {
                    neighbourhoodArray[k] = patchNeighbourhood.at(k);
                    arrangementArray[k] = patchArrangement.at(k);
                }
                */

                // find homography
                homography = findHomography(Mat(patchArrangement), Mat(patchNeighbourhood));

                cornerArrangement.clear();

                // define arbitrary corner co-ordinate
                for (int k = 0; k < 2; k++)
                {
                    for (int l = 0; l < 2; l++)
                    {
                        cornerArrangement.push_back(Point2f((float)(j-0.25+0.5*l), (float)(i-0.25+0.5*k)));
                    }
                }


                // apply homography to these co-ordinates
                Mat tmpMat1 = Mat(cornerArrangement);

                perspectiveTransform(tmpMat1, cornerLocs, homography);

                for (int k = 0; k < 4; k++)
                {
                    //printf("%s << adding..\n", __FUNCTION__);
                    newCorners.push_back(Point2f(cornerLocs.at<Vec3f>(k,0)[0], cornerLocs.at<Vec3f>(k,0)[1]));
                }

            }




        }
    }

    /*
    if (DEBUG_MODE > 2) {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);
        printf("%s << DONE.\n", __FUNCTION__);
    }
    */
    //printf("%s << pre cSP\n", __FUNCTION__);
    //cornerSubPix(imGrey, newCorners, Size(correctionDistance, correctionDistance), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));


    if (DEBUG_MODE > 2)
    {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);
        printf("%s << DONE.\n", __FUNCTION__);
    }

    vCorners.clear();
    vCorners.assign(newCorners.begin(), newCorners.end());


}

void addToBinMap(Mat& binMap, cv::vector<Point2f>& cornerSet, Size imSize)
{
    int x, y;
    for (unsigned int i = 0; i < cornerSet.size(); i++)
    {
        // Determine Bin Index for this specific corner
        x = (int)((cornerSet.at(i).x/(double(imSize.width)))*binMap.cols);
        y = (int)((cornerSet.at(i).y/(double(imSize.height)))*binMap.rows);

        // Increment the count for this bin
        binMap.at<int>(y, x) += 1;
    }
}

void prepForDisplay(const Mat& distributionMap, Mat& distributionDisplay)
{

    cv::Mat distMapCpy(distributionMap.size(), CV_8UC3), distMapCpyGray(distributionMap.size(), CV_8UC1);
    distributionDisplay = cv::Mat(distributionMap.size(), CV_8UC1);

    // Temp
    //distributionMap.copyTo(distMapCpy); // distMapCpy should be 0s and 1s
    
    distMapCpy = cv::Mat::zeros(distributionMap.size(), CV_8UC3);
	
	Scalar markerColor(0,0,0);
	
    for (int iii = 0; iii < distributionMap.rows; iii++) {
		for (int jjj = 0; jjj < distributionMap.cols; jjj++) {
			
			//printf("%s << distMapCpy(%d, %d) = (%d)\n", __FUNCTION__, iii, jjj, distMapCpy.at<unsigned char>(iii,jjj));
			
			distMapCpy.at<Vec3b>(iii,jjj)[0] = 255;
			distMapCpy.at<Vec3b>(iii,jjj)[1] = 255;
			distMapCpy.at<Vec3b>(iii,jjj)[2] = 255;
			
			// distMapCpy.at<unsigned char>(iii,jjj) = 255 - distMapCpy.at<unsigned char>(iii,jjj);
			
			if (distributionMap.at<unsigned char>(iii,jjj) > 0) {
				unsigned char val = 0; // max(0, 255-10*int(distributionMap.at<unsigned char>(iii,jjj)));
				markerColor = Scalar(val,val,val);
				circle(distMapCpy, Point(jjj,iii), 0, markerColor, 5, CV_AA, 0);
			}
			
			
			
			
			
		}
	}
	
	
	
    cvtColor(distMapCpy, distMapCpyGray, CV_RGB2GRAY);
    
    
    //distMapCpy += 100;
    equalizeHist(distMapCpyGray, distributionDisplay);  // distDisplay should be 0s and 255s
    
    //imshow("a", distMapCpy);
    //waitKey();
    
    GaussianBlur(distributionDisplay, distMapCpyGray, Size(11, 11), 1.0, 1.0); // distMapCpy should range from 0 to 255
    
    //imshow("a", distMapCpy);
    //waitKey();
   
    //equalizeHist(distMapCpy, distributionDisplay);  // distMapCpy should range from 0 to 255
    normalize(distMapCpyGray, distributionDisplay, 0,255, NORM_MINMAX);

	//imshow("a", distMapCpy);
    //waitKey();
	
    //distMapCpy.copyTo(distributionDisplay);

    return;
}

void addToRadialDistribution(double *radialDistribution, cv::vector<Point2f>& cornerSet, Size imSize)
{

    Point2f center((float)((double(imSize.height-1))/2), (float)((double(imSize.width-1))/2));
    double dist, maxDist;
    int index;

    //maxDist = max(double(imSize.height)/2, double(imSize.width)/2);

    // Really max dist should be larger than half the major axis, although you don't know how large it really
    // needs to be until after you've calibrated..
    maxDist = pow(pow(double(imSize.height)/2, 2) + pow(double(imSize.width)/2, 2), 0.5);

    for (unsigned int i = 0; i < cornerSet.size(); i++)
    {

        dist = distBetweenPts2f(center, cornerSet.at(i));

        // Only add the point to the distribution if it's within the desired range
        if (dist < maxDist)
        {
            index = int((dist/maxDist)*(RADIAL_LENGTH-0.00001));
            radialDistribution[index]++;
        }

    }

}

void addToDistributionMap(Mat& distributionMap, vector<Point2f>& corners)
{
    for (unsigned int i = 0; i < corners.size(); i++)
    {
        distributionMap.at<unsigned char>((int)corners.at(i).y, (int)corners.at(i).x) += (unsigned char) 1;
    }
}

double obtainSetScore(Mat& distributionMap,
                      Mat& binMap,
                      Mat& gaussianMat,
                      cv::vector<Point2f>& cornerSet,
                      double *radialDistribution)
{
    double score = 1.0;
    double mean = 0.0, variance = 0.0;
    int count = 0;
    double area = 0.0;
    Point centroid, center;
    double centrality = 0.0;

    Mat distributionDisplay(distributionMap.size(), CV_8UC1);
    Mat binTemp(binMap.size(), CV_8UC1);

    center = Point(distributionMap.size().width/2 - 1, distributionMap.size().height/2 - 1);

    cv::vector<Point> fullHull, simplifiedHull;

    for (unsigned int i = 0; i < cornerSet.size(); i++)
    {
        fullHull.push_back(Point(int(cornerSet.at(i).x), int(cornerSet.at(i).y)));
    }

    // Create a copy of bin Map
    Mat binMapCpy;
    binMap.copyTo(binMapCpy);

    // Add corners to copy binMap
    addToBinMap(binMapCpy, cornerSet, distributionMap.size());

    count = binMapCpy.size().width * binMapCpy.size().height;

    // SCORING CONTRIBUTIONS
    // Area as Fraction of FOV:
    convexHull(Mat(fullHull), simplifiedHull);

    area = contourArea(Mat(simplifiedHull));

    area /= (distributionMap.size().width * distributionMap.size().height);

    //printf("%s << area as fraction = %f\n", __FUNCTION__, area);

    // Centrality
    centroid = findCentroid(simplifiedHull);
    centrality = distBetweenPts(centroid, center);
    centrality = 1 - (centrality / (distributionMap.size().height/2));

    // Calculate mean value per bin for spreadiness
    for (int i = 0; i < binMapCpy.size().width; i++)
    {
        for (int j = 0; j < binMapCpy.size().height; j++)
        {
            mean += binMapCpy.at<int>(j, i);
        }
    }

    // Radial Distribution

    //printf("%s << DEBUG %d\n", __FUNCTION__, 0);

    // Make copy of radial distribution
    double *radialDistCpy;
    radialDistCpy = new double[RADIAL_LENGTH];
    for (int i = 0; i < RADIAL_LENGTH; i++)
    {
        radialDistCpy[i] = radialDistribution[i];
    }

    // Add current pointset to radial distribution copy
    addToRadialDistribution(radialDistCpy, cornerSet, distributionMap.size());

    double radialMean = 0.0, radialVariance = 0.0, desiredCount;
    double *radialCumulative;
    radialCumulative = new double[RADIAL_LENGTH];

    // Calculate cumulative Distribution and mean (avg points per quantization bin)
    //printf("debugging radialCumulative: ");
    for (int i = 0; i < RADIAL_LENGTH; i++)
    {
        radialMean += radialDistCpy[i];
        radialCumulative[i] = radialMean;
        //printf("%f\t", radialCumulative[i]);
    }
    radialMean /= RADIAL_LENGTH;
    //printf("\n");
    //cin.get();

    // Calculate the variation from an ideal/uniform distribution with the new pointset included
    //printf("debugging radialVariance: ");
    for (int i = 0; i < RADIAL_LENGTH; i++)
    {
        desiredCount = double(i+1)*radialMean;
        // Variance is the sum of the differences of where the cumulative count is for each bin, and where it
        // should be if it's a uniform distribution over the full radial range
        radialVariance += pow(radialCumulative[i] - desiredCount, 2);
        //printf("%f\t", pow(radialCumulative[i] - desiredCount, 2));

        //radialVariance += pow(radialDistCpy[i]-radialMean, 2);
    }

    //printf("\n");


    //printf("%s << DEBUG %d\n", __FUNCTION__, 4);

    // Turn variance count into standard deviation
    radialVariance /= RADIAL_LENGTH;
    radialVariance = pow(radialVariance, 0.5);

    //printf("radialVariance = %f\n", radialVariance);    // around about 5 - 20...?
    //cin.get();

    mean /= count;

    //printf("mean = %f\n", mean);

    // NEW
    // ===================
    // Add the test points to a double precision copy of the distribution map
    Mat distMapCpy(distributionMap.size(), CV_64FC1);

    for (int i = 0; i < distMapCpy.size().width; i++)
    {
        for (int j = 0; j < distMapCpy.size().height; j++)
        {
            distMapCpy.at<double>(j,i) += double(distributionMap.at<unsigned char>(j,i));
        }
    }

    //printf("%s << DEBUG %d\n", __FUNCTION__, 5);

    // Massively blur it
    // or could you just convolve it with optimum distribution to find correlation?

    // Quantize it to same bins as gassian map
    // Then compare its variance with the gaussian distribution map
    // ===================


    // OLD
    // ===================

    // Scale gaussian mat up
    gaussianMat *= mean;

    for (int i = 0; i < binMapCpy.size().width; i++)
    {
        for (int j = 0; j < binMapCpy.size().height; j++)
        {
            variance += pow((double(binMapCpy.at<int>(j, i)) - gaussianMat.at<double>(j, i)), 2);
        }
    }

    // Scale gaussian mat back
    gaussianMat /= mean;
    variance /= count;
    // ===================

    /*
    printf("%s << area = %f\n", __FUNCTION__, area);
    printf("%s << centrality = %f\n", __FUNCTION__, centrality);
    printf("%s << radialVariance = %f\n", __FUNCTION__, radialVariance);
    cin.get();
    */

    // Figure out total score
    //score = area;
    //score = double(pow(area, 1.0)) * variance;
    //score = (area*centrality) / radialVariance;
    //score = area / radialVariance;
    //score = area / radialVariance;
    //score = 1 / radialVariance;
    //score = centrality * area;
    score = pow(area, 2.0) / radialVariance;
    //score = area + (1 / radialVariance);

    //printf("%s << score = %f [centrality = %f; area = %f\n", __FUNCTION__, score, centrality, area);

    delete[] radialDistCpy;
    delete[] radialCumulative;

    if (score < 0)
    {
        printf("%s << ERROR: Negative score acquired. Returning.\n", __FUNCTION__);
        return -1;
    }

    return score;
}

bool findPatchCorners(const Mat& image, Size patternSize, Mat& homography, vector<Point2f>& corners, vector<Point2f>& patchCentres2f, int mode, int detector)
{

    if (DEBUG_MODE > 2)
    {
        printf("%s << Entered function...\n", __FUNCTION__);
    }

    Mat inputDisp;

    vector<Point2f> cornerEstimates;
    vector<vector<Point2f> > vvOriginalCentres;
    vvOriginalCentres.push_back(patchCentres2f);

    interpolateCornerLocations2(image, mode, patternSize, vvOriginalCentres.at(0), cornerEstimates);

    //estimateUnknownPositions(XXX, vvOriginalCentres.at(0), XXX, cornerEstimates);

    if (DEBUG_MODE > 2)
    {
        Mat cornersForDisplay(cornerEstimates);
        printf("%s << Step 3: cornerEstimates.size() = %d\n", __FUNCTION__, cornerEstimates.size());
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);
        printf("%s << DONE.\n", __FUNCTION__);
    }

    corners.clear();

    //vector<Point2f> innerCornerEstimates;
    //cornerEstimates.copyTo(innerCornerEstimates);

    sortCorners(image.size(), patternSize, cornerEstimates);

    findBestCorners(image, cornerEstimates, corners, patternSize, detector);

    //correctInnerCorners(innerCornerEstimates, cornerEstimates, )

    if (DEBUG_MODE > 3)
    {
        Mat cornersForDisplay(corners);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);
    }

    bool retVal = false;

    // sortCorners(image.size(), patternSize, corners);

    retVal = verifyCorners(image.size(), patternSize, corners, MIN_CORNER_SEPARATION, MAX_CORNER_SEPARATION);

    if (!retVal)
    {
        corners.clear();
        return retVal;
    }

    groupPointsInQuads(patternSize, corners);
    
    if (DEBUG_MODE > 3) {
		printf("%s << Displaying corners after grouping...\n", __FUNCTION__);
        Mat cornersForDisplay(corners);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay, true, 50.0);
    }
    
    refineCornerPositions(image, patternSize, corners);
    
    if (DEBUG_MODE > 3) {
		printf("%s << Displaying corners after refinement...\n", __FUNCTION__);
        Mat cornersForDisplay(corners);        
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay, true, 50.0);
    }
    
    sortCorners(image.size(), patternSize, corners);
    
    if (DEBUG_MODE > 3) {
		printf("%s << Displaying corners after sorting...\n", __FUNCTION__);
        Mat cornersForDisplay(corners);        
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay, true, 50.0);
    }

    // 6. verify...

    //printf("%s << patternSize = (%d, %d)\n", __FUNCTION__, patternSize.width, patternSize.height);
    retVal = verifyCorners(image.size(), patternSize, corners, MIN_CORNER_SEPARATION, MAX_CORNER_SEPARATION);

    if (!retVal)
    {
		printf("%s << Pattern verification failed.\n", __FUNCTION__);
        corners.clear();
    } else {
		printf("%s << Pattern verification succeeded.\n", __FUNCTION__);
	}

    return retVal;

}

bool verifyCorners(Size imSize, Size patternSize, vector<Point2f>& patternPoints, double minDist, double maxDist)
{
    vector<Point2f> simplePoints;

    for (unsigned int i = 0; i < patternPoints.size(); i++)
    {
        simplePoints.push_back(Point(int(patternPoints.at(i).x), int(patternPoints.at(i).y)));
    }

    return verifyPattern(imSize, patternSize, simplePoints, minDist, maxDist);
}

void refineCornerPositions(const Mat& image, Size patternSize, vector<Point2f>& vCorners)
{

    Mat imGrey;

    vector<Point2f> targetCorner;

    if (image.channels() > 1)
    {
        cvtColor(image, imGrey, CV_RGB2GRAY);
    }
    else
    {
        image.copyTo(imGrey);
    }

    vector<Point2f> neighbouringPts;

    int X, Y;

    X = patternSize.width/2;
    Y = patternSize.height/2;

    vector<Point2f> fullCornerGrid;

    double minDimension = findMinimumSeparation(vCorners);

    int correctionDistance;
    //printf("%s << correctionDistance = %d\n", __FUNCTION__, correctionDistance);
    //cin.get();

    // Sets up all corner grid points
    for (int i = 0; i < Y; i++)
    {
        for (int j = 0; j < X; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                Point2f pt;

                if (k == 0)
                {
                    pt = Point2f(float(j-0.25), float(i-0.25));
                }
                else if (k == 1)
                {
                    pt = Point2f(float(j+0.25), float(i-0.25));
                }
                else if (k == 2)
                {
                    pt = Point2f(float(j-0.25), float(i+0.25));
                }
                else if (k == 3)
                {
                    pt = Point2f(float(j+0.25), float(i+0.25));
                }

                fullCornerGrid.push_back(pt);
            }

        }
    }

    vector<Point2f> patchNeighbourhood;
    vector<Point2f> patchArrangement;
    vector<Point2f> newCorners, bestCorners;
    vector<Point2f> cornerArrangement;
    Mat cornerLocs(2, 2, CV_32FC2);
    //Point2f *neighbourhoodArray;
    //Point2f* arrangementArray;

    Mat homography;

    // Depending on the location of the patch, a different number of neighbouring patches
    // are used to determine an approximate homography
    int neighbourhoodCount = 0;

    int index = 0;

    int groupedIndex = -1;

    newCorners.assign(vCorners.begin(), vCorners.end());

    if (DEBUG_MODE > 2)
    {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        printf("%s << Initial corners.\n", __FUNCTION__);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);

    }

    index = 0;

    // just edges
    for (int i = 0; i < 2*Y; i++)
    {
        for (int j = 0; j < 2*X; j++)
        {
            //if (((i != 0) && (i != 2*Y-1) && (j != 0) && (j != 2*X-1)) || (((i == 0) || (i == 2*Y-1)) && ((j == 0) || (j == 2*X-1)))) {

            if (((i == 0) || (i == 2*Y-1) || (j == 0) || (j == 2*X-1)) && !(((i < 2) || (i > 2*Y-3)) && ((j < 2) || (j > 2*X-3))))
            {

                patchNeighbourhood.clear();
                patchArrangement.clear();

                neighbourhoodCount = 0;

                //index = 2*i + (4*(X-1))*(i/2) + 2*(j/2)+j;
                index = 2*X*i + j;
                groupedIndex = 4*(X*(i/2)+(j/2)) + (j%2) + 2*(i%2);

                //printf("%s << (i, j) = (%d, %d); index = %d\n", __FUNCTION__, i, j, index);

                int z[4], c[4];

                if (i == 0)                 // For top edge
                {
                    c[0] = index+2*X-1;
                    c[1] = index+2*X+1;
                    c[2] = index+4*X-1;
                    c[3] = index+4*X+1;
                }
                else if (i == 2*Y-1)        // For bottom edge
                {
                    c[0] = index-4*X-1;
                    c[1] = index-4*X+1;
                    c[2] = index-2*X-1;
                    c[3] = index-2*X+1;
                }
                else if (j == 0)            // For left edge
                {
                    c[0] = index-2*X+1;
                    c[1] = index-2*X+2;
                    c[2] = index+2*X+1;
                    c[3] = index+2*X+2;
                }
                else if (j == 2*X-1)        // For right edge
                {
                    c[0] = index-2*X-2;
                    c[1] = index-2*X-1;
                    c[2] = index+2*X-2;
                    c[3] = index+2*X-1;
                }

                int m, n;

                for (int k = 0; k < 4; k++)
                {
                    // row
                    m = c[k] / (2*X);
                    // col
                    n = c[k] % (2*X);
                    // grouped index
                    //z[k] = 4*((n/2)*X+(m/2))+(m % 2) + 2*(n/2);
                    //z[k] = 2*m + (4*(X-1))*(m/2) + 2*(n/2)+n;
                    z[k] = 4*(X*(m/2)+(n/2)) + (n%2) + 2*(m%2);
                    //z[k] = (X/2)*m + 2*(m%2) + 4*(n/2) ((n+1)%2);

                    //printf("%s << (m, n) = (%d, %d); c[%d] = %d; z[%d] = %d\n", __FUNCTION__, m, n, k, c[k], k, z[k]);
                }

                //cin.get();

                for (int k = 0; k < 4; k++)
                {
                    //printf("%s << z[%d] = %f\n", __FUNCTION__, k, z[k]);
                    patchNeighbourhood.push_back(newCorners.at(z[k]));
                    patchArrangement.push_back(fullCornerGrid.at(z[k]));
                    //printf("%s << pn = (%f, %f)\n", __FUNCTION__, patchNeighbourhood.at(k).x, patchNeighbourhood.at(k).y);
                    //printf("%s << pa = (%f, %f)\n", __FUNCTION__, patchArrangement.at(k).x, patchArrangement.at(k).y);
                    neighbourhoodCount++;
                }


                // find homography
                //printf("%s << Finding homography...\n", __FUNCTION__);
                homography = findHomography(Mat(patchArrangement), Mat(patchNeighbourhood));
                //printf("%s << Homography found!\n", __FUNCTION__);

                cornerArrangement.clear();

                cornerArrangement.push_back(fullCornerGrid.at(groupedIndex));

                // apply homography to these co-ordinates
                Mat tmpMat1 = Mat(cornerArrangement);

                perspectiveTransform(tmpMat1, cornerLocs, homography);

                newCorners.at(groupedIndex) = Point2f(cornerLocs.at<Vec3f>(0,0)[0], cornerLocs.at<Vec3f>(0,0)[1]);

                // Calculate maximum search distance for correcting this local point
                minDimension = findMinimumSeparation(patchNeighbourhood);
                correctionDistance = max(int(double(minDimension)/4.0), MINIMUM_CORRECTION_DISTANCE);

                //printf("%s << correctionDistance = %d\n", __FUNCTION__, correctionDistance);

                // Implement search for just this point
                targetCorner.clear();
                targetCorner.push_back(newCorners.at(groupedIndex));
                cornerSubPix(imGrey, targetCorner, Size(correctionDistance, correctionDistance), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));
                newCorners.at(groupedIndex) = targetCorner.at(0);

            }
        }
    }

    /*
    if (DEBUG_MODE > 2) {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        printf("%s << Fixing the edges...\n", __FUNCTION__);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);
    }

    printf("%s << pre cSP\n", __FUNCTION__);
    cornerSubPix(imGrey, newCorners, Size(correctionDistance, correctionDistance), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));
    */

    if (DEBUG_MODE > 2)
    {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        printf("%s << Refinement of edges.\n", __FUNCTION__);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay, false);

    }

    index = 0;

    // second last ring
    for (int i = 0; i < 2*Y; i++)
    {
        for (int j = 0; j < 2*X; j++)
        {
            //if (((i != 0) && (i != 2*Y-1) && (j != 0) && (j != 2*X-1)) || (((i == 0) || (i == 2*Y-1)) && ((j == 0) || (j == 2*X-1)))) {

            if ((((i == 1) || (i == 2*Y-2)) && ((j == 0) || (j == 2*X-1))) || (((i == 0) || (i == 2*Y-1)) && ((j == 1) || (j == 2*X-2))))
            {

                patchNeighbourhood.clear();
                patchArrangement.clear();

                neighbourhoodCount = 0;

                //index = 2*i + (4*(X-1))*(i/2) + 2*(j/2)+j;
                index = 2*X*i + j;
                groupedIndex = 4*(X*(i/2)+(j/2)) + (j%2) + 2*(i%2);

                //printf("%s << (i, j) = (%d, %d); index = %d\n", __FUNCTION__, i, j, index);

                int z[4], c[4];

                // c = {0, 0, 0, 0};


                if ((i == 0) && (j == 1))                                   // top left (top)
                {
                    c[0] = index+1;
                    c[1] = index+2*X;
                    c[2] = index+2*X+2;
                    c[3] = index+4*X+1;     // c = {index+1, index+2*X, index+2*X+2, index+4*X+2};
                }
                else if ((i == 1) && (j == 0))                              // top left (bot)
                {
                    c[0] = index+1;
                    c[1] = index+2*X;
                    c[2] = index+2*X+2;
                    c[3] = index+4*X+1;
                }
                else if ((i == 0) && (j == 2*X-2))                          // top right (top)
                {
                    c[0] = index-1;
                    c[1] = index+2*X-2;
                    c[2] = index+2*X;
                    c[3] = index+4*X-1;
                }
                else if ((i == 1) && (j == 2*X-1))                          // top right (bot)
                {
                    c[0] = index-1;
                    c[1] = index+2*X-2;
                    c[2] = index+2*X;
                    c[3] = index+4*X-1;     // c = {index-1, index+2*X-2, index+2*X-1, index+4*X-2};
                }
                else if ((i == 2*Y-2) && (j == 0))                          // bottom left (top)
                {
                    c[0] = index-4*X+1;
                    c[1] = index-2*X;
                    c[2] = index-2*X+2;
                    c[3] = index+1;
                }
                else if ((i == 2*Y-1) && (j == 1))                          // bot left (bot)
                {
                    c[0] = index-4*X+1;
                    c[1] = index-2*X;
                    c[2] = index-2*X+2;
                    c[3] = index+1;     // c = {index-4*X+1, index-4*X+2, index-2*X, index-2*X+1};
                }
                else if ((i == 2*Y-2) && (j == 2*X-1))                      // bottom right (top)
                {
                    c[0] = index-4*X-1;
                    c[1] = index-2*X-2;
                    c[2] = index-2*X;
                    c[3] = index-1;
                }
                else if ((i == 2*Y-1) && (j == 2*X-2))                      // bot right (bot)
                {
                    c[0] = index-4*X-1;
                    c[1] = index-2*X-2;
                    c[2] = index-2*X;
                    c[3] = index-1;     // c = {index-4*X-2, index-4*X-1, index-2*X-1, index-2*X};
                }


                int m, n;

                for (int k = 0; k < 4; k++)
                {
                    // row
                    m = c[k] / (2*X);
                    // col
                    n = c[k] % (2*X);
                    // grouped index
                    //z[k] = 4*((n/2)*X+(m/2))+(m % 2) + 2*(n/2);
                    //z[k] = 2*m + (4*(X-1))*(m/2) + 2*(n/2)+n;
                    z[k] = 4*(X*(m/2)+(n/2)) + (n%2) + 2*(m%2);
                    //z[k] = (X/2)*m + 2*(m%2) + 4*(n/2) ((n+1)%2);

                    //printf("%s << (m, n) = (%d, %d); c[%d] = %d; z[%d] = %d\n", __FUNCTION__, m, n, k, c[k], k, z[k]);
                }

                //cin.get();

                for (int k = 0; k < 4; k++)
                {
                    //printf("%s << z[%d] = %f\n", __FUNCTION__, k, z[k]);
                    patchNeighbourhood.push_back(newCorners.at(z[k]));
                    patchArrangement.push_back(fullCornerGrid.at(z[k]));
                    //printf("%s << pn = (%f, %f)\n", __FUNCTION__, patchNeighbourhood.at(k).x, patchNeighbourhood.at(k).y);
                    //printf("%s << pa = (%f, %f)\n", __FUNCTION__, patchArrangement.at(k).x, patchArrangement.at(k).y);
                    neighbourhoodCount++;
                }


                // find homography
                //printf("%s << Finding homography...\n", __FUNCTION__);
                homography = findHomography(Mat(patchArrangement), Mat(patchNeighbourhood));
                //printf("%s << Homography found!\n", __FUNCTION__);

                cornerArrangement.clear();

                cornerArrangement.push_back(fullCornerGrid.at(groupedIndex));

                // apply homography to these co-ordinates
                Mat tmpMat1 = Mat(cornerArrangement);

                perspectiveTransform(tmpMat1, cornerLocs, homography);

                newCorners.at(groupedIndex) = Point2f(cornerLocs.at<Vec3f>(0,0)[0], cornerLocs.at<Vec3f>(0,0)[1]);


                // Calculate maximum search distance for correcting this local point
                minDimension = findMinimumSeparation(patchNeighbourhood);
                correctionDistance = max(int(double(minDimension)/4.0), MINIMUM_CORRECTION_DISTANCE);

                //printf("%s << correctionDistance = %d\n", __FUNCTION__, correctionDistance);

                // Implement search for just this point
                targetCorner.clear();
                targetCorner.push_back(newCorners.at(groupedIndex));
                cornerSubPix(imGrey, targetCorner, Size(correctionDistance, correctionDistance), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));
                newCorners.at(groupedIndex) = targetCorner.at(0);

            }
        }
    }

    /*

    if (DEBUG_MODE > 2) {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        printf("%s << Fixing the pseudocorners...\n", __FUNCTION__);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);
    }

    printf("%s << pre cSP\n", __FUNCTION__);
    cornerSubPix(imGrey, newCorners, Size(correctionDistance, correctionDistance), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));

    */

    if (DEBUG_MODE > 2)
    {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        printf("%s << Refinement of psuedocorners.\n", __FUNCTION__);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay, false);
    }

    index = 0;

    // final corners
    for (int i = 0; i < 2*Y; i++)
    {
        for (int j = 0; j < 2*X; j++)
        {
            //if (((i != 0) && (i != 2*Y-1) && (j != 0) && (j != 2*X-1)) || (((i == 0) || (i == 2*Y-1)) && ((j == 0) || (j == 2*X-1)))) {

            if (((i == 0) || (i == 2*Y-1)) && ((j == 0) || (j == 2*X-1)))
            {

                patchNeighbourhood.clear();
                patchArrangement.clear();

                neighbourhoodCount = 0;

                //index = 2*i + (4*(X-1))*(i/2) + 2*(j/2)+j;
                index = 2*X*i + j;
                groupedIndex = 4*(X*(i/2)+(j/2)) + (j%2) + 2*(i%2);

                //printf("%s << (i, j) = (%d, %d); index = %d\n", __FUNCTION__, i, j, index);

                int z[4], c[4];

                // c = {0, 0, 0, 0};


                if ((i == 0) && (j == 0))                                   // top left
                {
                    c[0] = index+1;
                    c[1] = index+2*X;
                    c[2] = index+2*X+2;
                    c[3] = index+4*X+1;
                }
                else if ((i == 0) && (j == 2*X-1))                          // top right
                {
                    c[0] = index-1;
                    c[1] =index+2*X-2;
                    c[2] = index+2*X;
                    c[3] = index+4*X-1;
                }
                else if ((i == 2*Y-1) && (j == 0))                          // bot left
                {
                    c[0] = index-4*X+1;
                    c[1] = index-2*X;
                    c[2] = index-2*X+2;
                    c[3] = index+1;
                }
                else if ((i == 2*Y-1) && (j == 2*X-1))                      // bot right
                {
                    c[0] = index-4*X-1;
                    c[1] = index-2*X-2;
                    c[2] = index-2*X;
                    c[3] = index-1;
                }


                int m, n;

                for (int k = 0; k < 4; k++)
                {
                    // row
                    m = c[k] / (2*X);
                    // col
                    n = c[k] % (2*X);
                    // grouped index
                    //z[k] = 4*((n/2)*X+(m/2))+(m % 2) + 2*(n/2);
                    //z[k] = 2*m + (4*(X-1))*(m/2) + 2*(n/2)+n;
                    z[k] = 4*(X*(m/2)+(n/2)) + (n%2) + 2*(m%2);
                    //z[k] = (X/2)*m + 2*(m%2) + 4*(n/2) ((n+1)%2);

                    //printf("%s << (m, n) = (%d, %d); c[%d] = %d; z[%d] = %d\n", __FUNCTION__, m, n, k, c[k], k, z[k]);
                }

                //cin.get();

                for (int k = 0; k < 4; k++)
                {
                    //printf("%s << z[%d] = %f\n", __FUNCTION__, k, z[k]);
                    patchNeighbourhood.push_back(newCorners.at(z[k]));
                    patchArrangement.push_back(fullCornerGrid.at(z[k]));
                    //printf("%s << pn = (%f, %f)\n", __FUNCTION__, patchNeighbourhood.at(k).x, patchNeighbourhood.at(k).y);
                    //printf("%s << pa = (%f, %f)\n", __FUNCTION__, patchArrangement.at(k).x, patchArrangement.at(k).y);
                    neighbourhoodCount++;
                }


                // find homography
                //printf("%s << Finding homography...\n", __FUNCTION__);
                homography = findHomography(Mat(patchArrangement), Mat(patchNeighbourhood));
                //printf("%s << Homography found!\n", __FUNCTION__);

                cornerArrangement.clear();

                cornerArrangement.push_back(fullCornerGrid.at(groupedIndex));

                // apply homography to these co-ordinates
                Mat tmpMat1 = Mat(cornerArrangement);

                perspectiveTransform(tmpMat1, cornerLocs, homography);

                newCorners.at(groupedIndex) = Point2f(cornerLocs.at<Vec3f>(0,0)[0], cornerLocs.at<Vec3f>(0,0)[1]);

                // Calculate maximum search distance for correcting this local point
                minDimension = findMinimumSeparation(patchNeighbourhood);
                correctionDistance = max(int(double(minDimension)/4.0), 5);

                //printf("%s << correctionDistance = %d\n", __FUNCTION__, correctionDistance);

                // Implement search for just this point
                targetCorner.clear();
                targetCorner.push_back(newCorners.at(groupedIndex));
                cornerSubPix(imGrey, targetCorner, Size(correctionDistance, correctionDistance), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));
                newCorners.at(groupedIndex) = targetCorner.at(0);

            }
        }
    }

    /*
    if (DEBUG_MODE > 2) {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        printf("%s << Fixing the true corners...\n", __FUNCTION__);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay);
    }

    printf("%s << pre cSP\n", __FUNCTION__);
    cornerSubPix(imGrey, newCorners, Size(correctionDistance/2, correctionDistance/2), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));
    */

    if (DEBUG_MODE > 2)
    {
        vector<Point2f> redistorted;
        Mat cornersForDisplay(newCorners);
        printf("%s << Refinement of true corners.\n", __FUNCTION__);
        debugDisplayPattern(image, cvSize(patternSize.width, patternSize.height), cornersForDisplay, false);

    }

    index = 0;

    vCorners.clear();
    vCorners.assign(newCorners.begin(), newCorners.end());

}

void groupPointsInQuads(Size patternSize, vector<Point2f>& corners)
{
    vector<Point2f> newCorners;

    int id1, id2, id3, id4;

    for (int i = 0; i < patternSize.height/2; i++)
    {
        for (int j = 0; j < patternSize.width/2; j++)
        {

            id1 = 2*(i*patternSize.width+j);
            id2 = 2*(i*patternSize.width+j)+1;
            id3 = 2*(i*patternSize.width+j)+patternSize.width;
            id4 = 2*(i*patternSize.width+j)+patternSize.width+1;

            //printf("%s << ids (%d, %d) = [%d, %d, %d, %d]\n", __FUNCTION__, i, j, id1, id2, id3, id4);
            newCorners.push_back(corners.at(id1));
            newCorners.push_back(corners.at(id2));
            newCorners.push_back(corners.at(id3));
            newCorners.push_back(corners.at(id4));
        }
    }

    corners.assign(newCorners.begin(), newCorners.end());
}

int findBestCorners(const Mat& image, vector<Point2f>& src, vector<Point2f>& dst, Size patternSize, int detector, int searchDist)
{
    //goodFeaturesToTrack(image, retCorner, 1, 0.05, 1.0, mask, 3, true, 0.04);
    int successfulMappings = 0;

    double maxDist = 0.0;

    vector<Point2f> foundCorners;
    int maxCorners = 4*int(src.size());

    unsigned int *srcBestMatches;
    unsigned int foundBestMatch;
    double bestDist, currDist;

    /*
    Mat inputDisp;
    image.copyTo(inputDisp);
    drawChessboardCorners(inputDisp, cvSize(12, 8), Mat(src), true);
    imshow("debugWin", inputDisp);
    waitKey(500);
    */

    vector<KeyPoint> keypoints;

    //printf("%s << About to grayify.\n", __FUNCTION__);


    Mat imGrey, tmpMat;
    if (image.channels() > 1)
    {
        cvtColor(image, imGrey, CV_RGB2GRAY);
    }
    else
    {
        image.copyTo(imGrey);
    }


    Mat imageCopy(480, 640, CV_8UC1);
    Mat inputDisp;

#ifdef _WIN32
    /*
    qacdtl::group_array<double> corners;
    vil_image_view<vxl_byte> vByteImage(imGrey.cols, imGrey.rows);
    vil_image_view<double> vDblImage(imGrey.cols, imGrey.rows);
    */
#endif

    //printf("%s << Entered function.\n", __FUNCTION__);

    switch (detector)
    {
        // ==================================================
    case 0:     //        NO ACTUAL CORNER DETECTOR USED
        // ==================================================

        dst.assign(src.begin(), src.end());
        return 0;

        // ==================================================
    case 1:     //        BASIC HARRIS DETECTOR
        // ==================================================

        // Blur
        //GaussianBlur(imGrey, tmpMat, Size(21,21), 5.0);
        //tmpMat.copyTo(imGrey);

        // Sets maximum number of corners to 4 times the actual number
        goodFeaturesToTrack(imGrey, foundCorners, maxCorners, 0.001, 5.0, Mat(), 11, true, 0.04);		// 3rd last should be small to limit scale...

        //printf("%s << foundCorners.size() = %d\n", __FUNCTION__, foundCorners.size());

        break;
        // ==================================================
    case 2:     //        BIPOLAR HESSIAN DETECTOR
        // ==================================================
#ifdef _WIN32
        /*
        // Copy from cv::Mat to vil_image view
        imageCopy.data = vByteImage.top_left_ptr();		// Point imageCopy data to vByteImage


        imGrey.copyTo(imageCopy);						// Copy image data to this location
        //vByteImage.top_left_ptr() = image.data;

        imshow("copiedImage", imageCopy);
        waitKey(50);
        //printf("%s << imageCopy.rows = %d; imageCopy.cols = %d\n", __FUNCTION__, imageCopy.rows, imageCopy.cols);

        // Convert to double
        vil_convert_cast(vByteImage, vDblImage);

        // Extract corners.
        qaclfl::extractor_hessian_graph::extract_corners(corners, vDblImage, 5.0);	//

        // Convert from VXL corners back to "foundCorners" vector
        for (unsigned int i = 0; i < corners.size(); i++) {
        	// x, y, dx, dy, scale, strength ?
        	foundCorners.push_back(Point2d(double(corners(i)[0]), double(corners(i)[1])));		// 2nd argument: double(corners(i)[0])
        	//printf("%s << pt(%d) = (%f; %f)\n", __FUNCTION__, i, corners(i)[1], corners(i)[0]);
        }

        //printf("%s << foundCorners.size() = %d\n", __FUNCTION__, foundCorners.size());
        */
        // That's it!
        break;

#endif

#ifndef _WIN32
        // Same as case 0
        dst.assign(src.begin(), src.end());
        return 0;
#endif
        // ==================================================
    case 3:     //        BASIC FAST DETECTOR
        // ==================================================

        //printf("%s << Using FAST detector..\n", __FUNCTION__);
        // Sets maximum number of corners to 4 times the actual number
        FAST(imGrey, keypoints, 16);

        for (unsigned int i = 0; i < keypoints.size(); i++)
        {
            foundCorners.push_back(keypoints.at(i).pt);
        }

        //printf("%s << foundCorners.size() = %d\n", __FUNCTION__, foundCorners.size());

        break;
        // ==================================================
    case 4:     //        cornerSubPix()
        // ==================================================
        //printf("%s << image.size() = (%d, %d)\n", __FUNCTION__, image.cols, image.rows);
        //printf("%s << src.size() = %d \n", __FUNCTION__, src.size());
        //printf("%s << searchDist = %f \n", __FUNCTION__, searchDist);
        //printf("%s << pre cSP\n", __FUNCTION__);

        // Rather than trying to correct all at one time, modify the distance

        initialRefinementOfCorners(imGrey, src, patternSize);

        dst.assign(src.begin(), src.end());

        if (DEBUG_MODE > 3)
        {
            image.copyTo(inputDisp);
            drawChessboardCorners(inputDisp, cvSize(12, 8), Mat(dst), true);
            imshow("mainWin", inputDisp);
            waitKey(0);
        }

        return 0;
        //break;
        // ==================================================
    case 5:     //        cornerSubPix() + BIPOLAR HESSIAN DETECTOR
        // ==================================================
        cornerSubPix(image, src, Size(searchDist*2, searchDist*2), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));

        maxDist = 0.5;	// reduce searchDist for using bipolar hessian to refine the cornerSubPix() values slightly, but not radically..

#ifdef _WIN32
        /*
        // Copy from cv::Mat to vil_image view
        imageCopy.data = vByteImage.top_left_ptr();		// Point imageCopy data to vByteImage


        imGrey.copyTo(imageCopy);						// Copy image data to this location
        //vByteImage.top_left_ptr() = image.data;

        imshow("copiedImage", imageCopy);
        waitKey(50);
        //printf("%s << imageCopy.rows = %d; imageCopy.cols = %d\n", __FUNCTION__, imageCopy.rows, imageCopy.cols);

        // Convert to double
        vil_convert_cast(vByteImage, vDblImage);

        // Extract corners.
        qaclfl::extractor_hessian_graph::extract_corners(corners, vDblImage, 3.0);	//

        // Convert from VXL corners back to "foundCorners" vector
        for (unsigned int i = 0; i < corners.size(); i++) {
        	// x, y, dx, dy, scale, strength ?
        	foundCorners.push_back(Point2d(double(corners(i)[0]), double(corners(i)[1])));		// 2nd argument: double(corners(i)[0])
        	//printf("%s << pt(%d) = (%f; %f)\n", __FUNCTION__, i, corners(i)[1], corners(i)[0]);
        }

        //printf("%s << foundCorners.size() = %d\n", __FUNCTION__, foundCorners.size());
        */
        // That's it!
        break;

#endif

#ifndef _WIN32
        // Same as case 0
        dst.assign(src.begin(), src.end());
        return 0;
#endif
    default:
        return -1;
    }

    //printf("%s << Corners Detected.\n", __FUNCTION__);

    // Display raw corners detected:
    //printf("%s << A...\n", __FUNCTION__);
    Mat imCpy(imGrey.rows, imGrey.cols, CV_8UC3);
    Mat imCpy2;
    //printf("%s << Trying to convert...\n", __FUNCTION__);
    cvtColor(imGrey, imCpy, CV_GRAY2RGB);
    //printf("%s << Converted.\n", __FUNCTION__);
    resize(imCpy, imCpy2, Size(), 2.0, 2.0);

    Scalar color;

    for (unsigned int i = 0; i < foundCorners.size(); i++)
    {
        color = Scalar( rand()&255, rand()&255, rand()&255 );
        circle(imCpy2, Point2f(foundCorners.at(i).x * float(2.0), foundCorners.at(i).y * float(2.0)), 4, color, 2);
    }

    imshow("rawCorners", imCpy2);
    waitKey(20);

    srcBestMatches = new unsigned int[src.size()];

    // Search for best matches between src and found
    for (unsigned int i = 0; i < src.size(); i++)
    {
        bestDist = 9e99;
        for (unsigned int j = 0; j < foundCorners.size(); j++)
        {
            currDist = pow(pow(double(src.at(i).x) - double(foundCorners.at(j).x), 2.0) + pow(double(src.at(i).y) - double(foundCorners.at(j).y), 2.0), 0.5);

            if (currDist < bestDist)
            {
                bestDist = currDist;
                srcBestMatches[i] = j;
            }
        }
    }

    // Check if these best matches are reversed
    for (unsigned int i = 0; i < src.size(); i++)
    {
        bestDist = 9e99;
        for (unsigned int k = 0; k < src.size(); k++)
        {
            // Distance between src.at(i)'s best match, and all other src elements
            currDist = pow(pow(double(src.at(k).x) - double(foundCorners.at(srcBestMatches[i]).x), 2.0) + pow(double(src.at(k).y) - double(foundCorners.at(srcBestMatches[i]).y), 2.0), 0.5);

            if (currDist < bestDist)
            {
                bestDist = currDist;
                foundBestMatch = k;
            }
        }

        // if src.at(i)'s best match is src.at(i), it's a besty!
        if ((foundBestMatch == i) && (bestDist < searchDist))
        {
            successfulMappings++;
            dst.push_back(foundCorners.at(srcBestMatches[i]));
        }
        else
        {
            dst.push_back(src.at(i));
        }
    }

    //printf("%s << src.size() = %d\n", __FUNCTION__, src.size());
    //printf("%s << dst.size() = %d\n", __FUNCTION__, dst.size());

    /*
    // Or, if you just want 1-way mappings:
    for (unsigned int i = 0; i < src.size(); i++) {
    	dst.push_back(foundCorners.at(srcBestMatches[i]));
    }
    */

    delete[] srcBestMatches;

    //printf("%s << successfulMappings = %d\n", __FUNCTION__, successfulMappings);

    return successfulMappings;
}

void initialRefinementOfCorners(const Mat& imGrey, vector<Point2f>& src, Size patternSize)
{
    // ...

    int correctionDistance;
    double minDimension;

    Mat imDisplay;
    imGrey.copyTo(imDisplay);

    Mat forDisplay(src);

    Mat forDisp2;

    if (DEBUG_MODE > 2)
    {
        debugDisplayPattern(imDisplay, patternSize, forDisplay, false);
    }


    vector<Point2f> ptNeighbourhood, targetCorner, dst;

    dst.assign(src.begin(), src.end());

    // Go through all points and establish neighbourhood

    for (int j = 1; j < patternSize.height-1; j++)
    {
        for (int i = 1; i < patternSize.width-1; i++)
        {

            ptNeighbourhood.clear();

            //printf("%s << index = (%d, %d); vals = (", __FUNCTION__, i, j);

            // add 4 surrounding points
            int val;

            val = (j-1)*patternSize.width+(i-1);
            ptNeighbourhood.push_back(src.at(val));
            //printf("%d, ", val);

            val = (j-1)*patternSize.width+(i+1);
            ptNeighbourhood.push_back(src.at(val));
            //printf("%d, ", val);

            val = (j+1)*patternSize.width+(i-1);
            ptNeighbourhood.push_back(src.at(val));
            //printf("%d, ", val);

            val = (j+1)*patternSize.width+(i+1);
            ptNeighbourhood.push_back(src.at(val));
            //printf("%d)\n", val);

            imGrey.copyTo(imDisplay);

            forDisp2 = Mat(ptNeighbourhood);

            if (DEBUG_MODE > 2)
            {
                //debugDisplayPattern(imDisplay, Size(2, 2), forDisp2, true);
            }


            minDimension = findMinimumSeparation(ptNeighbourhood);

            //printf("%s << minDimension = %f\n", __FUNCTION__, minDimension);

            correctionDistance = max(int(double(minDimension)/4), 1);

            //printf("%s << correctionDistance = %d\n", __FUNCTION__, correctionDistance);

            targetCorner.clear();
            targetCorner.push_back(src.at(j*patternSize.width+i));

            cornerSubPix(imGrey, targetCorner, Size(correctionDistance, correctionDistance), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));

            dst.at(j*patternSize.width+i) = targetCorner.at(0);

        }
    }

    Mat imCol;

    cvtColor(imDisplay, imCol, CV_GRAY2RGB);


	/*
    if (DEBUG_MODE > 2)
    {
        debugDisplayPattern(imDisplay, patternSize, forDisplay, false);

        drawLinesBetweenPoints(imCol, src, dst);

        imshow("lineDrawing", imCol);
        waitKey(0);
    }
	*/


    src.assign(dst.begin(), dst.end());

    //imGrey.copyTo(imDisplay);
    //debugDisplayPattern(imGrey, patternSize, forDisplay, true);



}

void sortCorners(Size imageSize, Size patternSize, vector<Point2f>& corners)
{
    vector<Point2f> newCorners;

    int X = patternSize.width/2;
    int Y = patternSize.height/2;

    // 0,1,4,5,8,9,12,13,16,17,20,21,2,3,6,7,10,11,14,15,18,19,22,23
    // 0,4,8,12,16,20

    for (int i = 0; i < Y; i++)         // for each row (square)
    {
        for (int j = 0; j < X; j++)        // for each column (square)
        {
            if (DEBUG_MODE > 3)
            {
                // printf("%d, %d, ", 4*i*X+j*4, 4*i*X+j*4+1);
            }
            newCorners.push_back(corners.at(4*i*X+j*4));        // push back first element
            newCorners.push_back(corners.at(4*i*X+j*4+1));      // push back second element
        }

        for (int j = 0; j < X; j++)
        {
            if (DEBUG_MODE > 3)
            {
                printf("%d, %d, ", 4*i*X+j*4+2, 4*i*X+j*4+3);
            }
            newCorners.push_back(corners.at(4*i*X+j*4+2));        // push back third element
            newCorners.push_back(corners.at(4*i*X+j*4+3));      // push back fourth element                                        // push back third element
        }
        //printf("\n");
    }

    corners.clear();

    for (unsigned int i = 0; i < newCorners.size(); i++)
    {
        corners.push_back(newCorners.at(i));
    }
}

bool correctPatchCentres(const Mat& image, Size patternSize, vector<Point2f>& patchCentres, int mode)
{

    bool found = true;

    return found;

    printf("%s << Entering...\n", __FUNCTION__);

    Mat imGrey;
    if (image.channels() > 1)
    {
        cvtColor(image, imGrey, CV_RGB2GRAY);
    }
    else
    {
        image.copyTo(imGrey);
    }

    Point3f newPoint;
    cv::vector<Point3f> row;

    //printf("%s << patternSize = (%d, %d)\n", __FUNCTION__, patternSize.width, patternSize.height);

    if (mode == 1)
    {
        for (int i = 0; i < patternSize.height/2; i++)
        {
            for (int j = 0; j < patternSize.width/2; j++)
            {
                newPoint = Point3f(float(i), float(j), 0.0);
                row.push_back(newPoint);
            }

        }
    }
    else
    {
        for (int i = 0; i < patternSize.height+1; i++)
        {
            for (int j = 0; j < patternSize.width+1; j++)
            {
                newPoint = Point3f(float(i), float(j), 0.0);
                row.push_back(newPoint);
            }

        }
    }

    if (DEBUG_MODE > 2)
    {
        Mat cornersMat(patchCentres);
        debugDisplayPattern(image, cvSize(patternSize.width/2, patternSize.height/2), cornersMat);
    }

    //printf("%s << row.size() = %d\n", __FUNCTION__, row.size());

    //printf("%s << STEP 1a...\n", __FUNCTION__);

    cv::vector< cv::vector<Point3f> > objectPoints;
    objectPoints.push_back(row);

    vector<vector<Point2f> > vvOriginalCentres;
    vvOriginalCentres.push_back(patchCentres);

    //printf("%s << STEP 1b...\n", __FUNCTION__);

    Mat cameraMatrix, distCoeffs, newCamMat;

    cv::vector<Mat> rvecs, tvecs;

    calibrateCamera(objectPoints, vvOriginalCentres, image.size(), cameraMatrix, distCoeffs, rvecs, tvecs, PATCH_CORRECTION_INTRINSICS_FLAGS);

    double alpha = 0.5;

    Rect validROI;
    newCamMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image.size(), alpha, image.size(), &validROI);

    Mat undistortedImage;
    undistort(image, undistortedImage, cameraMatrix, distCoeffs, newCamMat);

    vector<Point2f> newCentres;
    found = findPatternCentres(undistortedImage, patternSize, newCentres, mode);

    if (!found)
    {
        printf("%s << Pattern could not be found once image was undistorted..\n", __FUNCTION__);
        //cin.get();
        return found;
    }

    printf("%s << Redistorting MSER centroids...\n", __FUNCTION__);
    //patchCentres.clear();
    redistortPoints(newCentres, patchCentres, cameraMatrix, distCoeffs, newCamMat);
    printf("%s << newCentres.size() = %d\n", __FUNCTION__, newCentres.size());
    printf("%s << patchCentres.size() = %d\n", __FUNCTION__, patchCentres.size());
    printf("%s << DONE.\n", __FUNCTION__);

    if (DEBUG_MODE > 2)
    {
        Mat cornersMat(patchCentres);
        debugDisplayPattern(image, cvSize(patternSize.width/2, patternSize.height/2), cornersMat);
    }

    /*
    double meanVal = 0.0, maxVal = 0.0;

    for (int i = 0; i < patchCentres.size(); i++) {
        for (int j = 0; j < 1; j++) {
            for (int k = 0; k < 1; k++) {
                if (imGrey.at<unsigned char>(patchCentres.at(i).y+j, patchCentres.at(i).x+k) > maxVal) {
                    maxVal = imGrey.at<unsigned char>(patchCentres.at(i).y+j, patchCentres.at(i).x+k);
                }
                meanVal += imGrey.at<unsigned char>(patchCentres.at(i).y+j, patchCentres.at(i).x+k);
            }
        }
    }



    meanVal /= (1*patchCentres.size());

    printf("%s << meanVal = %f\n", __FUNCTION__, meanVal);

    Mat enhancedImage(imGrey.size(), imGrey.type());

    //imshow("correcting", imGrey);
    //waitKey(0);

    //imGrey.copyTo(enhancedImage);
    contrastEnhance(imGrey, enhancedImage, int(0.5*meanVal), int(meanVal + 0.5*(255 - meanVal)));    // int(meanVal*6)

    imshow("correcting", enhancedImage);
    waitKey(40);

    vector<Point2f> newCentres;

    Mat enhancedCol;
    cvtColor(enhancedImage, enhancedCol, CV_GRAY2RGB);


    bool found = false;

    printf("%s << Searching for pattern centres...\n", __FUNCTION__);
    found = findPatternCentres(enhancedCol, patternSize, newCentres, 0);
    printf("%s << Search complete.\n", __FUNCTION__);

    if (found) {
        printf("%s << found 2nd time.\n", __FUNCTION__);
        patchCentres.assign(newCentres.begin(), newCentres.end());
    }

    printf("%s << Exiting...\n", __FUNCTION__);
    */

    return found;
}

bool findPatternCentres(const Mat& image, Size patternSize, vector<Point2f>& centres, int mode)
{
    // mode 0: MSER chessboard finder
    // mode 1: MSER mask finder

    if (!checkAcutance())
    {
        return false;
    }

    int patchCols, patchRows, desiredPatchQuantity;
    determinePatchDistribution(patternSize, mode, patchRows, patchCols, desiredPatchQuantity);

    vector<vector<Point> > msers;
    findAllPatches(image, patternSize, msers);

    if (DEBUG_MODE > 3)
    {
        debugDisplayPatches(image, msers);
    }

    if (int(msers.size()) < desiredPatchQuantity)
    {
        centres.clear();
        if (DEBUG_MODE > 1)
        {
            printf("%s << Insufficient patches found. Returning.\n", __FUNCTION__);
        }

        return false;
    }

    bool found = refinePatches(image, patternSize, msers, centres, mode);

    if (DEBUG_MODE > 1)
    {
        printf("%s << Patches found after refinement = %d\n", __FUNCTION__, msers.size());
    }

    if (DEBUG_MODE > 3)
    {
        debugDisplayPatches(image, msers);
    }

    // If patches still not found...
    if (!found)
    {
        centres.clear();

        if (DEBUG_MODE > 1)
        {
            printf("%s << Correct number of patches not found. Returning.\n", __FUNCTION__);
        }

        return false;
    }

    sortPatches(image.size(), patternSize, centres, mode);

    if (DEBUG_MODE > 3)
    {
        Mat patchCentres_(centres);
        debugDisplayPattern(image, cvSize(patchCols, patchRows), patchCentres_);
    }

    found = verifyPatches(image.size(), patternSize, centres, mode, MIN_CORNER_SEPARATION, MAX_CORNER_SEPARATION);

    if (!found)
    {
        centres.clear();

        if (DEBUG_MODE > 1)
        {
            printf("%s << Pattern verification failed. Returning.\n", __FUNCTION__);
        }

        return false;
    }

    return found;
}

bool verifyPattern(Size imSize, Size patternSize, vector<Point2f>& patternPoints, double minDist, double maxDist)
{

    // Check that all points are within the image view:
    bool retVal = true;

    retVal = patternInFrame(imSize, patternPoints);

    if (retVal == false)
    {

        if (DEBUG_MODE > 1)
        {
            printf("%s << Some pattern points are outside valid area.\n", __FUNCTION__);
        }

        return retVal;
    }

    double dist, dist1, dist2, factor;

    double maxStraightnessDist = 30.0;

    int index;

    Mat blackImage = Mat::zeros(imSize, CV_8UC3);
    Scalar blue(255, 0, 0), green(0, 255, 0), red(0, 0, 255);


    // STRAIGHTNESS TEST
    // For each row
    for (int i = 0; i < patternSize.height; i++)
    {
        for (int j = 0; j < patternSize.width-2; j++)
        {
            // check distance of point with one to its right

            index = i*patternSize.width + j;

            blackImage.setTo(0);

            /*
            if (DEBUG_MODE > 3) {
                circle(blackImage, patternPoints.at(index), 5, red, 3);
                circle(blackImage, patternPoints.at(index + 1), 5, green, 3);
                circle(blackImage, patternPoints.at(index + 2), 5, blue, 3);
                imshow("testImage", blackImage);
                waitKey(0);
            }
            */


            dist = perpDist(patternPoints.at(index), patternPoints.at(index + 1), patternPoints.at(index + 2));
            dist1 = distBetweenPts2f(patternPoints.at(index),patternPoints.at(index + 1));
            dist2 = distBetweenPts2f(patternPoints.at(index + 1),patternPoints.at(index + 2));

            factor = max(dist1, dist2) / min(dist1, dist2);

            //printf("%s << dist = %f\n", __FUNCTION__, dist);
            if (dist > maxStraightnessDist)
            {
                if (DEBUG_MODE > 0)
                {
                    printf("%s << Row out of alignment.\n", __FUNCTION__);
                }

                //waitKey(0);
                return false;
            }

            if (factor > 2.0)
            {
                if (DEBUG_MODE > 0)
                {
                    printf("%s << Row factor is out.\n", __FUNCTION__);
                    printf("%s << factor / [dist1, dist2] = %f / [%f, %f].\n", __FUNCTION__, factor, dist1, dist2);
                }
                //waitKey(0);
                return false;
            }
        }
    }

    //waitKey(0);

    // For each column
    for (int i = 0; i < patternSize.height-2; i++)
    {
        for (int j = 0; j < patternSize.width; j++)
        {
            // check distance of point with one below it
            dist = perpDist(patternPoints.at(i*patternSize.width + j), patternPoints.at((i+1)*patternSize.width + j), patternPoints.at((i+2)*patternSize.width + j));
            dist1 = distBetweenPts2f(patternPoints.at(i*patternSize.width + j), patternPoints.at((i+1)*patternSize.width + j));
            dist2 = distBetweenPts2f(patternPoints.at((i+1)*patternSize.width + j), patternPoints.at((i+2)*patternSize.width + j));

            factor = max(dist1, dist2) / min(dist1, dist2);

            if (dist > maxStraightnessDist)
            {
                if (DEBUG_MODE > 0)
                {
                    printf("%s << Column out of alignment.\n", __FUNCTION__);
                }
                //waitKey(0);
                return false;
            }

            if (factor > 2.0)
            {
                if (DEBUG_MODE > 0)
                {
                    printf("%s << Column out of scale. 1\n", __FUNCTION__);
                }
                //waitKey(0);
                return false;
            }
        }
    }

    // CLOSENESS TEST
    // For point
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            
       
            // For each of its neighbours
            
            for (int a = 0; a < patternSize.height; a++) {
				for (int b = 0; b < patternSize.width; b++) {
					
            // for (int a = max(0,i-1); a < min(patternSize.height,i+1+1); a++) {
				// for (int b = max(0,j-1); b < min(patternSize.width,j+1+1); b++) {
					
					if ((a == i) && (b == j)) { continue; }
				
					dist = distBetweenPts2f(patternPoints.at(i*patternSize.width + j), patternPoints.at(a*patternSize.width + b));
            
					if ((dist > maxDist) && (a >= i-1) && (a <= i+1) && (b >= j-1) && (b <= j+1)) {
						if (DEBUG_MODE > 0)	{
							printf("%s << Point (%d, %d) failed adjacent test (%f) [%d, %d] vs [%f, %f].\n", __FUNCTION__, i, j, dist, a, b, minDist, maxDist);
						}
						//waitKey(0);
						return false;
					}
            
					if (dist <= minDist) {
						
						if (DEBUG_MODE > 0)	{
							printf("%s << Point (%d, %d) failed closeness test (%f) [%d, %d] vs [%f, %f].\n", __FUNCTION__, i, j, dist, a, b, minDist, maxDist);
						}
						//waitKey(0);
						return false;
						
					}
					
					if ((DEBUG_MODE > 0) && (dist < 1.0))	{
						printf("%s << Point (%d, %d) has distance of (%f) to [%d, %d] vs [%f, %f].\n", __FUNCTION__, i, j, dist, a, b, minDist, maxDist);
					}
				}
					
					
            
            // check distance of point with one to its right
            
            }
        }
    }

    // If it has still survived:
    return true;
}

bool verifyPatches(Size imSize, Size patternSize, vector<Point2f>& patchCentres, int mode, double minDist, double maxDist)
{

    Size newPatternSize;

    if (mode == 0)
    {
        newPatternSize = Size(patternSize.width+1, patternSize.height+1);
    }
    else
    {
        newPatternSize = Size(patternSize.width/2, patternSize.height/2);
    }

    return verifyPattern(imSize, newPatternSize, patchCentres, minDist, maxDist);

    // old stuff..
    int64 t = getTickCount();

    // Some kind of big loop test, and if a failure is found return false immediately
    for (unsigned int i = 0; i < 10; i++)
    {
        if (0)
        {
            if (DEBUG_MODE > 0)
            {
                t = getTickCount() - t;
                printf("%s << Algorithm duration: %fms\n", __FUNCTION__, t*1000/getTickFrequency());
            }
            return false;
        }
    }

    if (DEBUG_MODE > 0)
    {
        t = getTickCount() - t;
        printf("%s << Algorithm duration: %fms\n", __FUNCTION__, t*1000/getTickFrequency());
    }

    return true;
}

bool patternInFrame(Size imSize, vector<Point2f>& patternPoints, int minBorder)
{

    // Check that all points are within the image view:
    for (unsigned int i = 0; i < patternPoints.size(); i++)
    {
        if ((patternPoints.at(i).x >= imSize.width-minBorder) || (patternPoints.at(i).x < minBorder) || (patternPoints.at(i).y >= imSize.height-minBorder) || (patternPoints.at(i).y < minBorder))
        {

            patternPoints.clear();

            return false;
        }
    }

    // None of the points are within minBorder pixels of the border
    return true;

}

void shapeFilter(vector<mserPatch>& patches, vector<vector<Point> >& msers)
{
    // TODO:
    // Improve height/width measurement so that it more accurately detects skinniness, since
    // at the moment thin msers that are at angles are still accepted.
    // Once this has been implemented, you can relax the requirements so that it doesn't reject
    // the pattern when it is at a significant angle to the camera.

    vector<mserPatch> newPatches;
    vector<vector<Point> > newMsers;

    // Any MSERs that have a height/width ratio within the acceptableFactor are accepted
    double cWidth, cHeight;
    double acceptableFactor = 2.0;

    for (unsigned int i = 0; i < patches.size(); i++)
    {
        cWidth = 0;
        cHeight = 0;
        //contourDimensions(patches.at(i).hull, cWidth, cHeight);
        contourDimensions(msers.at(i), cWidth, cHeight);
        //printf("contour [%d] dimensions: %f x %f\n", i, cWidth, cHeight);

        if (((cHeight/cWidth) < acceptableFactor) && ((cWidth/cHeight) < acceptableFactor))
        {
            newMsers.push_back(msers.at(i));
            newPatches.push_back(patches.at(i));
        }
    }

    patches.clear();
    msers.clear();

    for (unsigned int i = 0; i < newPatches.size(); i++)
    {
        patches.push_back(newPatches.at(i));
        msers.push_back(newMsers.at(i));
    }

}

void varianceFilter(vector<mserPatch>& patches, vector<vector<Point> >& msers)
{
    vector<mserPatch> newPatches;
    vector<vector<Point> > newMsers;

    //double maxAcceptableVariance = 256.0;   // 16 squared...
    double maxAcceptableVariance = 1024.0;   // 32 squared...

    for (unsigned int i = 0; i < patches.size(); i++)
    {
        if (patches.at(i).varIntensity < maxAcceptableVariance)
        {
            newMsers.push_back(msers.at(i));
            newPatches.push_back(patches.at(i));
        }
    }

    patches.clear();
    msers.clear();

    for (unsigned int i = 0; i < newPatches.size(); i++)
    {
        patches.push_back(newPatches.at(i));
        msers.push_back(newMsers.at(i));
    }

}

void enclosureFilter(vector<mserPatch>& patches, vector<vector<Point> >& msers)
{

    unsigned int i = 0, j = 0;

    bool encloses = false;

    double enclosureScore = 0.0;

    while (i < patches.size())
    {
        encloses = false;
        j = 0;

        //printf("%s << i = %d\n", __FUNCTION__, i);

        while ((j < patches.size()) && !encloses)
        {

            //printf("%s << j = %d\n", __FUNCTION__, j);

            if (j != i)
            {
                if (patches.at(i).area >= patches.at(j).area)
                {

                    //printf("%s << i = %d\n", __FUNCTION__, i);
                    //printf("%s << j = %d\n", __FUNCTION__, j);

                    //printf("%s << i area is larger than j.\n", __FUNCTION__);
                    // Check if the first MSER is within the second MSER
                    enclosureScore = pointPolygonTest(Mat(patches.at(i).hull), patches.at(j).centroid, false);

                    //printf("%s << enclosureScore = %f\n", __FUNCTION__, enclosureScore);

                    if (enclosureScore > 0.0)
                    {
                        encloses = true;
                        //printf("%s << patch [%d] encloses patch [%d]\n", __FUNCTION__, i, j);
                    }
                }
            }

            j++;
        }



        if (encloses)
        {
            patches.erase(patches.begin() + i);
            msers.erase(msers.begin() + i);
        }
        else
        {
            i++;
        }
    }

    //printf("%s << patches.size() = %d\n", __FUNCTION__, patches.size());

    return;

    // old enclosureFilter() ................

    //printf("patches.at(0).hull.size() = %d\n", patches.at(0).hull.size());
    vector<mserPatch> newPatches;
    vector<vector<Point> > newMsers;
    double enclosed = -1.0;
    Point avCenter(0, 0);
    unsigned int accumulatedCount = 0;

    for (unsigned int i = 0; i < patches.size()-1; i++)
    {
        // Assumed it's not 'enclosed' by another MSER
        enclosed = -1.0;

        // First point will always be the smallest in a series
        if (i == 0)
        {
            newPatches.push_back(patches.at(i));
            newMsers.push_back(msers.at(i));

            avCenter += patches.at(i).centroid;		// Add centroid to accumulated center
            accumulatedCount++;
        }


        // If the area of the 2nd one is greater
        if (patches.at(i).area < patches.at(i+1).area)
        {
            // Check if the first MSER is within the second MSER
            enclosed = pointPolygonTest(Mat(patches.at(i+1).hull), patches.at(i).centroid, false);
        }


        // If new point does not enclose current point, you can add new point to the newMsers vector
        if (enclosed < 0.0)
        {
            //printf("not enclosed\n");

            // Want to correct the last added point to be the mean of all of the 'series'

            //printf("%s << Changing centroid: (%d, %d) to ", __FUNCTION__, newPatches.at(newPatches.size()-1).centroid.x, newPatches.at(newPatches.size()-1).centroid.y);
            //newPatches.at(newPatches.size()-1).centroid = Point(avCenter.x/accumulatedCount, avCenter.y/accumulatedCount);
            //newPatches.at(newPatches.size()-1).centroid = patches.at(i).centroid;
            //printf("(%d, %d)\n", newPatches.at(newPatches.size()-1).centroid.x, newPatches.at(newPatches.size()-1).centroid.y);
            //cin.get();


            accumulatedCount = 0;
            avCenter = Point(0, 0);

            newPatches.push_back(patches.at(i+1));
            newMsers.push_back(msers.at(i+1));
        }

        avCenter += patches.at(i+1).centroid;
        accumulatedCount++;

    }

    patches.clear();
    msers.clear();

    for (unsigned int i = 0; i < newPatches.size(); i++)
    {
        patches.push_back(newPatches.at(i));
        msers.push_back(newMsers.at(i));
    }

}

void reduceCluster(vector<mserPatch>& patches, vector<vector<Point> >& msers, int totalPatches)
{
	
    // While the number of patches is larger than totalPatches, this function will
    // eliminate the patch that causes the largest change in area of a convex hull fitted to all patch
    // centers, when it's removed
    
    // Suggested improvements:
    //		A problem exists currently if there are several incorrect blobs that are spatially near each 
    //		other, but away from the main pattern. Removing a single one of these blobs is unlikely to 
    //		reduce the size of the convex hull considerably, so a real blob may be removed preferentially.
    
    //		Perhaps if there several more blobs than required, and for an iteration no single blob has 
    //		a substantially larger reduction in overall area than any others, you can look at the effect
    //		of removing PAIRS of blobs and vice versa. PAIRS will be selected on the basis of for a single 
    //		blob, which other blob has the closest center to it.
    
    // 		IN ANY CASE, this function should be prevented from reducing the blob count if it doesn't have 
	//		strong enough evidence to reduce any single blob. A later filter then may be able to reduce 
	//		the blob count to the correct number, but if not, it is preferable to return a FAIL in 
	//		detecting a pattern, than to incorrectly return the pattern location (rather than relying 
	//		on all the verification so much).

    vector<Point> pointsForArea;
    RotatedRect wrapperRectangle;
    Point2f circleCenter;
    float radius;

    double totalArea;
    double newArea;
    double maxDiff = 0;
    int maxIndex = 0;

    // While there are too many patches
    while (patches.size() > ((unsigned int)totalPatches))
    {
        maxDiff = 0;
        maxIndex = 0;

        // calculate total current area
        pointsForArea.clear();
        for (unsigned int i = 0; i < patches.size(); i++)
        {
            pointsForArea.push_back(patches.at(i).centroid);
        }

        wrapperRectangle = fitEllipse(Mat(pointsForArea));
        minEnclosingCircle(Mat(pointsForArea), circleCenter, radius);

        // totalArea = contourArea(Mat(pointsForArea));
        // totalArea = (wrapperRectangle.size.width) * (wrapperRectangle.size.height);
        totalArea = radius;

        //printf("%s << totalArea = %f\n", __FUNCTION__, totalArea);

        for (unsigned int i = 0; i < patches.size(); i++)
        {

            // determine newArea when element i is removed:
            pointsForArea.clear();
            for (unsigned int j = 0; j < patches.size(); j++)
            {
                if (j != i)
                {
                    //printf("%s << pushing back... [%d] / %d,%d\n", __FUNCTION__, j, patches.at(j).centroid.x, patches.at(j).centroid.y);
                    pointsForArea.push_back(patches.at(j).centroid);
                }
            }

            wrapperRectangle = fitEllipse(Mat(pointsForArea));
            minEnclosingCircle(Mat(pointsForArea), circleCenter, radius);

            //newArea = contourArea(Mat(pointsForArea));
            //newArea = (wrapperRectangle.size.width) * (wrapperRectangle.size.height);
            newArea = radius;

            //printf("%s << newArea[%d] = %f\n", __FUNCTION__, i, newArea);

            // If the area reduction is maximal, record:
            if ((totalArea - newArea) > maxDiff)
            {
                maxDiff = totalArea - newArea;
                maxIndex = i;

                //printf("%s << maxIndex = [%d]\n", __FUNCTION__, i);
            }

        }

        // remove the offending patch from the patches vector

        patches.erase(patches.begin() + maxIndex);
        msers.erase(msers.begin() + maxIndex);

    }

    return;
    
}

void clusterFilter(vector<mserPatch>& patches, vector<vector<Point> >& msers, int totalPatches)
{
	// Suggested improvements:
	//		At the moment, only the patch area is considered. Should re-introduce looking at other 
	//		properties such as the thinness and graylevels, but only smartly.
	//		Perhaps another property that could be looked at would be for each point, getting the 
	//		median distance to all other points..
	
	vector<double> patchAreas, patchAreasSorted;
	
	for (unsigned int iii = 0; iii < patches.size(); iii++) {
		
		patchAreas.push_back(patches.at(iii).area);
		
	}
	
	while (patches.size() > ((unsigned int)totalPatches)) {
		
		// Determine medians:
		patchAreasSorted.clear();
		patchAreasSorted.insert(patchAreasSorted.end(), patchAreas.begin(), patchAreas.end());
		sort(patchAreasSorted.begin(), patchAreasSorted.end());
		
		
		
		for (unsigned int iii = 0; iii < patchAreasSorted.size(); iii++) {
			//printf("%s << Area(%d) = (%f)\n", __FUNCTION__, iii, patchAreasSorted.at(iii));
		}
		
		double medianVal = patchAreas.at(int(patchAreasSorted.size() / 2));
		
		double mean, stdev;
		
		calcParameters(patchAreasSorted, mean, stdev);
		
		//printf("%s << medianVal = (%f); mean = (%f); stdev = (%f)\n", __FUNCTION__, medianVal, mean, stdev);
		
		double maxDiff = -1.0;;
		int maxDiffIndex = -1;

		
		for (unsigned int iii = 0; iii < patchAreasSorted.size(); iii++) {
			//printf("%s << Area(%d) = (%f)\n", __FUNCTION__, iii, patchAreasSorted.at(iii));
		}
		
		for (unsigned int iii = 0; iii < patchAreasSorted.size(); iii++) {
			
			if ((iii > 0) && (iii < patchAreasSorted.size()-1)) {
				// continue;
			}
		
			double tempDiff;
			
			 
			/*
			if ((patchAreasSorted.at(iii) > medianVal) && (iii > 0)) {
				tempDiff = abs(patchAreasSorted.at(iii) - patchAreasSorted.at(iii-1));
				printf("%s << A For area(%d) = (%f), diff = (%f)\n", __FUNCTION__, iii, patchAreasSorted.at(iii), tempDiff);
			} else if ((patchAreasSorted.at(iii) < medianVal) && (iii < patchAreasSorted.size()-1)) {
				tempDiff = abs(patchAreasSorted.at(iii) - patchAreasSorted.at(iii+1));
				printf("%s << B For area(%d) = (%f), diff = (%f)\n", __FUNCTION__, iii, patchAreasSorted.at(iii), tempDiff);
			}
			*/
			
			tempDiff = abs(patchAreasSorted.at(iii) - mean);
			
			if (tempDiff > maxDiff) {
				maxDiff = tempDiff;
				maxDiffIndex = iii;
			}
			
			
			// OLD
			/*
			if ((abs(patchAreasSorted.at(iii) - medianVal)) > maxDiff) {
				maxDiff = abs(patchAreasSorted.at(iii) - medianVal);
				maxDiffIndex = iii;
			}
			*/
			
			//printf("%s << area(%d) = %f (diff = %f\n", __FUNCTION__, iii, patchAreas.at(iii), abs(patchAreas.at(iii) - medianVal));
			
		}
		
		//printf("%s << maxDiffIndex = %d / %d\n", __FUNCTION__, maxDiffIndex, patchAreasSorted.size());
		
		double badArea = patchAreasSorted.at(maxDiffIndex);
		
		//printf("%s << Bad Area(%d) = (%f)\n", __FUNCTION__, maxDiffIndex, badArea);
		
		if (abs(badArea - mean)/stdev < 3.0) {
			//printf("%s << Worst area still within 3 standard devs\n", __FUNCTION__);
			return;
		} 
		
		bool culpritFound = false;
		unsigned int iii = 0;
		while (!culpritFound) {
			//printf("%s << iii = (%d / %d)\n", __FUNCTION__, iii, patchAreas.size());
			if (patchAreas.at(iii) == badArea) {
				culpritFound = true;
			} else {
				iii++;
			}
		}
		
		patchAreasSorted.erase(patchAreasSorted.begin() + maxDiffIndex);
		patchAreas.erase(patchAreas.begin() + iii);

		patches.erase(patches.begin() + iii);
		msers.erase(msers.begin() + iii);
		
		
	}
	
	
	return;
	
    // TODO:
    // maybe avoid discriminating based on intensity initially - since sometimes with the
    // chessboard you can get large clusters of white squares (and other background features) which
    // could throw off the algorithm.
    // get it to take into account colour if it's a multiple channel image

    vector<mserPatch> newPatches;
    vector<vector<Point> > newMsers;

    bool clusterFound = false;
    unsigned int ii = 0, jj = 0;
    double minDist, maxDist;
    double minArea, maxArea;
    Point centroidA, centroidB;
    double intensityA, intensityB;
    double areaA, areaB;
    double areaVar = 1.0;				// 0.5
    double intensityVar = 32.0;			// 32
    double distVar = 1.0;				// 1.0

    // newMsers is the new cluster:
    newPatches.push_back(patches.at(patches.size()-1));
    patches.pop_back();
    newMsers.push_back(msers.at(msers.size()-1));
    msers.pop_back();

    while (!clusterFound)
    {


        while (jj < newPatches.size())
        {

            // obtain interest point values
            centroidA = newPatches.at(jj).centroid;
            areaA = newPatches.at(jj).area;
            intensityA = newPatches.at(jj).meanIntensity;




            // obtain limits
            minArea = areaA/(1+areaVar);
            maxArea = areaA*(1+areaVar);
            minDist = (2*pow(areaA, 0.5))/(1+distVar);
            maxDist = (2*pow(areaA, 0.5))*(1+distVar);

            /*
            if (DEBUG_MODE > 3) {
            	printf("%s << area = %f; intensity = %f\n", __FUNCTION__, areaA, intensityA);
            	printf("%s << minArea = %f; maxArea = %f\n", __FUNCTION__, minArea, maxArea);
            	printf("%s << minDist = %f; maxDist = %f\n", __FUNCTION__, minDist, maxDist);
            }
            */

            while (ii < patches.size())
            {
                //printf("jj = %d; ii = %d.\n", jj, ii);

                // obtain comparison point values
                centroidB = patches.at(ii).centroid;
                areaB = patches.at(ii).area;
                intensityB = patches.at(ii).meanIntensity;

                /*
                if (DEBUG_MODE > 3) {
                	printf("%s << test pt area = %f; test pt intensity = %f\n", __FUNCTION__, areaB, intensityB);
                	waitKey(0);
                }
                */

                // Series of checks:

                // Distance check
                if ((distBetweenPts(centroidA, centroidB) < maxDist) && (distBetweenPts(centroidA, centroidB) > minDist))
                {
                    // Intensity check
                    if (abs(intensityA - intensityB) < intensityVar)
                    {
                        // Area check
                        if (max(areaA/areaB, areaB/areaA) < (1 + areaVar))
                        {
                            transferMserElement(newMsers, msers, ii);
                            transferPatchElement(newPatches, patches, ii);
                        }
                        else
                        {
                            ii++;
                        }
                    }
                    else
                    {
                        ii++;
                    }
                }
                else
                {
                    ii++;
                }
            }

            jj++;
            ii = 0;
        }

        if (DEBUG_MODE > 1)
        {
            printf("%s << Cluster Size = %d\n", __FUNCTION__, newMsers.size());
        }

        // Test
        if (newPatches.size() < ((unsigned int)totalPatches))   // if cluster has too few
        {
            if (patches.size() >= ((unsigned int)totalPatches))   // but remaining points are sufficient
            {
                newPatches.clear();
                newMsers.clear();

                newPatches.push_back(patches.at(patches.size()-1));
                patches.pop_back();

                newMsers.push_back(msers.at(msers.size()-1));
                msers.pop_back();

            }
            else        // and remaining points are insufficient
            {
                if (DEBUG_MODE > 1)
                {
                    printf("%s << No cluster is large enough.\n", __FUNCTION__);

                }
                return;
            }
        }
        else
        {
            clusterFound = true;
            if (DEBUG_MODE > 1)
            {
                printf("%s << Cluster found. size = %d.\n", __FUNCTION__, newPatches.size());

            }
        }

        ii = 0;
        jj = 0;
    }

    patches.clear();
    msers.clear();

    for (unsigned int i = 0; i < newPatches.size(); i++)
    {
        patches.push_back(newPatches.at(i));
        msers.push_back(newMsers.at(i));
    }

}

void transferPatchElement(vector<mserPatch>& dst, vector<mserPatch>& src, int index)
{
    // Move from old one to new one
    dst.push_back(src.at(index));

    // Replace point in old one with the end point
    src.at(index) = src.at(src.size()-1);

    // Truncate the original vector (effectively discarding old point)
    src.pop_back();
}

void transferMserElement(vector<vector<Point> >& dst, vector<vector<Point> >& src, int index)
{
    // Move from old one to new one
    dst.push_back(src.at(index));

    // Replace point in old one with the end point
    src.at(index) = src.at(src.size()-1);

    // Truncate the original vector (effectively discarding old point)
    src.pop_back();
}

bool refinePatches(const Mat& image, Size patternSize, vector<vector<Point> >& msers, vector<Point2f>& patchCentres, int mode)
{
    // TODO:
    // Lots of room for improvement here, in terms of both accuracy and speed.
    // For Mode 0: include white squares for as long as possible before applying the “colour” filter.
    // ensure that colour filter selects the darker of the two “modal” colours
    // (since a fair few will be white as well)
    // How about some kind of convex vs concave hull comparison? there shouldn't be much difference in area
    //      between these two for a sold square MSER.
    // More specific "TODO"s are included throught this function.
    
    Mat colorImage;
    
    if (DEBUG_MODE > 3) {
		if (image.channels() == 1) {
			cvtColor(image, colorImage, CV_GRAY2RGB);
		} else {
			image.copyTo(colorImage);
		}
		
	}

    int64 t = getTickCount();

    vector<mserPatch> patches;
    vector<vector<Point> > newMsers, newerMsers, tmpMsers;
    //int j = 0;
    //double area1, area2;
    vector<Point> hull;
    vector<Point> hull1, hull2;
    vector<Point> centroids;
    vector<Point> centroids2f;
    Moments moments1, moments2;
    //double x1c, y1c, x2c, y2c;
    Mat imCpy, dispMat;
    Scalar color(0, 0, 255);

    int x = patternSize.width, y = patternSize.height;
    double Xdb = double(x)/2, Ydb = double(y)/2;
    int X = x/2, Y = y/2;
    int totalPatches;

    if (mode == 0)
    {
        // Not fully tested or verified - check QCAT notebook for methodology
        totalPatches = int(floor(( 2*floor(Xdb)*floor(Ydb)+1+floor(Xdb)+floor(Ydb)+(ceil(Xdb)-floor(Xdb))*floor(Ydb)+(ceil(Ydb)-floor(Ydb))*floor(Xdb)+(ceil(Xdb)-floor(Xdb))*(ceil(Ydb)-floor(Ydb)) + 0.01)));
    }
    else if (mode == 1)
    {
        totalPatches = X*Y;
    }


    if (DEBUG_MODE > 1)
    {
        printf("%s << totalPatches = %d\n", __FUNCTION__, totalPatches);
        printf("%s << Patches found before refinement = %d\n", __FUNCTION__, msers.size());
    }


    // Convert msers to mserPatches (including hulls)
    patches.clear();
    for (unsigned int i = 0; i < msers.size(); i++)
    {
        patches.push_back(mserPatch(msers.at(i), image));
    }

    if (DEBUG_MODE > 3)
    {
        printf("%s << Patches found before filtering = %d\n", __FUNCTION__, msers.size());

        //color = Scalar(255, 255, 0);
        colorImage.copyTo(imCpy);
        drawContours(imCpy, msers, -1, color, 2);
        imshow("mainWin", imCpy);
        waitKey(0);
    }

    // ==============================SHAPE FILTER
    // TODO:
    // Improve height/width measurement so that it more accurately detects skinniness
    // (since angled, thin msers are still accepted at present)

    // Other option is to create an arbitrary square contour and compare other contours with it using
    // that OpenCV function

    t = getTickCount();

    shapeFilter(patches, msers);
    
    if (DEBUG_MODE > 0)
    {
        t = getTickCount() - t;
        printf("%s << Shape Filter duration: %fms\n", __FUNCTION__, t*1000/getTickFrequency());
        printf("%s << Patches found after shape filter = %d\n", __FUNCTION__, msers.size());
    }

    if (DEBUG_MODE > 6)
    {
        colorImage.copyTo(imCpy);
        drawContours(imCpy, msers, -1, color, 2);
        imshow("mainWin", imCpy);
        waitKey(0);
    }

    
    // ==============================END OF SHAPE FILTER

    // ==============================VARIANCE FILTER
	/*
    t = getTickCount();

    varianceFilter(patches, msers);

    if (DEBUG_MODE > 6)
    {

        //color = Scalar(255, 255, 0);
        image.copyTo(imCpy);
        drawContours(imCpy, msers, -1, color, 2);
        if (image.cols > 640)
        {
            resize(imCpy, dispMat, Size(0,0), 0.5, 0.5);
            imshow("mainWin", dispMat);
        }
        else
        {
            imshow("mainWin", imCpy);
        }
        waitKey(0);
    }

    if (DEBUG_MODE > 0)
    {
        t = getTickCount() - t;
        printf("%s << Variance Filter duration: %fms\n", __FUNCTION__, t*1000/getTickFrequency());
        printf("%s << Patches found after variance filter = %d\n", __FUNCTION__, msers.size());
    }
    */
    // ==============================END OF VARIANCE FILTER

    // ==============================ENCLOSURE FILTER
    t = getTickCount();

    enclosureFilter(patches, msers);

    // If you've lost too many, return a failure
    if (patches.size() < ((unsigned int)totalPatches))
    {
        if (DEBUG_MODE > 1)
        {
            printf("There are an insufficient (%d/%d) number of patches after enclosure filter.\n", msers.size(), totalPatches);
        }
        return false;
    }


    if (DEBUG_MODE > 1)
    {
        printf("%s << Patches found after enclosure filter = %d\n", __FUNCTION__, msers.size());
    }

    if (DEBUG_MODE > 6)
    {
        colorImage.copyTo(imCpy);
        color = Scalar(0, 255, 0);
        drawContours(imCpy, msers, -1, color, 2);
        imshow("mainWin", imCpy);
        waitKey(0);
    }

    if (DEBUG_MODE > 0)
    {
        t = getTickCount() - t;
        printf("%s << Enclosure Filter duration: %fms\n", __FUNCTION__, t*1000/getTickFrequency());
    }
    // ==============================END OF ENCLOSURE FILTER


    // ==============================
    // LOOPING FILTERS
    // ==============================
    //double minArea, maxArea, tmpArea;

    int previousPatches = 9999;

    // While the number of msers remaining is greater than the number you want
    // OR there has been no change since the previous cycle
    while ((msers.size() > ((unsigned int)totalPatches)) && (msers.size() < ((unsigned int)previousPatches)))
    {

        if (DEBUG_MODE > 1)
        {
            printf("%s << Looping through filters again...\n", __FUNCTION__);
        }

        previousPatches = int(msers.size());

        // ==============================CLUSTER FILTER
        t = getTickCount();
        clusterFilter(patches, msers, totalPatches);

        if (DEBUG_MODE > 1)
        {
            printf("%s << Patches found after cluster filter = %d\n", __FUNCTION__, msers.size());
        }

        if (DEBUG_MODE > 6)
        {
            //color = Scalar(0, 255, 0);
            colorImage.copyTo(imCpy);
            drawContours(imCpy, msers, -1, color, 2);
            imshow("mainWin", imCpy);
            waitKey(0);
        }

        // If the cluster was larger than m x n (probably (m x n)+1)
        if (patches.size() > ((unsigned int)totalPatches))
        {
            reduceCluster(patches, msers, totalPatches);
        }
        
        // COULD POTENTIALLY ADD ANOTHER "FILTER" HERE:
        // Some kind of straight-line test that scores each blob based on how many other blobs appear to 
        // be approximately "in-line" with it. Blobs that do not occur on lines could then be rejected.

        if (DEBUG_MODE > 1)
        {
            printf("%s << Patches remaining after area-based reduction = %d\n", __FUNCTION__, msers.size());
        }

        if (DEBUG_MODE > 6)
        {
            //color = Scalar(0, 255, 0);
            colorImage.copyTo(imCpy);
            drawContours(imCpy, msers, -1, color, 2);
            if (image.cols > 640)
            {
                resize(imCpy, dispMat, Size(0,0), 0.5, 0.5);
                imshow("mainWin", dispMat);
            }
            else
            {
                imshow("mainWin", imCpy);
            }
            waitKey(0);
        }

        if (DEBUG_MODE > 0)
        {
            t = getTickCount() - t;
            printf("%s << Cluster Filter duration: %fms\n", __FUNCTION__, t*1000/getTickFrequency());
        }



        // Anything else you can think of to remove outliers, if necessary


    }

    if (DEBUG_MODE > 3)
    {
        color = Scalar(255, 0, 0);
        colorImage.copyTo(imCpy);
        drawContours(imCpy, msers, -1, color, 2);
        if (colorImage.cols > 640)
        {
            resize(imCpy, dispMat, Size(0,0), 0.5, 0.5);
            imshow("mainWin", dispMat);
        }
        else
        {
            imshow("mainWin", imCpy);
        }
        waitKey(0);
    }

    //bool acceptable = false;

    if (int(msers.size()) == totalPatches)      // if the correct number has now been found, clear the old set and add the new set
    {

        // do some kind of check, to make sure that the last x*y/4 are truly a rectangular pattern
        if (DEBUG_MODE > 1)
        {
            printf("%s << Pattern believed to be found.\n", __FUNCTION__);
        }



        // Assign patch centres
        for (unsigned int i = 0; i < patches.size(); i++)
        {
            patchCentres.push_back(patches.at(i).centroid2f);
        }

        //acceptable = verifyPatches(image, patternSize, msers, patchCentres);
        //acceptable = verifyPatches(image.size(), patternSize, patchCentres, 0, 1000);

        return true;
    }
    else if (msers.size() > ((unsigned int)totalPatches))
    {
        if (DEBUG_MODE > 1)
        {
            printf("%s << Too many final patches = %d/%d\n", __FUNCTION__, msers.size(), totalPatches);
        }
        return false;
    }
    else
    {
        if (DEBUG_MODE > 1)
        {
            printf("%s << Too few final patches: %d/%d\n", __FUNCTION__, msers.size(), totalPatches);
        }
        return false;
    }
}
