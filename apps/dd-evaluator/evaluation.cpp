#include "evaluation.hpp"

#ifdef _IS_LINUX_

char matchesFilename[256];
bool printMatches;

static void usage(const char *argv0) {
	printf("Usage: %s [options]\n", argv0);
	printf("Supported options:\n\n");
    printf("-h [help]       		                    displays this menu\n");

	printf("-i [path]       		                    e.g. /home/$USER$/OpenCV/opencv_extras/testdata/cv\n");

	printf("-c                 		                    activate fusion mode for descriptors\n");

	printf("-d [dataset]       		                    e.g. aquavist, office, ...\n");
	printf("-m [modality]       		                e.g. thermal, visible, ...\n");
	printf("-s [subset]        		                    e.g. profile, ofb, ...\n");

    printf("-a        		                            regenerate ALL results\n");

    printf("-b        		                            regenerate homographies\n");
    printf("-f        		                            regenerate features\n");
    printf("-r        		                            regenerate detector results\n");
    printf("-e        		                            regenerate descriptors\n");
	printf("-g        		                            regenerate descriptor results\n");

	printf("-x 		                                    test across modalities\n");
    printf("-z		                                    display throughout process\n");

    printf("-n [no. of features]                        \n");

    printf("-w write images                             \n");

    printf("\n");
}

bool configData::processArguments(int argc, char** argv) {
    opterr = 0;
    int c;

    while ((c = getopt(argc, argv, "i:d:m:rgbs:xzwfeac")) != -1) {

        switch (c) {
            case 'i':
                dataPath = optarg;
                break;
            case 'c':
                fusionMode = true;
                break;
            case 'd':
                datasetString = optarg;
                singleDatasetOnly = true;
                break;
            case 'm':
                modalityString = optarg;
                singleModalityOnly = true;
                break;
            case 'g':
                regenerateDescriptorResults = true;
                break;
            case 'a':
                regenerateAllResults = true;
                break;
            case 'f':
                regenerateFeatures = true;
                break;
            case 'r':
                regenerateDetectorResults = true;
                break;
            case 'e':
                regenerateDescriptors = true;
                break;
            case 'b':
                regenerateHomographies = true;
                break;
            case 's':
                subsetString = optarg;
                singleSubsetOnly = true;
                break;
            case 'x':
                testAcrossModalities = true;
                break;
            case 'z':
                displayMode = true;
                break;
            case 'w':
                writeMode = true;
                break;
            default:
                printf("Invalid option -%c\n", c);
                printf("Run %s -h for help.\n", argv[0]);
                usage(argv[0]);
                return false;
        }

    }

    return true;
}

static inline Point2f applyHomography( const Mat_<double>& H, const Point2f& pt ) {
    double z = H(2,0)*pt.x + H(2,1)*pt.y + H(2,2);
    if( z )
    {
        double w = 1./z;
        return Point2f( (float)((H(0,0)*pt.x + H(0,1)*pt.y + H(0,2))*w), (float)((H(1,0)*pt.x + H(1,1)*pt.y + H(1,2))*w) );
    }
    return Point2f( numeric_limits<float>::max(), numeric_limits<float>::max() );
}

static inline void linearizeHomographyAt( const Mat_<double>& H, const Point2f& pt, Mat_<double>& A ) {
    A.create(2,2);
    double p1 = H(0,0)*pt.x + H(0,1)*pt.y + H(0,2),
           p2 = H(1,0)*pt.x + H(1,1)*pt.y + H(1,2),
           p3 = H(2,0)*pt.x + H(2,1)*pt.y + H(2,2),
           p3_2 = p3*p3;
    if( p3 )
    {
        A(0,0) = H(0,0)/p3 - p1*H(2,0)/p3_2; // fxdx
        A(0,1) = H(0,1)/p3 - p1*H(2,1)/p3_2; // fxdy

        A(1,0) = H(1,0)/p3 - p2*H(2,0)/p3_2; // fydx
        A(1,1) = H(1,1)/p3 - p2*H(2,1)/p3_2; // fydx
    }
    else
        A.setTo(Scalar::all(numeric_limits<double>::max()));
}

EllipticKeyPoint::EllipticKeyPoint() {
    *this = EllipticKeyPoint(Point2f(0,0), Scalar(1, 0, 1) );
}

EllipticKeyPoint::EllipticKeyPoint( const Point2f& _center, const Scalar& _ellipse ) {
    center = _center;
    ellipse = _ellipse;

    Mat_<double> M = getSecondMomentsMatrix(_ellipse), eval;
    eigen( M, eval );
    assert( eval.rows == 2 && eval.cols == 1 );
    axes.width = 1.f / (float)sqrt(eval(0,0));
    axes.height = 1.f / (float)sqrt(eval(1,0));

    double ac_b2 = ellipse[0]*ellipse[2] - ellipse[1]*ellipse[1];
    boundingBox.width = (float)sqrt(ellipse[2]/ac_b2);
    boundingBox.height = (float)sqrt(ellipse[0]/ac_b2);
}

Mat_<double> EllipticKeyPoint::getSecondMomentsMatrix( const Scalar& _ellipse ) {
    Mat_<double> M(2, 2);
    M(0,0) = _ellipse[0];
    M(1,0) = M(0,1) = _ellipse[1];
    M(1,1) = _ellipse[2];
    return M;
}

Mat_<double> EllipticKeyPoint::getSecondMomentsMatrix() const {
    return getSecondMomentsMatrix(ellipse);
}

void EllipticKeyPoint::calcProjection( const Mat_<double>& H, EllipticKeyPoint& projection ) const {
    Point2f dstCenter = applyHomography(H, center);

    Mat_<double> invM; invert(getSecondMomentsMatrix(), invM);
    Mat_<double> Aff; linearizeHomographyAt(H, center, Aff);
    Mat_<double> dstM; invert(Aff*invM*Aff.t(), dstM);

    projection = EllipticKeyPoint( dstCenter, Scalar(dstM(0,0), dstM(0,1), dstM(1,1)) );
}

void EllipticKeyPoint::convert( const vector<KeyPoint>& src, vector<EllipticKeyPoint>& dst ) {
    if( !src.empty() )
    {
        dst.resize(src.size());
        for( size_t i = 0; i < src.size(); i++ )
        {
            float rad = src[i].size/2;
            assert( rad );
            float fac = 1.f/(rad*rad);
            dst[i] = EllipticKeyPoint( src[i].pt, Scalar(fac, 0, fac) );
        }
    }
}

void EllipticKeyPoint::convert( const vector<EllipticKeyPoint>& src, vector<KeyPoint>& dst ) {
    if( !src.empty() )
    {
        dst.resize(src.size());
        for( size_t i = 0; i < src.size(); i++ )
        {
            Size_<float> axes = src[i].axes;
            float rad = sqrt(axes.height*axes.width);
            dst[i] = KeyPoint(src[i].center, 2*rad );
        }
    }
}

void EllipticKeyPoint::calcProjection( const vector<EllipticKeyPoint>& src, const Mat_<double>& H, vector<EllipticKeyPoint>& dst ) {
    if( !src.empty() )
    {
        assert( !H.empty() && H.cols == 3 && H.rows == 3);
        dst.resize(src.size());
        vector<EllipticKeyPoint>::const_iterator srcIt = src.begin();
        vector<EllipticKeyPoint>::iterator       dstIt = dst.begin();
        for( ; srcIt != src.end(); ++srcIt, ++dstIt )
            srcIt->calcProjection(H, *dstIt);
    }
}

static void filterEllipticKeyPointsByImageSize( vector<EllipticKeyPoint>& keypoints, const Size& imgSize ) {
    if( !keypoints.empty() )
    {
        vector<EllipticKeyPoint> filtered;
        filtered.reserve(keypoints.size());
        vector<EllipticKeyPoint>::const_iterator it = keypoints.begin();
        for( int i = 0; it != keypoints.end(); ++it, i++ )
        {
            if( it->center.x + it->boundingBox.width < imgSize.width &&
                it->center.x - it->boundingBox.width > 0 &&
                it->center.y + it->boundingBox.height < imgSize.height &&
                it->center.y - it->boundingBox.height > 0 )
                filtered.push_back(*it);
        }
        keypoints.assign(filtered.begin(), filtered.end());
    }
}

static void computeOneToOneMatchedOverlaps( const vector<EllipticKeyPoint>& keypoints1, const vector<EllipticKeyPoint>& keypoints2t,
                                            bool commonPart, vector<SIdx>& overlaps, float minOverlap ) {
    CV_Assert( minOverlap >= 0.f );
    overlaps.clear();
    if( keypoints1.empty() || keypoints2t.empty() )
        return;

    overlaps.clear();
    overlaps.reserve(cvRound(keypoints1.size() * keypoints2t.size() * 0.01));

    for( size_t i1 = 0; i1 < keypoints1.size(); i1++ )
    {
        EllipticKeyPoint kp1 = keypoints1[i1];
        float maxDist = sqrt(kp1.axes.width*kp1.axes.height),
              fac = 30.f/maxDist;
        if( !commonPart )
            fac=3;

        maxDist = maxDist*4;
        fac = 1.f/(fac*fac);

        EllipticKeyPoint keypoint1a = EllipticKeyPoint( kp1.center, Scalar(fac*kp1.ellipse[0], fac*kp1.ellipse[1], fac*kp1.ellipse[2]) );

        for( size_t i2 = 0; i2 < keypoints2t.size(); i2++ )
        {
            EllipticKeyPoint kp2 = keypoints2t[i2];
            Point2f diff = kp2.center - kp1.center;

            if( norm(diff) < maxDist )
            {
                EllipticKeyPoint keypoint2a = EllipticKeyPoint( kp2.center, Scalar(fac*kp2.ellipse[0], fac*kp2.ellipse[1], fac*kp2.ellipse[2]) );
                //find the largest eigenvalue
                int maxx =  (int)ceil(( keypoint1a.boundingBox.width > (diff.x+keypoint2a.boundingBox.width)) ?
                                     keypoint1a.boundingBox.width : (diff.x+keypoint2a.boundingBox.width));
                int minx = (int)floor((-keypoint1a.boundingBox.width < (diff.x-keypoint2a.boundingBox.width)) ?
                                    -keypoint1a.boundingBox.width : (diff.x-keypoint2a.boundingBox.width));

                int maxy =  (int)ceil(( keypoint1a.boundingBox.height > (diff.y+keypoint2a.boundingBox.height)) ?
                                     keypoint1a.boundingBox.height : (diff.y+keypoint2a.boundingBox.height));
                int miny = (int)floor((-keypoint1a.boundingBox.height < (diff.y-keypoint2a.boundingBox.height)) ?
                                    -keypoint1a.boundingBox.height : (diff.y-keypoint2a.boundingBox.height));
                int mina = (maxx-minx) < (maxy-miny) ? (maxx-minx) : (maxy-miny) ;

                //compute the area
                float dr = (float)mina/50.f;
                int N = (int)floor((float)(maxx - minx) / dr);
                IntersectAreaCounter ac( dr, minx, miny, maxy, diff, keypoint1a.ellipse, keypoint2a.ellipse );
                parallel_reduce( BlockedRange(0, N+1), ac );
                if( ac.bna > 0 )
                {
                    float ov =  (float)ac.bna / (float)ac.bua;
                    if( ov >= minOverlap )
                        overlaps.push_back(SIdx(ov, (int)i1, (int)i2));
                }
            }
        }
    }

    sort( overlaps.begin(), overlaps.end() );

    typedef vector<SIdx>::iterator It;

    It pos = overlaps.begin();
    It end = overlaps.end();

    while(pos != end)
    {
        It prev = pos++;
        end = std::remove_if(pos, end, SIdx::UsedFinder(*prev));
    }
    overlaps.erase(pos, overlaps.end());
}

void calculateRepeatability( const Mat& img1, const Mat& img2, const Mat& H1to2,
                                    const vector<KeyPoint>& _keypoints1, const vector<KeyPoint>& _keypoints2,
                                    float& repeatability, int& correspondencesCount, float overlapThreshold,
                                    Mat* thresholdedOverlapMask  ) {
    vector<EllipticKeyPoint> keypoints1, keypoints2, keypoints1t, keypoints2t;
    EllipticKeyPoint::convert( _keypoints1, keypoints1 );
    EllipticKeyPoint::convert( _keypoints2, keypoints2 );

    //printf("%s << Got to here.\n", __FUNCTION__);

    // calculate projections of key points
    EllipticKeyPoint::calcProjection( keypoints1, H1to2, keypoints1t );
    Mat H2to1; invert(H1to2, H2to1);
    EllipticKeyPoint::calcProjection( keypoints2, H2to1, keypoints2t );

    //float overlapThreshold;
    bool ifEvaluateDetectors = thresholdedOverlapMask == 0;
    if( ifEvaluateDetectors )
    {
        overlapThreshold = 1.f - overlapThreshold;  // inverting it because the error rather than the threshold is passed in..

        // remove key points from outside of the common image part
        Size sz1 = img1.size(), sz2 = img2.size();
        filterEllipticKeyPointsByImageSize( keypoints1, sz1 );
        filterEllipticKeyPointsByImageSize( keypoints1t, sz2 );
        filterEllipticKeyPointsByImageSize( keypoints2, sz2 );
        filterEllipticKeyPointsByImageSize( keypoints2t, sz1 );
    }
    else
    {
        overlapThreshold = 1.f - 0.5f;

        thresholdedOverlapMask->create( (int)keypoints1.size(), (int)keypoints2t.size(), CV_8UC1 );
        thresholdedOverlapMask->setTo( Scalar::all(0) );
    }
    size_t minCount = min( keypoints1.size(), keypoints2t.size() );

    // calculate overlap errors
    vector<SIdx> overlaps;
    computeOneToOneMatchedOverlaps( keypoints1, keypoints2t, ifEvaluateDetectors, overlaps, overlapThreshold/*min overlap*/ );

    correspondencesCount = -1;
    repeatability = -1.f;
    if( overlaps.empty() )
        return;

    correspondencesCount = (int)overlaps.size();
    repeatability = minCount ? (float)correspondencesCount / minCount : -1;

    if( !ifEvaluateDetectors )
    {
        for( size_t i = 0; i < overlaps.size(); i++ )
        {
            int y = overlaps[i].i1;
            int x = overlaps[i].i2;
            thresholdedOverlapMask->at<uchar>(y,x) = 1;
        }
    }
}

void calculateMatchability( const Mat& img1, const Mat& img2, const Mat& H1to2,
                                    const vector<KeyPoint>& _keypoints1, const vector<KeyPoint>& _keypoints2,
                                    float& matchability, int& correspondencesCount, float overlapThreshold,
                                    Mat* thresholdedOverlapMask ) {

    int maximumValue = min(_keypoints1.size(), _keypoints2.size());
    double bestPrScore = 0.00, prScore = 0.00;

    // For each descriptor
    for (int ddd = 0; ddd < DD_DESCRIPTORS_COUNT; ddd++) {


        vector<KeyPoint> newPts1, newPts2;
        Mat desc1, desc2;

        newPts1.assign(_keypoints1.begin(), _keypoints1.end());
        newPts2.assign(_keypoints2.begin(), _keypoints2.end());

        if ((DD_DESCRIPTORS_NAMES[ddd] == "FREAK") || (DD_DESCRIPTORS_NAMES[ddd] == "SURF") || (DD_DESCRIPTORS_NAMES[ddd] == "SIFT") || (DD_DESCRIPTORS_NAMES[ddd] == "ORB") || (DD_DESCRIPTORS_NAMES[ddd] == "BRIEF")) {

            printf("%s << ( %s ) Descriptor...\n", __FUNCTION__, DD_DESCRIPTORS_NAMES[ddd].c_str());

            // Describe features
            Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create((DD_DESCRIPTORS_NAMES[ddd]).c_str());
            extractor->compute(img1, newPts1, desc1);
            extractor->compute(img2, newPts2, desc2);

            if ((newPts1.size() == 0) || (desc1.rows == 0) || (newPts2.size() == 0) || (desc2.rows == 0)) {
                continue;
            }

            printf("%s << (%d) -> (%d); (%d) -> (%d)\n", __FUNCTION__, newPts1.size(), desc1.rows, newPts2.size(), desc2.rows);

            Ptr<DescriptorMatcher> dmatcher;

            // Based on the descriptor matrix types, may use a different matcher:
            if (desc1.type() == CV_32FC1) {
                dmatcher = DescriptorMatcher::create("BruteForce");
            } else if (desc1.type() == CV_64FC1) {
                dmatcher = DescriptorMatcher::create("BruteForce");
            } else if (desc1.type() == CV_8UC1) {
                dmatcher = DescriptorMatcher::create("BruteForce-Hamming");
            } else {
            }

            // Match features
            vector<vector<DMatch> > matches1to2;
            dmatcher->knnMatch( desc1, desc2, matches1to2, 1 );

            if (matches1to2.size() == 0) {
                continue;
            }

            vector<vector<uchar> > *correctMatches1to2Mask, buf2;
            correctMatches1to2Mask = &buf2;

            printf("%s << DEBUG: %d\n", __FUNCTION__, 0);

            correctMatches1to2Mask->resize(matches1to2.size());

            printf("%s << DEBUG: %d\n", __FUNCTION__, 1);

            for( size_t i = 0; i < matches1to2.size(); i++ )
            {
                (*correctMatches1to2Mask)[i].resize(matches1to2[i].size());
                for( size_t j = 0;j < matches1to2[i].size(); j++ )
                {
                    int indexQuery = matches1to2[i][j].queryIdx;
                    int indexTrain = matches1to2[i][j].trainIdx;
                    (*correctMatches1to2Mask)[i][j] = (*thresholdedOverlapMask).at<uchar>( indexQuery, indexTrain );
                }
            }

            // 2-way matching
            vector<vector<DMatch> > matches2to1;
            dmatcher->knnMatch( desc2, desc1, matches2to1, 1 );

            if (matches1to2.size() == 0) {
                continue;
            }

            vector<vector<uchar> > *correctMatches2to1Mask, buf2b;
            correctMatches2to1Mask = &buf2b;

            printf("%s << DEBUG: %d\n", __FUNCTION__, 0);

            correctMatches2to1Mask->resize(matches2to1.size());

            printf("%s << DEBUG: %d\n", __FUNCTION__, 1);

            for( size_t i = 0; i < matches2to1.size(); i++ )
            {
                (*correctMatches2to1Mask)[i].resize(matches2to1[i].size());
                for( size_t j = 0;j < matches2to1[i].size(); j++ )
                {
                    int indexQuery = matches2to1[i][j].queryIdx;
                    int indexTrain = matches2to1[i][j].trainIdx;
                    (*correctMatches2to1Mask)[i][j] = (*thresholdedOverlapMask).at<uchar>( indexTrain, indexQuery );
                }
            }

            vector<Point2f> results;

            printf("%s << matches1to2.size() = %d (%d) (%d)\n", __FUNCTION__, matches1to2.size(), matches2to1.size(), maximumValue);

            computeRecallPrecisionCurveModified( matches1to2, *correctMatches1to2Mask, matches2to1, *correctMatches2to1Mask, results, maximumValue );

            if (results.size() == 0) {
                continue;
            }

            printf("%s << results.size() = %d\n", __FUNCTION__, results.size());

            smoothAndSubsamplePrecisionRecallPoints(results, 0.001);

            //printf("%s << AND HERE.\n", __FUNCTION__);

            prScore = calculatePrecisionRecallScore(results);

            printf("%s << prScore = %f\n", __FUNCTION__, prScore);

            if (prScore > bestPrScore) {
                bestPrScore = prScore;
            }

        }

    }



    matchability = bestPrScore;
}

void smoothAndSubsamplePrecisionRecallPoints(vector<Point2f>& pts, double resolution) {
    vector<Point2f> newPts;
    Point2f tmpPt;

    if (pts.size() == 0) {
        return;
    }

    /*
    ofstream myfile;

    char filename[256];

    sprintf(filename, "%s-%d.csv", "/home/steve/Desktop/precrecall-", ::globalCounter, ".csv");
    //::globalCounter++;
    myfile.open(filename);

    //myfile << "[";

    for (unsigned int iii = 0; iii < pts.size(); iii++) {
        myfile << pts.at(iii).x << "," << pts.at(iii).y << endl;


    }

   // myfile << "]";

    myfile.close();
    */

    sort(pts.begin(), pts.end(), sort_point2f_vector_by_x_val);

    // Remove duplicate precision values
    for (int iii = 0; iii < pts.size()-1; iii++) {
        if (pts.at(iii).x >= pts.at(iii+1).x) {
            pts.at(iii).y = float(max(pts.at(iii).y, pts.at(iii+1).y));
            pts.erase(pts.begin()+iii+1);
        }
    }

    // Lift recall levels
    for (int iii = 0; iii < pts.size()-1; iii++) {
        if (pts.at(iii).y >= pts.at(iii+1).y) {
            pts.at(iii+1).y = pts.at(iii).y;
        }
    }

    // Add starter and terminator
    if (pts.at(0).x != 0.00) {
        tmpPt.x = 0.00;
        tmpPt.y = 0.00;

        pts.insert(pts.begin(), tmpPt);

        if (pts.at(1).x > resolution) {
            tmpPt.x = pts.at(1).x - resolution;
            tmpPt.y = 0.00;

            pts.insert(pts.begin(), tmpPt);
        }


    }




    tmpPt.x = 1.00;
    tmpPt.y = pts.at(pts.size()-1).y;
    pts.push_back(tmpPt);

    int currIndex = 0;
    double prevRecall = 0.00, maxRecall = 0.00;

    int maxSamples = int(1.00/resolution);

    printf("%s << maxSamples = %d\n", __FUNCTION__, maxSamples);

    // For each subsample
    for (int iii = 0; iii < maxSamples; iii++) {

        // Aimed precision
        tmpPt.x = float(iii)*resolution;

        //printf("%s << iii = %d; pts.at(currIndex).x = %f; tmpPt.x = %f\n", __FUNCTION__, iii, pts.at(currIndex).x, tmpPt.x);

        // Want to find the original point with the highest precision but lower than the aimed precision..
        while (float(pts.at(currIndex).x) < float(tmpPt.x)) {

            //printf("%s << currIndex = %d; pts.at(currIndex).x = %f\n", __FUNCTION__, currIndex, pts.at(currIndex).x);
            currIndex++;
            prevRecall = pts.at(currIndex).y;
        }

        //printf("%s << prevRecall = %f\n", __FUNCTION__, prevRecall);

        // Assign the last
        tmpPt.y = prevRecall;
        newPts.push_back(tmpPt);

        //cin.get();


        /*

        if (currIndex > pts.size()) {
            prevRecall = maxRecall;
        } else {
            while ((pts.at(currIndex).x < tmpPt.x) && (pts.size() < currIndex-1)) {
                currIndex++;
                prevRecall = pts.at(currIndex).y;

                if (prevRecall > maxRecall) {
                    maxRecall = prevRecall;
                }
            }
        }

        */

    }

    tmpPt.x = 1.00;
    tmpPt.y = newPts.at(newPts.size()-1).y;
    newPts.push_back(tmpPt);

    newPts.swap(pts);

    //printf("%s << pts.at(0).x = %f\n", __FUNCTION__, pts.at(0).x);

    /*

    int initPadReq = (pts.at(0).x / resolution);
    int endPadReq = (pts.at(pts.size()-1).x / resolution);

    //printf("%s << initPadReq = %d; endPadReq = %d\n", __FUNCTION__, initPadReq, endPadReq);


    // Zero-pad at start
    for (int iii = 0; iii < initPadReq; iii++) {
        tmpPt.x = double(iii)*resolution;
        tmpPt.y = 0;
        pts.insert(pts.begin(), tmpPt);
    }

    // Max-pad at end
    for (int iii = 0; iii < endPadReq; iii++) {
        tmpPt.x = 1 - double(endPadReq-iii-1)*resolution;
        tmpPt.y = 0;
        pts.insert(pts.end(), tmpPt);
    }

    // Flatten large gaps

    for (int iii = 0; iii < pts.size()-1; iii++) {
        if (abs(float(pts.at(iii).x) - float(pts.at(iii+1).x)) > float(resolution)) {
            tmpPt.x = (pts.at(iii).x + pts.at(iii+1).x) / 2.0;
            tmpPt.y = pts.at(iii).y;
            pts.insert(pts.begin() + iii, tmpPt);
        }
    }

    //cin.get();

    */

    /*
    sprintf(filename, "%s-%d.csv", "/home/steve/Desktop/precrecall-x-", ::globalCounter, ".csv");

    myfile.open(filename);

    //myfile << "[";

    for (unsigned int iii = 0; iii < newPts.size(); iii++) {
        myfile << newPts.at(iii).x << "," << newPts.at(iii).y << endl;


    }

   // myfile << "]";

    myfile.close();

    ::globalCounter++;
    */
}

void cv::evaluateFeatureDetector( const Mat& img1, const Mat& img2, const Mat& H1to2,
                              vector<KeyPoint>* _keypoints1, vector<KeyPoint>* _keypoints2,
                              float& repeatability, int& correspCount,
                              const Ptr<FeatureDetector>& _fdetector ) {
    Ptr<FeatureDetector> fdetector(_fdetector);
    vector<KeyPoint> *keypoints1, *keypoints2, buf1, buf2;
    keypoints1 = _keypoints1 != 0 ? _keypoints1 : &buf1;
    keypoints2 = _keypoints2 != 0 ? _keypoints2 : &buf2;

    if( (keypoints1->empty() || keypoints2->empty()) && fdetector.empty() )
        CV_Error( CV_StsBadArg, "fdetector must be no empty when keypoints1 or keypoints2 is empty" );

    if( keypoints1->empty() )
        fdetector->detect( img1, *keypoints1 );
    if( keypoints2->empty() )
        fdetector->detect( img2, *keypoints2 );

    float overlapThreshold = EVALUATION_OVERLAP_THRESHOLD;

    calculateRepeatability( img1, img2, H1to2, *keypoints1, *keypoints2, repeatability, correspCount, overlapThreshold );
}

static inline float recall( int correctMatchCount, int correspondenceCount ) {
    return correspondenceCount ? (float)correctMatchCount / (float)correspondenceCount : -1;
}

static inline float precision( int correctMatchCount, int falseMatchCount ) {
    return correctMatchCount + falseMatchCount ? (float)correctMatchCount / (float)(correctMatchCount + falseMatchCount) : -1;
}

double calculatePrecisionRecallScore(vector<Point2f>& recallPrecisionCurve) {

    double minDiff = 1.00;
    double prScore = 0.00;
    double tmpVal = -1.00;

    for (int iii = 0; iii < recallPrecisionCurve.size(); iii++) {
        tmpVal = abs((1 - recallPrecisionCurve.at(iii).x) - recallPrecisionCurve.at(iii).y);

        //printf("%s << tmpVal = %f (%f, %f)\n", __FUNCTION__, tmpVal, recallPrecisionCurve.at(iii).x, recallPrecisionCurve.at(iii).y);
        //cin.get();
        if (tmpVal < minDiff) {
            minDiff = tmpVal;
            prScore = ((1 - recallPrecisionCurve.at(iii).x) + recallPrecisionCurve.at(iii).y) / 2.0;
        }
    }

    //cin.get();

    return prScore;
}

void createMatchingMatrix(Mat& matchingMatrix, const vector<vector<DMatch> >& matches) {
    matchingMatrix = Mat::zeros(matches.size(), matches.size(), CV_64FC1);

    // IM 1 to IM 2
    for (int iii = 0; iii < matches.size(); iii++) {
        for (int jjj = 0; jjj < matches[iii].size(); jjj++) {
            matchingMatrix.at<double>(iii, matches.at(iii).at(jjj).trainIdx) += matches.at(iii).at(jjj).distance;
        }
    }

}

void createMatchingMatrix(Mat& matchingMatrix, const vector<vector<DMatch> >& matches1to2, const vector<vector<DMatch> >& matches2to1) {
    matchingMatrix = Mat::zeros(matches1to2.size(), matches2to1.size(), CV_64FC1);

    Mat countMat = Mat::zeros(matches1to2.size(), matches2to1.size(), CV_64FC1);

    // IM 1 to IM 2
    for (int iii = 0; iii < matches1to2.size(); iii++) {
        for (int jjj = 0; jjj < matches1to2[iii].size(); jjj++) {
            matchingMatrix.at<double>(iii, matches1to2.at(iii).at(jjj).trainIdx) += matches1to2.at(iii).at(jjj).distance;
            countMat.at<double>(iii, matches1to2.at(iii).at(jjj).trainIdx) += 1.0;
        }
    }

    // IM 2 to IM 1
    for (int iii = 0; iii < matches2to1.size(); iii++) {
        for (int jjj = 0; jjj < matches2to1[iii].size(); jjj++) {
            matchingMatrix.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += matches2to1.at(iii).at(jjj).distance;
            countMat.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += 1.0;
        }
    }

    /*
    Mat matMatDisp = normForDisplay(countMat);
    imshow("testWin", matMatDisp);
    waitKey();
    */

    matchingMatrix /= 2.0;

}

/*
double calcDistance(double dist, double ratio, double *coeffs) {
    double retVal = 0.0;

    retVal = abs(coeffs[0] * dist - ratio + coeffs[2]) / sqrt(pow(coeffs[0], 2) + 1);

    return retVal;
}
*/

/*
double calcLinePerpDistance(double *line1, double *line2) {
    double retVal = 0.0;

    retVal = abs(line2[2] - line1[2]) / sqrt(pow(line1[0], 2) + pow(line1[1], 2));

    return retVal;
}
*/

/*
double reweightDistanceWithLinearSVM(double dist, double ratio, double gradient, double *shift) {
    double retVal = 0.0;    // ratio

    double LineNeg1[3], LineZero[3], LinePos1[3];

    LineNeg1[0] = LineZero[0] = LinePos1[0] = gradient;
    LineNeg1[1] = LineZero[1] = LinePos1[1] = -1;
    LineNeg1[2] = shift[0];
    LineZero[2] = shift[1];
    LinePos1[2] = shift[2];

    // Find distance between lines
    double negSeg = calcLinePerpDistance(LineNeg1, LineZero);
    double posSeg = calcLinePerpDistance(LinePos1, LineZero);

    //printf("%s << Distances between lines = (%f, %f)\n", __FUNCTION__, negSeg, posSeg);

    // Find distance from point to negative line
    double negDist = calcDistance(dist, ratio, LineNeg1);

    // Find distance from point to neutral line
    double zeroDist = calcDistance(dist, ratio, LineZero);

    // Find distance from point to positive line
    double posDist = calcDistance(dist, ratio, LinePos1);

    //printf("%s << distances = (%f, %f, %f)\n", __FUNCTION__, negDist, zeroDist, posDist);
    //cin.get();

    if (posDist < negDist) {
        retVal = -1.0 * zeroDist;
    } else {
        retVal = +1.0 * zeroDist;
    }

    retVal = retVal + 10.0;

    // Assign total score
    //printf("%s << dist/ratio = (%f, %f), retVal = %f\n", __FUNCTION__, dist, ratio, retVal);
    //cin.get();


    return retVal;
}
*/

void sortMatches(const vector<vector<DMatch> >& matches, vector<vector<double> >& scores, vector<vector<unsigned int> >& indices) {

    vector<vector<DMatch> > matchesCpy;

    // For each feature in image A
    for (int iii = 0; iii < matches.size(); iii++) {

        // Create "paired" vector for sorting
        vector<pair<double,unsigned int> > indexedScores;
        pair<double,unsigned int> pairVal;

        for (int jjj = 0; jjj < matches.at(iii).size(); jjj++) {
            pairVal.first = matches.at(iii).at(jjj).distance;
            pairVal.second = jjj;
            indexedScores.push_back(pairVal);

            // printf("%s << val(%d) = %f (%d)\n", __FUNCTION__, jjj, pairVal.first, pairVal.second);
        }

        // cin.get();

        // Sort paired vec
        sort(indexedScores.begin(), indexedScores.end());

        for (int jjj = 0; jjj < indexedScores.size(); jjj++) {

            // printf("%s << val(%d) = %f (%d)\n", __FUNCTION__, jjj, indexedScores.at(jjj).first, indexedScores.at(jjj).second);
        }

        // cin.get();

        vector<double> tmpScores;
        vector<unsigned int> tmpIndices;

        // Separate sorted vect
        for (int jjj = 0; jjj < indexedScores.size(); jjj++) {
            tmpScores.push_back(indexedScores.at(jjj).first);
            tmpIndices.push_back(indexedScores.at(jjj).second);
        }

        scores.push_back(tmpScores);
        indices.push_back(tmpIndices);

    }
}

double applyPolynomialCoefficients(double val, double* polyCoeffs, int polyOrder) {
    double retVal = 0.00;

    for (int iii = 0; iii < polyOrder+1; iii++) {
        retVal += polyCoeffs[iii]*pow(val, polyOrder-iii);
    }

    return retVal;
}

int determineCorrectMatches(Mat& flagMat, const vector<vector<DMatch> >& matches1to2, const vector<vector<uchar> >& correctMatches1to2Mask, const vector<vector<DMatch> >& matches2to1, const vector<vector<uchar> >& correctMatches2to1Mask) {


    int retVal = 0;

    // printf("%s << (%d, %d)\n", __FUNCTION__, correctMatches1to2Mask.size(), correctMatches2to1Mask.size());

    flagMat = Mat::zeros(correctMatches1to2Mask.size(), correctMatches2to1Mask.size(), CV_64FC1);

    for (int iii = 0; iii < correctMatches1to2Mask.size(); iii++) {
        for (int jjj = 0; jjj < correctMatches1to2Mask.at(iii).size(); jjj++) {
            if (correctMatches1to2Mask[iii][jjj]) {
                flagMat.at<double>(iii,matches1to2.at(iii).at(jjj).trainIdx) += 1.0;
                retVal++;
            }
        }
    }

    for (int iii = 0; iii < correctMatches2to1Mask.size(); iii++) {
        for (int jjj = 0; jjj < correctMatches2to1Mask.at(iii).size(); jjj++) {
            if (correctMatches2to1Mask[iii][jjj]) {
                flagMat.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += 1.0;
            }
        }
    }

    flagMat /= 2.0;

    return retVal;
}

void getOneWayPriorityScores(Mat& matchingMatrix, vector<vector<double> >& priorityScores, vector<vector<int> >& priorityMatches) {

    // The differences for this one are:
        // it doesn't check to make sure that it is the best match in the other direction
        // It prioritizes in terms of the row with the strongest best strength match

    Mat matchingCopy;
    matchingMatrix.copyTo(matchingCopy);

    int * alreadyMatched;
    alreadyMatched = new int[matchingMatrix.cols];

    vector<double> dummyVec;
    vector<int> dummyMatchesVec;

    for (int iii = 0; iii < matchingMatrix.rows; iii++) {
        priorityScores.push_back(dummyVec);
        priorityMatches.push_back(dummyMatchesVec);
        alreadyMatched[iii] = -1;
    }

    int matchesCount = 0, matchesAim = matchingMatrix.cols;

    while (matchesCount < matchesAim) {

        // For each image 1 feature
        for (int iii = 0; iii < matchingMatrix.rows; iii++) {

            //printf("%s << Searching for best match for feature [%d]\n", __FUNCTION__, iii);
            //cin.get();

            if (alreadyMatched[iii] != 1) {

                int bestIndex = -1;
                double bestScore = 9e99;

                // For each image 2 feature
                for (int jjj = 0; jjj < matchingMatrix.cols; jjj++) {
                    if ((matchingCopy.at<double>(iii,jjj) <= bestScore) && (matchingCopy.at<double>(iii,jjj) != -1.0)) {
                        bestScore = matchingCopy.at<double>(iii,jjj);
                        bestIndex = jjj;
                    }
                }

                // Then get that best score in that column

                int bestRecipIndex = -1;
                double bestRecipScore = 9e99;

                for (int kkk = 0; kkk < matchingMatrix.rows; kkk++) {
                    if ((matchingCopy.at<double>(kkk,bestIndex) <= bestRecipScore) && (matchingCopy.at<double>(kkk,bestIndex) != -1.0)) {
                        bestRecipScore = matchingCopy.at<double>(kkk,bestIndex);
                        bestRecipIndex = kkk;
                    }
                }

                if (bestRecipIndex == iii) {
                    // It's a 2-way best match!
                    matchesCount++;
                    alreadyMatched[iii] = 1;
                    matchingCopy.at<double>(iii,bestIndex) = -1.0;

                    vector<double> twoBestScores;
                    twoBestScores.push_back(bestScore);

                    // Search for 2nd best score:

                    double secondScore = 9e99;
                    for (int aaa = 0; aaa < matchingMatrix.cols; aaa++) {
                        if ((matchingCopy.at<double>(iii,aaa) <= secondScore) && (matchingCopy.at<double>(iii,aaa) != -1.0)) {
                            secondScore = matchingCopy.at<double>(iii,aaa);
                        }
                    }

                    for (int aaa = 0; aaa < matchingMatrix.rows; aaa++) {
                        if ((matchingCopy.at<double>(aaa,bestIndex) <= secondScore) && (matchingCopy.at<double>(aaa,bestIndex) != -1.0)) {
                            secondScore = matchingCopy.at<double>(aaa,bestIndex);
                        }
                    }

                    // Negate everything in that row and column...
                    for (int aaa = 0; aaa < matchingMatrix.cols; aaa++) {
                        matchingCopy.at<double>(iii,aaa) = -1.0;
                        matchingCopy.at<double>(aaa,bestIndex) = -1.0;
                    }

                    twoBestScores.push_back(secondScore);

                    priorityMatches.at(0).at(iii) = bestIndex;
                    priorityScores.at(iii) = twoBestScores;

                    //printf("%s << IMG MATCH [%d][%d] : %f, %f\n", __FUNCTION__, iii, priorityMatches.at(iii), priorityScores.at(iii).at(0), priorityScores.at(iii).at(1));

                    /*
                    Mat matMatDisp = normForDisplay(matchingCopy);
                    imshow("testWin", matMatDisp);
                    waitKey(40);
                    */

                }
            }
        }
    }

}

void getPriorityScores(const Mat& matchingMatrix, vector<vector<double> >& priorityScores, vector<int>& priorityMatches) {

    //return;

    // Potentially more efficient way of doing things:
        // Just search matrix for lowest match score, and record the next best in that 'cross' and blank out that cross,
        // then search for the lowest match score remaining in the matrix and so on and so forth...
        //
        // Actually many-many, 1-many, 1-1 can all use this basic approach, but just with different "blanking" strategies
        // e.g. 1-many would blank out the entire column, but not row
        // only many-many would not be able to record a valid "second best" score
            // and many-many would also have N^2 matches rather than just N for the others...

    //printf("%s << Entered.\n", __FUNCTION__);


    Mat matchingCopy;
    matchingMatrix.copyTo(matchingCopy);

    //printf("%s << DEBUG X[%d].\n", __FUNCTION__, 0);

    int alreadyMatched[1024];

    if ((matchingMatrix.cols > 1024) || (matchingMatrix.rows > 1024)) {
        printf("%s << ERROR! matchingMatrix.dim() is too large compared with 1024...\n", __FUNCTION__);
        cin.get();
    }

    vector<double> dummyVec;
    dummyVec.push_back(0.0);
    dummyVec.push_back(0.0);

    //printf("%s << DEBUG X[%d].\n", __FUNCTION__, 1);

    int matchesCount = 0, matchesAim;

    matchesAim = std::min(matchingMatrix.cols, matchingMatrix.rows);

    //printf("%s << DEBUG X[%d].\n", __FUNCTION__, 2);

    priorityScores.clear();
    priorityMatches.clear();

    for (int iii = 0; iii < matchingMatrix.rows; iii++) {
        priorityScores.push_back(dummyVec);
        priorityMatches.push_back(-1);
        alreadyMatched[iii] = -1;
    }

    //printf("%s << DEBUG X[%d].\n", __FUNCTION__, 3);

    vector<double> twoBestScores;
    twoBestScores.push_back(0.00);
    twoBestScores.push_back(0.00);

    //printf("%s << Starting loop.\n", __FUNCTION__);

    while (matchesCount < matchesAim) {

        // For each image 1 feature
        for (int iii = 0; iii < matchingMatrix.rows; iii++) {

            printf("%s << Searching for best match for feature [%d] (matchesCount = %d)\n", __FUNCTION__, iii, matchesCount);
            //cin.get();

            if (alreadyMatched[iii] != 1) {

                printf("%s << Invalid check.\n", __FUNCTION__);

                int bestIndex = -1;
                double bestScore = 9e99;

                printf("%s << DEBUG [%d]\n", __FUNCTION__, 0);

                // For each image 2 feature
                for (int jjj = 0; jjj < matchingMatrix.cols; jjj++) {
                    if ((matchingCopy.at<double>(iii,jjj) <= bestScore) && (matchingCopy.at<double>(iii,jjj) != -1.0)) {
                        bestScore = matchingCopy.at<double>(iii,jjj);
                        bestIndex = jjj;
                    }
                }

                printf("%s << DEBUG [%d]\n", __FUNCTION__, 1);

                // Then get that best score in that column

                int bestRecipIndex = -1;
                double bestRecipScore = 9e99;

                for (int kkk = 0; kkk < matchingMatrix.rows; kkk++) {
                    if ((matchingCopy.at<double>(kkk,bestIndex) <= bestRecipScore) && (matchingCopy.at<double>(kkk,bestIndex) != -1.0)) {
                        bestRecipScore = matchingCopy.at<double>(kkk,bestIndex);
                        bestRecipIndex = kkk;
                    }
                }

                printf("%s << DEBUG [%d] (%d vs %d) (%d)\n", __FUNCTION__, 2, bestRecipIndex, iii, bestIndex);



                if (bestRecipIndex == iii) {
                    // It's a 2-way best match!
                    matchesCount++;
                    alreadyMatched[iii] = 1;

                    printf("%s << DEBUG [%d]\n", __FUNCTION__, 30);

                    printf("%s << (%d, %d) of (%d, %d)\n", __FUNCTION__, iii, bestIndex, matchingCopy.rows, matchingCopy.cols);
                    matchingCopy.at<double>(iii,bestIndex) = -1.0;

                    printf("%s << DEBUG [%d]\n", __FUNCTION__, 31);

                    twoBestScores.at(0) = bestScore;

                    // Search for 2nd best score:

                    printf("%s << DEBUG [%d]\n", __FUNCTION__, 3);

                    double secondScore = 9e99;
                    for (int aaa = 0; aaa < matchingMatrix.cols; aaa++) {
                        if ((matchingCopy.at<double>(iii,aaa) <= secondScore) && (matchingCopy.at<double>(iii,aaa) != -1.0)) {
                            secondScore = matchingCopy.at<double>(iii,aaa);
                        }
                    }

                    printf("%s << DEBUG [%d]\n", __FUNCTION__, 4);

                    for (int aaa = 0; aaa < matchingMatrix.rows; aaa++) {
                        //printf("%s << ((%d, %d)) of (%d, %d)\n", __FUNCTION__, aaa, bestIndex, matchingCopy.rows, matchingCopy.cols);
                        if ((matchingCopy.at<double>(aaa,bestIndex) <= secondScore) && (matchingCopy.at<double>(aaa,bestIndex) != -1.0)) {
                            secondScore = matchingCopy.at<double>(aaa,bestIndex);
                        }
                    }

                    printf("%s << DEBUG [%d]\n", __FUNCTION__, 5);

                    // Negate everything in that row and column...
                    for (int aaa = 0; aaa < matchingMatrix.cols; aaa++) {
                        matchingCopy.at<double>(iii,aaa) = -1.0;

                    }

                    for (int aaa = 0; aaa < matchingMatrix.rows; aaa++) {
                        matchingCopy.at<double>(aaa,bestIndex) = -1.0;
                    }

                    printf("%s << DEBUG [%d]\n", __FUNCTION__, 6);

                    twoBestScores.at(1) = secondScore;

                    printf("%s << (%d) priorityMatches.size() = %d; priorityScores.size() = %d\n", __FUNCTION__, iii, priorityMatches.size(), priorityScores.size());

                    priorityMatches.at(iii) = bestIndex;
                    priorityScores.at(iii).at(0) = twoBestScores.at(0);
                    priorityScores.at(iii).at(1) = twoBestScores.at(1);

                    //printf("%s << IMG MATCH [%d][%d] : %f, %f\n", __FUNCTION__, iii, priorityMatches.at(iii), priorityScores.at(iii).at(0), priorityScores.at(iii).at(1));

                    printf("%s << DEBUG [%d]\n", __FUNCTION__, 7);

                    /*
                    Mat matMatDisp = normForDisplay(matchingCopy);
                    imshow("testWin", matMatDisp);
                    waitKey(40);
                    */

                }
            }

            //printf("%s << Skipped...\n", __FUNCTION__);
        }
    }

    printf("%s << Exiting.\n", __FUNCTION__);

}

void computeRecallPrecisionCurveModified( const vector<vector<DMatch> >& matches1to2,
                                      const vector<vector<uchar> >& correctMatches1to2Mask,
                                      const vector<vector<DMatch> >& matches2to1,
                                      const vector<vector<uchar> >& correctMatches2to1Mask,
                                      vector<Point2f>& recallPrecisionCurve,
                                      int correspCount) {

  printf("%s << DEBUG [%d]\n", __FUNCTION__, -3);

    // DISTANCE_MATCHING           0
    // DISTANCE_RATIO_MATCHING     1
    // SVM_MATCHING                2

    // ONE_WAY_MATCHING            0
    // TWO_WAY_RANK_MATCHING       1   // Not true two-way matching, but uses average rank in both directions...
    // TWO_WAY_PRIORITY_MATCHING   2

    int matchingMetric = DISTANCE_MATCHING;
    int matchingMethod = TWO_WAY_PRIORITY_MATCHING;

    // SVM DETAILS
        // THERMAL
            // BRIEF: -0.19;, SURF: -1.42; Sc8Tn32-r8: -0.95

        // VISIBLE
            // SURF: -2.32; Sc8Vn32-r8: -1.779404505624886

    double gradient = -2.2231;

        // THERMAL
            //double shift[3] = {2.71, 1.78, 0.87}; // BRIEF
            //double shift[3] = {1.86, 1.04, 0.21}; // SURF
            //double shift[3] = {1.70, 1.03, 0.37}; // Sc8Tn32-r8

        // VISIBLE
            // SURF: {1.51, 1.15, 0.79};
            // Sc8Vn32-r8: {1.907240016333909,   1.360760371925608,   0.825794377025795};

    double shift[3] = {1.51, 1.15, 0.79};

    CV_Assert( matches1to2.size() == correctMatches1to2Mask.size() );

    srand ( (unsigned int)(time(NULL)) );
    waitKey(50);

    vector<DMatchForEvaluation> allMatches;
    int correspondenceCount = 0, trueCorrespondences = 0;

    printf("%s << DEBUG [%d]\n", __FUNCTION__, -2);

    Mat matMatDisp;

    // Create true matches matrix
    Mat correctMatchFlags;
    trueCorrespondences = determineCorrectMatches(correctMatchFlags, matches1to2, correctMatches1to2Mask, matches2to1, correctMatches2to1Mask);

    /*
    matMatDisp = normForDisplay(correctMatchFlags);
    imshow("testWin", matMatDisp);
    waitKey();
    */

    // For outputting actual matches..

    ofstream myfile;

    if (printMatches) {
        myfile.open(matchesFilename);
        //cin.get();
    }
    /*
    ofstream myfile1, myfile2;

    myfile2.open("/home/steve/sparse-coding/matches/visible-desk-B-A.txt");
    */
    // -i /home/steve/features-data -m visible -s profile -g
    // { "", "", "", "", "", "", "", "", "", "" };


    printf("%s << DEBUG [%d] (%d) (%d)\n", __FUNCTION__, -1, matches1to2.size(), matches2to1.size());

    // Get 1-way matching matrices...

    // Create 2-way matching scores matrix
    Mat matchingMatrix;
    createMatchingMatrix(matchingMatrix, matches1to2, matches2to1);

    printf("%s << DEBUG [%d] matchingMatrix.size() = (%d, %d)\n", __FUNCTION__, 55, matchingMatrix.rows, matchingMatrix.cols);

    // Determine the N best matches
    vector<vector<double> > priorityScores(matchingMatrix.rows);
    vector<int> priorityMatches(matchingMatrix.rows);

    printf("%s << getting priority scores... (%d, %d), %d, %d\n", __FUNCTION__, matchingMatrix.rows, matchingMatrix.cols, priorityScores.size(), priorityMatches.size());
    getPriorityScores(matchingMatrix, priorityScores, priorityMatches);

    printf("%s << finished.\n", __FUNCTION__);

    // Get 1-way priority scores
    //vector<vector<double> > oneWayPriorityScores;
    //vector<vector<int> > oneWayPriorityMatches;

    //getOneWayPriorityScores(matchingMatrix, oneWayPriorityScores, oneWayPriorityMatches);

    /*
    matMatDisp = normForDisplay(matchingMatrix);
    imshow("testWin", matMatDisp);
    waitKey();
    */

    printf("%s << DEBUG [%d]\n", __FUNCTION__, 0);


    Mat rankMat = Mat::zeros(matches1to2.size(), matches2to1.size(), CV_64FC1);

    // For each original feature in image 1
    for (int iii = 0; iii < matches1to2.size(); iii++) {
        // For each potential feature in image 2
        for (int jjj = 0; jjj < matches2to1.size(); jjj++) {
            rankMat.at<double>(iii, matches1to2.at(iii).at(jjj).trainIdx) += (double) jjj;
        }
    }

    // For each original feature in image 2
    for (int iii = 0; iii < matches2to1.size(); iii++) {
        // For each potential feature in image 1
        for (int jjj = 0; jjj < matches1to2.size(); jjj++) {
            rankMat.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += (double) jjj;
        }
    }

    rankMat /= 2.0;

    printf("%s << DEBUG [%d]\n", __FUNCTION__, 1);

    // Now rank matrix should be complete...

    /*
    matMatDisp = normForDisplay(rankMat);
    imshow("testWin", matMatDisp);
    waitKey();
    */

    // Then, for each row of the "RankMat" you need to determine both the best and second best scores, as
    // these will be used to characterize the feature match..

    vector<vector<double> > rankPairs;
    vector<vector<int> > rankIndices;

    printf("%s << For each feature...\n", __FUNCTION__);

    vector<double> currBest;
    currBest.push_back(0.00);
    currBest.push_back(0.00);

    vector<int> currIndices;
    currIndices.push_back(0);
    currIndices.push_back(0);

    // For each original feature in image 1
    for (int iii = 0; iii < rankMat.rows; iii++) {

        printf("%s << rankMat(%d).size() = (%d, %d)\n", __FUNCTION__, iii, rankMat.rows, rankMat.cols);

        double bestRankScore = std::min(rankMat.at<double>(iii,0), rankMat.at<double>(iii,1));
        double secondRankScore = std::max(rankMat.at<double>(iii,0), rankMat.at<double>(iii,1));

        int bestRank, secondRank;

        if (rankMat.at<double>(iii,0) < rankMat.at<double>(iii,1)) {
            bestRank = 0;
            secondRank = 1;
        } else {
            bestRank = 1;
            secondRank = 0;
        }

        for (int jjj = 2; jjj < rankMat.cols; jjj++) {
            if (rankMat.at<double>(iii,jjj) <= bestRankScore) {
                secondRank = bestRank;
                bestRank = jjj;

                secondRankScore = bestRankScore;
                bestRankScore = rankMat.at<double>(iii,jjj);
            }
        }



        // Make sure you are assigning corresponding scores, rather than corresponding ranks...
        currBest.at(0) = (matchingMatrix.at<double>(iii,bestRank));        // rankMat
        currBest.at(1) = (matchingMatrix.at<double>(iii,secondRank));     // matchingMatrix

        currIndices.at(0) = (bestRank);        // rankMat
        currIndices.at(1) = (secondRank);     // matchingMatrix

        rankPairs.push_back(currBest);
        rankIndices.push_back(currIndices);

    }

    printf("%s << DEBUG [%d]\n", __FUNCTION__, 2);

    for( size_t i = 0; i < matches1to2.size(); i++ )
    {

        // DISTANCE_MATCHING           0
        // DISTANCE_RATIO_MATCHING     1
        // SVM_MATCHING                2

        // ONE_WAY_MATCHING            0
        // TWO_WAY_RANK_MATCHING       1   // Not true two-way matching, but uses average rank in both directions...
        // TWO_WAY_PRIORITY_MATCHING   2

        for( size_t j = 0; j < 1; j++ ) // matches1to2[i].size()
        {
            // Default match to be assigned...
            DMatchForEvaluation match = matches1to2[i][0];

            if (matchingMethod == ONE_WAY_MATCHING) {

                match.isCorrect = correctMatches1to2Mask[i][j];

                if (matchingMetric == DISTANCE_MATCHING) {

                    match.distance = matches1to2[i][j].distance;

                } else if (matchingMetric == DISTANCE_RATIO_MATCHING) {

                    match.distance = (matches1to2[i][j].distance / matches1to2[i][j+1].distance);

                } else if (matchingMetric == SVM_MATCHING) {

                    match.distance = reweightDistanceWithLinearSVM(matches1to2[i][j].distance, (matches1to2[i][j].distance / matches1to2[i][j+1].distance), gradient);
                }




            } else if (matchingMethod == TWO_WAY_RANK_MATCHING) {
                if (correctMatchFlags.at<double>(i, rankIndices.at(i).at(0))) {
                    match.isCorrect = true;
                } else {
                    match.isCorrect = false;
                }

                match.trainIdx = rankIndices.at(i).at(0);

                if (matchingMetric == DISTANCE_MATCHING) {

                    match.distance = rankPairs.at(i).at(0);

                } else if (matchingMetric == DISTANCE_RATIO_MATCHING) {

                    match.distance = rankPairs.at(i).at(0) / rankPairs.at(i).at(1);

                } else if (matchingMetric == SVM_MATCHING) {

                    match.distance = reweightDistanceWithLinearSVM(rankPairs.at(i).at(0), (rankPairs.at(i).at(0) / rankPairs.at(i).at(1)), gradient);
                }

            } else if (matchingMethod == TWO_WAY_PRIORITY_MATCHING) {
                if (correctMatchFlags.at<double>(i, priorityMatches.at(i))) {
                    match.isCorrect = true;
                } else {
                    match.isCorrect = false;
                }

                match.trainIdx = priorityMatches.at(i);

                if (matchingMetric == DISTANCE_MATCHING) {

                    match.distance = priorityScores.at(i).at(0);

                } else if (matchingMetric == DISTANCE_RATIO_MATCHING) {

                    match.distance = priorityScores.at(i).at(0) / priorityScores.at(i).at(1);

                } else if (matchingMetric == SVM_MATCHING) {

                    match.distance = reweightDistanceWithLinearSVM(priorityScores.at(i).at(0), (priorityScores.at(i).at(0) / priorityScores.at(i).at(1)), gradient);

                }

            }

            if (printMatches) {

                myfile << priorityScores.at(i).at(0) << "," << (priorityScores.at(i).at(0) / priorityScores.at(i).at(1)) << ",";

                if (match.isCorrect) {
                    myfile << 1 << endl;
                } else {
                    myfile << 0 << endl;
                }

                if (0) {
                    if (match.isCorrect) {
                        // printf("%s << match[%d][%d] is correct.\n", __FUNCTION__, i, j);
                        myfile << matches1to2[i][j].trainIdx << " " << matches1to2[i][j].queryIdx << endl;
                    } else {
                        myfile << -1 << " " << matches1to2[i][j].queryIdx << endl;
                    }
                }

            }

            /*

            */

            allMatches.push_back( match );
            correspondenceCount += match.isCorrect != 0 ? 1 : 0;

        }

        //double polyCoeffs[3] = { -2.728346160934208,   2.953637715536362,   0.200685734845358 };
        //double scoreRatio = (rankPairs.at(i).at(0) / rankPairs.at(i).at(1));
        //scoreRatio -= applyPolynomialCoefficients(rankPairs.at(i).at(0), polyCoeffs, 2);

        //printf("%s << orig: (%f, %f) -> (%f)\n", __FUNCTION__, rankPairs.at(i).at(0), (rankPairs.at(i).at(0) / rankPairs.at(i).at(1)), scoreRatio);
        //cin.get();

        // 1-way SVM matching
        //match.distance = reweightDistanceWithLinearSVM(matches1to2[i][j].distance, (matches1to2[i][j].distance / matches1to2[i][j+1].distance), gradient, shift);

        // 1-way SVM matching but using 2-way match scores and 1-way ratio...
        //match.distance = reweightDistanceWithLinearSVM(rankPairs.at(i).at(0), (matches1to2[i][j].distance / matches1to2[i][j+1].distance), gradient, shift);

        // Cluster centres:
        // VISIBLE
            // SURF: 0.096, 0.417 / 0.253, 0.799
        //double validScore = 0.096, validRatio = 0.417, vacantScore = 0.253, vacantRatio = 0.799;
        //double distToValid, distToVacant;

        // 1-way cluster sorting
        //distToValid = sqrt(pow(matches1to2[i][j].distance - validScore, 2) + pow((matches1to2[i][j].distance / matches1to2[i][j+1].distance) - validRatio, 2));
        //distToVacant = sqrt(pow(matches1to2[i][j].distance - vacantScore, 2) + pow((matches1to2[i][j].distance / matches1to2[i][j+1].distance) - vacantRatio, 2));
        //match.distance = distToValid - distToVacant;
        // 2-way cluster sorting
        /*
        distToValid = sqrt(pow(rankPairs.at(i).at(0) - validScore, 2) + pow((rankPairs.at(i).at(0) / rankPairs.at(i).at(1)) - validRatio, 2));
        distToVacant = sqrt(pow(rankPairs.at(i).at(0) - vacantScore, 2) + pow((rankPairs.at(i).at(0) / rankPairs.at(i).at(1)) - vacantRatio, 2));
        match.distance = distToValid - distToVacant;
        */

        /*
        double score = matches1to2[i][j].distance;
        double factor = (matches1to2[i][j].distance / matches1to2[i][j+1].distance);
        */

        /*
        double score = rankPairs.at(i).at(0);
        double factor = (rankPairs.at(i).at(0) / rankPairs.at(i).at(1));


        double scoreKnee = 0.9;
        double factorKnee = 1.0 - scoreKnee;

        double scoreContrib = scoreKnee * min(score, scoreKnee) + factorKnee * max(score - scoreKnee, 0.0);
        double factorContrib = factorKnee * min(factor, factorKnee) + scoreKnee * max(factor - factorKnee, 0.0);

        match.distance = scoreContrib + factorContrib;
        */

        // Classifier-based matching

        //printf("%s << [%d,%d].distance = %f\n", __FUNCTION__, i, j, match.distance);
        //cin.get();


        //match.distance = rand() % 1000;
        //match.distance = 1 / std::abs(match.distance);
    }

    printf("%s << DEBUG [%d]\n", __FUNCTION__, 3);

    if (printMatches) {
        myfile.close();
    }
    //myfile.close();
    //cin.get();

    //printf("%s << allMatches.size() = %d\n", __FUNCTION__, allMatches.size());
    //printf("%s << correspondenceCount = %d / %d\n", __FUNCTION__, correspondenceCount, correspCount);
    //cin.get();

    // This should sort all matches into order from strongest/most confident to weakest.
    // However at present, allMatches probably only considers 2-way matching, rather than looking at average match strengths from the
    // perspectives of both images...
    //                  -- this has been fixed

    std::sort( allMatches.begin(), allMatches.end() );

    int correctMatchCount = 0, falseMatchCount = 0;

    printf("%s << allMatches.size() = %d\n", __FUNCTION__, allMatches.size());

    if (allMatches.size() < 1) {
        printf("%s << Potential error....\n");
        cin.get();
    }

    recallPrecisionCurve.resize( allMatches.size() );

    for( size_t i = 0; i < allMatches.size(); i++ )
    {
        if( allMatches[i].isCorrect ) {
            correctMatchCount++;
            //printf("%s << [%d]: correct\n", __FUNCTION__, i);
        } else {
            falseMatchCount++;
            //printf("%s << [%d]: false\n", __FUNCTION__, i);
        }

        //cin.get();

        //printf("%s << correspondenceCount = %d\n", __FUNCTION__, correspondenceCount);

        float r = recall( correctMatchCount, correspondenceCount );
        //float r = recall( correctMatchCount, correspCount );
        float p =  precision( correctMatchCount, falseMatchCount );

        printf("%s << processed match (%d): %d vs %d ((1-p) = %f vs r = %f)\n", __FUNCTION__, i, correctMatchCount, falseMatchCount, (1-p), r);

        //printf("%s << [%d]: (r, p) = (%f, %f)\n", __FUNCTION__, i, r, p);

        recallPrecisionCurve[i] = Point2f(1-p, r);

        //printf("%s << Here.\n", __FUNCTION__);
    }

    //cin.get();

    printf("%s << DEBUG [%d]\n", __FUNCTION__, 4);
}

float cv::getRecall( const vector<Point2f>& recallPrecisionCurve, float l_precision )
{
    int nearestPointIndex = getNearestPoint( recallPrecisionCurve, l_precision );

    float recall = -1.f;

    if( nearestPointIndex >= 0 )
        recall = recallPrecisionCurve[nearestPointIndex].y;

    return recall;
}

int cv::getNearestPoint( const vector<Point2f>& recallPrecisionCurve, float l_precision )
{
    int nearestPointIndex = -1;

    if( l_precision >= 0 && l_precision <= 1 )
    {
        float minDiff = FLT_MAX;
        for( size_t i = 0; i < recallPrecisionCurve.size(); i++ )
        {
            float curDiff = std::fabs(l_precision - recallPrecisionCurve[i].x);
            if( curDiff <= minDiff )
            {
                nearestPointIndex = (int)i;
                minDiff = curDiff;
            }
        }
    }

    return nearestPointIndex;
}

void evaluateGenericDescriptorMatcherModified( const Mat& img1, const Mat& img2, const Mat& H1to2,
                                           vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2,
                                           vector<vector<DMatch> >* _matches1to2, vector<vector<uchar> >* _correctMatches1to2Mask,
                                           vector<vector<DMatch> >* _matches2to1, vector<vector<uchar> >* _correctMatches2to1Mask,
                                           vector<Point2f>& recallPrecisionCurve,
                                           const Ptr<GenericDescriptorMatcher>& _dmatcher )
{
    Ptr<GenericDescriptorMatcher> dmatcher = _dmatcher;
    dmatcher->clear();

    vector<vector<DMatch> > *matches1to2, buf1;
    matches1to2 = _matches1to2 != 0 ? _matches1to2 : &buf1;

    vector<vector<uchar> > *correctMatches1to2Mask, buf2;
    correctMatches1to2Mask = _correctMatches1to2Mask != 0 ? _correctMatches1to2Mask : &buf2;

    vector<vector<DMatch> > *matches2to1, buf1b;
    matches2to1 = _matches2to1 != 0 ? _matches2to1 : &buf1b;

    vector<vector<uchar> > *correctMatches2to1Mask, buf2b;
    correctMatches2to1Mask = _correctMatches2to1Mask != 0 ? _correctMatches2to1Mask : &buf2b;

    if( keypoints1.empty() )
        CV_Error( CV_StsBadArg, "keypoints1 must be no empty" );

    if( matches1to2->empty() && dmatcher.empty() )
        CV_Error( CV_StsBadArg, "dmatch must be no empty when matches1to2 is empty" );

    if( matches2to1->empty() && dmatcher.empty() )
        CV_Error( CV_StsBadArg, "dmatch must be no empty when matches2to1 is empty" );

    bool computeKeypoints2ByPrj = keypoints2.empty();
    if( computeKeypoints2ByPrj )
    {
        assert(0);
        // TODO: add computing keypoints2 from keypoints1 using H1to2
    }

    if( matches1to2->empty() || computeKeypoints2ByPrj )
    {
        dmatcher->clear();
        dmatcher->radiusMatch( img1, keypoints1, img2, keypoints2, *matches1to2, std::numeric_limits<float>::max() );
    }

    if( matches2to1->empty() || computeKeypoints2ByPrj )
    {
        dmatcher->clear();
        dmatcher->radiusMatch( img2, keypoints2, img1, keypoints1, *matches2to1, std::numeric_limits<float>::max() );
    }

    float repeatability;
    int correspCount;
    Mat thresholdedOverlapMask; // thresholded allOverlapErrors
    calculateRepeatability( img1, img2, H1to2, keypoints1, keypoints2, repeatability, correspCount, EVALUATION_OVERLAP_THRESHOLD, &thresholdedOverlapMask );

    correctMatches1to2Mask->resize(matches1to2->size());
    for( size_t i = 0; i < matches1to2->size(); i++ )
    {
        (*correctMatches1to2Mask)[i].resize((*matches1to2)[i].size());
        for( size_t j = 0;j < (*matches1to2)[i].size(); j++ )
        {
            int indexQuery = (*matches1to2)[i][j].queryIdx;
            int indexTrain = (*matches1to2)[i][j].trainIdx;
            (*correctMatches1to2Mask)[i][j] = thresholdedOverlapMask.at<uchar>( indexQuery, indexTrain );
        }
    }

    // 2-way matching
    correctMatches2to1Mask->resize(matches2to1->size());
    for( size_t i = 0; i < matches2to1->size(); i++ )
    {
        (*correctMatches2to1Mask)[i].resize((*matches2to1)[i].size());
        for( size_t j = 0;j < (*matches2to1)[i].size(); j++ )
        {
            int indexQuery = (*matches2to1)[i][j].queryIdx;
            int indexTrain = (*matches2to1)[i][j].trainIdx;
            (*correctMatches2to1Mask)[i][j] = thresholdedOverlapMask.at<uchar>( indexTrain , indexQuery );
        }
    }

    computeRecallPrecisionCurveModified( *matches1to2, *correctMatches1to2Mask, *matches2to1, *correctMatches2to1Mask, recallPrecisionCurve, correspCount );
}

/****************************************************************************************\
*           Functions to evaluate affine covariant detectors and descriptors.            *
\****************************************************************************************/

static void calcKeyPointProjections( const vector<KeyPoint>& src, const Mat_<double>& H, vector<KeyPoint>& dst )
{
    if(  !src.empty() )
    {
        assert( !H.empty() && H.cols == 3 && H.rows == 3);
        dst.resize(src.size());
        vector<KeyPoint>::const_iterator srcIt = src.begin();
        vector<KeyPoint>::iterator       dstIt = dst.begin();
        for( ; srcIt != src.end(); ++srcIt, ++dstIt )
        {
            Point2f dstPt = applyHomography(H, srcIt->pt);

            float srcSize2 = srcIt->size * srcIt->size;
            Mat_<double> M(2, 2);
            M(0,0) = M(1,1) = 1./srcSize2;
            M(1,0) = M(0,1) = 0;
            Mat_<double> invM; invert(M, invM);
            Mat_<double> Aff; linearizeHomographyAt(H, srcIt->pt, Aff);
            Mat_<double> dstM; invert(Aff*invM*Aff.t(), dstM);
            Mat_<double> eval; eigen( dstM, eval );
            assert( eval(0,0) && eval(1,0) );
            float dstSize = (float)pow(1./(eval(0,0)*eval(1,0)), 0.25);

            // TODO: check angle projection
            float srcAngleRad = (float)(srcIt->angle*CV_PI/180);
            Point2f vec1(cos(srcAngleRad), sin(srcAngleRad)), vec2;
            vec2.x = (float)(Aff(0,0)*vec1.x + Aff(0,1)*vec1.y);
            vec2.y = (float)(Aff(1,0)*vec1.x + Aff(0,1)*vec1.y);
            float dstAngleGrad = fastAtan2(vec2.y, vec2.x);

            *dstIt = KeyPoint( dstPt, dstSize, dstAngleGrad, srcIt->response, srcIt->octave, srcIt->class_id );
        }
    }
}

static void filterKeyPointsByImageSize( vector<KeyPoint>& keypoints, const Size& imgSize )
{
    if( !keypoints.empty() )
    {
        vector<KeyPoint> filtered;
        filtered.reserve(keypoints.size());
        Rect r(0, 0, imgSize.width, imgSize.height);
        vector<KeyPoint>::const_iterator it = keypoints.begin();
        for( int i = 0; it != keypoints.end(); ++it, i++ )
            if( r.contains(it->pt) )
                filtered.push_back(*it);
        keypoints.assign(filtered.begin(), filtered.end());
    }
}

inline void writeKeypoints( FileStorage& fs, const vector<KeyPoint>& keypoints, int imgIdx )
{
    if( fs.isOpened() )
    {
        stringstream imgName; imgName << "img" << imgIdx;
        write( fs, imgName.str(), keypoints );
    }
}

void writeKeypoints_MATLAB( char *filename, const vector<KeyPoint>& keypoints )
{

    fstream filestr;

    filestr.open (filename, fstream::out | fstream::trunc);

    filestr << "1.0" << endl;
    filestr << keypoints.size() << endl;

    for (int iii = 0; iii < keypoints.size(); iii++) {
        filestr << keypoints.at(iii).pt.x << " " << keypoints.at(iii).pt.y << " ";
        filestr << keypoints.at(iii).size << " " << keypoints.at(iii).angle << " " << keypoints.at(iii).response << endl;
    }

    filestr.close();
}

void readKeypoints_MATLAB( char *filename, vector<KeyPoint>& keypoints )
{

    keypoints.clear();

    ifstream filestr(filename);

    string lineBuffer;

    float tmpFloat = 0.0;
    int keypointCount = 0;

    //getline(filestr, lineBuffer);
    //printf("%s << firstLine = %s\n", __FUNCTION__, lineBuffer.c_str());

    filestr >> tmpFloat;
    //printf("%s << tmpFloat = %f\n", __FUNCTION__, tmpFloat);
    filestr >> keypointCount;
    //printf("%s << keypointCount = %d\n", __FUNCTION__, keypointCount);

    KeyPoint currentKeypoint;

    for (int iii = 0; iii < keypointCount; iii++) {

        filestr >> currentKeypoint.pt.x >> currentKeypoint.pt.y >> currentKeypoint.size >> currentKeypoint.angle >> currentKeypoint.response;

        //printf("%s << vals = (%f, %f, %f, %f, %f)\n", __FUNCTION__, currentKeypoint.pt.x, currentKeypoint.pt.y, currentKeypoint.size, currentKeypoint.angle, currentKeypoint.response);

        keypoints.push_back(currentKeypoint);

        //cin.get();

    }


    filestr.close();
}

void writeDescriptors_MATLAB( char *filename, const vector<KeyPoint>& keypoints, const Mat& descriptors )
{

    // Should receive just a single Matrix, where each row represents the vector for a feature's descriptor

    fstream filestr;

    filestr.open (filename, fstream::out | fstream::trunc);

    filestr << "2.0" << endl;
    filestr << descriptors.rows << endl;



    if (descriptors.type() == CV_8UC1) {
        //printf("%s << Unsigned char.\n", __FUNCTION__);
        filestr << descriptors.cols*8 << endl;
    } else if (descriptors.type() == CV_8SC1) {
        //printf("%s << Signed char.\n", __FUNCTION__);
        filestr << descriptors.cols*8 << endl;
    } else if (descriptors.type() == CV_16UC1) {
        //printf("%s << Unsigned short.\n", __FUNCTION__);
        filestr << descriptors.cols << endl;
    } else if (descriptors.type() == CV_16SC1) {
        //printf("%s << Signed short.\n", __FUNCTION__);
        filestr << descriptors.cols << endl;
    } else if (descriptors.type() == CV_32SC1) {
        //printf("%s << Signed long.\n", __FUNCTION__);
        filestr << descriptors.cols << endl;
    } else if (descriptors.type() == CV_32FC1) {
        //printf("%s << Float.\n", __FUNCTION__);
        filestr << descriptors.cols << endl;
    }

    filestr << descriptors.type() << endl;

    //cin.get();

    int binaryArray[8];

    for (int iii = 0; iii < descriptors.rows; iii++) {

        filestr << keypoints.at(iii).pt.x << " " << keypoints.at(iii).pt.y << " ";
        filestr << keypoints.at(iii).size << " " << keypoints.at(iii).angle << " " << keypoints.at(iii).response << endl;

        for (int jjj = 0; jjj < descriptors.cols; jjj++) {



            if (descriptors.type() == CV_8UC1) {
                unsigned char val = descriptors.at<unsigned char>(iii, jjj);

                convertUcharToBinary(val, binaryArray);

                //printf("%s << val = %d\n", __FUNCTION__, val);

                //printf("%s << binary =", __FUNCTION__);

                for (int kkk = 0; kkk < 7; kkk++) {
                    //printf(" %d", binaryArray[kkk]);
                    filestr << binaryArray[kkk] << " ";
                }

                filestr << binaryArray[7];

                //printf("\n");

                // For each bit, spit out a zero or a one
                //cin.get();

                // filestr << val;
            } else if (descriptors.type() == CV_8SC1) {
                filestr << descriptors.at<char>(iii, jjj);
            } else if (descriptors.type() == CV_16UC1) {
                filestr << descriptors.at<unsigned short>(iii, jjj);
            } else if (descriptors.type() == CV_16SC1) {
                filestr << descriptors.at<short>(iii, jjj);
            } else if (descriptors.type() == CV_32SC1) {
                filestr << descriptors.at<int>(iii, jjj);
            } else if (descriptors.type() == CV_32FC1) {
                filestr << descriptors.at<float>(iii, jjj);
            } else {
                //printf("%s << Unable to write value to file...\n", __FUNCTION__);
            }

            if (jjj < (descriptors.cols-1)) {
                filestr << " ";
            }
        }

        if (iii < (descriptors.rows-1)) {
            filestr << endl;
        }

    }

    filestr.close();
}

void readDescriptors_MATLAB( char *filename, vector<KeyPoint>& keypoints, Mat& descriptors, unsigned int maxFeatures )
{

    //printf("%s << filename = %s\n", __FUNCTION__, filename);

    descriptors.release();

    ifstream filestr(filename);

    string lineBuffer;

    float tmpFloat = 0.0;
    unsigned int descriptorCount = 0, descriptorDim = 0, descriptorType = 0;

    //getline(filestr, lineBuffer);
    //printf("%s << firstLine = %s\n", __FUNCTION__, lineBuffer.c_str());

    filestr >> tmpFloat;
    //printf("%s << tmpFloat = %f\n", __FUNCTION__, tmpFloat);
    filestr >> descriptorCount;
    filestr >> descriptorDim;
    filestr >> descriptorType;
    //printf("%s << keypointCount = %d\n", __FUNCTION__, keypointCount);

    //printf("%s << tmpFloat = %f\n", __FUNCTION__, tmpFloat);
    //printf("%s << descriptorCount = %d\n", __FUNCTION__, descriptorCount);
    //printf("%s << descriptorDim = %d\n", __FUNCTION__, descriptorDim );
    //printf("%s << descriptorType = %d\n", __FUNCTION__, descriptorType);

    // cin.get();

    keypoints.clear();

    int totalFeatures = min(descriptorCount, maxFeatures);


    descriptors = Mat(Size(descriptorDim, totalFeatures), descriptorType);

    for (int iii = 0; iii < totalFeatures; iii++) {

        KeyPoint currentKeypoint;

        filestr >> currentKeypoint.pt.x >> currentKeypoint.pt.y >> currentKeypoint.size >> currentKeypoint.angle >> currentKeypoint.response;

        keypoints.push_back(currentKeypoint);

        for (int jjj = 0; jjj < descriptorDim; jjj++) {

            if (descriptorType == CV_8UC1) {
                filestr >> descriptors.at<unsigned char>(iii,jjj);
                //printf("%s << val(CV_8UC1) = %d\n", __FUNCTION__, descriptors.at<unsigned char>(iii,jjj));
            } else if (descriptorType == CV_32FC1) {
                filestr >> descriptors.at<float>(iii,jjj);
                //printf("%s << val(CV_32FC1) = %f\n", __FUNCTION__, descriptors.at<float>(iii,jjj));
            } else if (descriptorType == CV_64FC1) {
                filestr >> descriptors.at<double>(iii,jjj);
                //printf("%s << val(CV_64FC1) = %f\n", __FUNCTION__, descriptors.at<double>(iii,jjj));
            }

            //cin.get();
        }

    }

    filestr.close();
}

inline void readKeypoints( FileStorage& fs, vector<KeyPoint>& keypoints, int imgIdx )
{
    assert( fs.isOpened() );
    stringstream imgName; imgName << "img" << imgIdx;
    read( fs[imgName.str()], keypoints);
}



int update_progress( const string& /*name*/, int progress, int test_case_idx, int count, double dt )
{
    int width = 60 /*- (int)name.length()*/;
    if( count > 0 )
    {
        int t = cvRound( ((double)test_case_idx * width)/count );
        if( t > progress )
        {
            cout << "." << flush;
            progress = t;
        }
    }
    else if( cvRound(dt) > progress )
    {
        cout << "." << flush;
        progress = cvRound(dt);
    }

    return progress;
}



void testLog( bool isBadAccuracy )
{
    if( isBadAccuracy )
        printf(" bad accuracy\n");
    else
        printf("\n");
}

/****************************************************************************************\
*                                  Descriptors evaluation                                 *
\****************************************************************************************/

































/*
// Program arguments: /home/steve/OpenCV/opencv_extras/testdata/cv
int main( int argc, char** argv )
{
    if( argc != 2 )
    {
        cout << "Format: " << argv[0] << " testdata path (path to testdata/cv)" << endl;
        return -1;
    }

    data_path = argv[1];
#ifdef WIN32
    if( *data_path.rbegin() != '\\' )
        data_path = data_path + "\\";
#else
    if( *data_path.rbegin() != '/' )
        data_path = data_path + "/";
#endif

    Ptr<BaseQualityEvaluator> evals[] =
    {
        new DetectorQualityEvaluator( "ORB", "quality-detector-orb" ),
        //new DetectorQualityEvaluator( "FAST", "quality-detector-fast" ),
        //new DetectorQualityEvaluator( "GFTT", "quality-detector-gftt" ),
        //new DetectorQualityEvaluator( "HARRIS", "quality-detector-harris" ),
        //new DetectorQualityEvaluator( "MSER", "quality-detector-mser" ),
        //new DetectorQualityEvaluator( "STAR", "quality-detector-star" ),
        //new DetectorQualityEvaluator( "SIFT", "quality-detector-sift" ),
        //new DetectorQualityEvaluator( "SURF", "quality-detector-surf" ),


        //new DescriptorQualityEvaluator( "SIFT", "quality-descriptor-sift", "BruteForce" ),
        //new DescriptorQualityEvaluator( "SURF", "quality-descriptor-surf", "BruteForce" ),
        //new DescriptorQualityEvaluator( "FERN", "quality-descriptor-fern"),
        //new CalonderDescriptorQualityEvaluator()

    };

    for( size_t i = 0; i < sizeof(evals)/sizeof(evals[0]); i++ )
    {
        evals[i]->run();
        cout << endl;
    }
}
*/

void summarizeScores(vector<float> &scores, float *summ) {
    float mean = 0.0;

    for (unsigned int iii = 0; iii < scores.size(); iii++) {
        mean += scores.at(iii);
    }

    mean /= scores.size();

    float var = 0.0;

    for (unsigned int iii = 0; iii < scores.size(); iii++) {
        var += pow(abs(mean - scores.at(iii)), 2);
    }

    var /= scores.size();

    float stdev = pow(var, 0.5);

    summ[0] = mean;
    summ[1] = min((float)1.0, mean + stdev);
    summ[2] = max((float)0.0, mean - stdev);

}

int countAndSortFiles(char *input, vector<string> &inputList) {
    int fileCount = 0;

    DIR *dirp;
    struct dirent *entry;

    dirp = opendir(input);

    while ((entry = readdir(dirp)) != NULL) {

        if (entry->d_type == DT_REG) { // If the entry is a regular file
            inputList.push_back(string(entry->d_name));
            fileCount++;
        }
    }

    closedir(dirp);

    sort(inputList.begin(), inputList.end());

    return fileCount;
}


void initializeDetectors(vector< Ptr<FeatureDetector> > &allFeatureDetectors) {


    for (int ddd = 0; ddd < DD_DETECTORS_COUNT; ddd++) {

        Ptr<FeatureDetector> templateDetector;

        // DEFAULT_MAX_FEATURES
        if (DD_DETECTOR_NAMES[ddd] == "ORB") {
            templateDetector = new OrbFeatureDetector( 3*DEFAULT_MAX_FEATURES );
        } else if (DD_DETECTOR_NAMES[ddd] == "FAST") {
            templateDetector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("FAST"), DEFAULT_MAX_FEATURES, 3*DEFAULT_MAX_FEATURES, 100);
        } else if (DD_DETECTOR_NAMES[ddd] == "GFTT") {  // no response :(
            templateDetector = new GoodFeaturesToTrackDetector(1000, 0.01, 1, 3, false, 0.04);
        } else if (DD_DETECTOR_NAMES[ddd] == "HARRIS") { // no response :(
            templateDetector = new GoodFeaturesToTrackDetector(1000, 0.01, 1, 3, true, 0.04);
        } else if (DD_DETECTOR_NAMES[ddd] == "MSER") { // no response :(
            templateDetector = new MserFeatureDetector();
        } else if (DD_DETECTOR_NAMES[ddd] == "STAR") {
            templateDetector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("STAR"), DEFAULT_MAX_FEATURES, 3*DEFAULT_MAX_FEATURES, 100);
        } else if (DD_DETECTOR_NAMES[ddd] == "SIFT") { // no response :(
            templateDetector = new SiftFeatureDetector();
        } else if (DD_DETECTOR_NAMES[ddd] == "SURF") {
            templateDetector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SURF"), DEFAULT_MAX_FEATURES, 3*DEFAULT_MAX_FEATURES, 100);
        } else if (DD_DETECTOR_NAMES[ddd] == "SimpleBlob") {
            templateDetector = new SimpleBlobDetector();
        }

        //fixedDetector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SURF"), 2*maxFeatures, 10*maxFeatures, 100);

        allFeatureDetectors.push_back(templateDetector);
    }

}


void initializeDescriptors(vector< Ptr<DescriptorExtractor> > &allFeatureDescriptors) {

    //printf("%s << ENTERED.\n", __FUNCTION__);

    for (int ddd = 0; ddd < DD_DESCRIPTORS_COUNT; ddd++) {

        //printf("%s << [%d]\n", __FUNCTION__, ddd);

        Ptr<DescriptorExtractor> templateDescriptor;

        // DEFAULT_MAX_FEATURES
        if ((DD_DESCRIPTORS_NAMES[ddd] == "FREAK") || (DD_DESCRIPTORS_NAMES[ddd] == "SURF") || (DD_DESCRIPTORS_NAMES[ddd] == "SIFT") || (DD_DESCRIPTORS_NAMES[ddd] == "ORB") || (DD_DESCRIPTORS_NAMES[ddd] == "BRIEF")) {

            if (DD_DESCRIPTORS_NAMES[ddd] == "SURF") {
                cout << "Creating SURF descriptor extractor..." << endl;
                templateDescriptor = new SurfDescriptorExtractor(4, 2, false);
                //cin.get();
            } else {
                templateDescriptor = DescriptorExtractor::create((DD_DESCRIPTORS_NAMES[ddd]).c_str());
            }


        }

        allFeatureDescriptors.push_back(templateDescriptor);
    }

    //printf("%s << EXITING.\n", __FUNCTION__);
}


/*
void sortKeypoints(vector<KeyPoint>& keypoints, unsigned int maxKeypoints) {
    vector<KeyPoint> sortedKeypoints;

    if (keypoints.size() <= 1) {
        return;
    }

    // Add the first one
    sortedKeypoints.push_back(keypoints.at(0));

    for (unsigned int i = 1; i < keypoints.size(); i++) {

        unsigned int j = 0;
        bool hasBeenAdded = false;

        while ((j < sortedKeypoints.size()) && (!hasBeenAdded)) {

            if (abs(keypoints.at(i).response) > abs(sortedKeypoints.at(j).response)) {
                sortedKeypoints.insert(sortedKeypoints.begin() + j, keypoints.at(i));

                hasBeenAdded = true;
            }

            j++;
        }

        if (!hasBeenAdded) {
            sortedKeypoints.push_back(keypoints.at(i));
        }

    }



    keypoints.swap(sortedKeypoints);

    if (maxKeypoints < keypoints.size()) {
        keypoints.erase(keypoints.begin()+maxKeypoints, keypoints.end());
    }

    return;

}
*/

void obtainSubset(vector<KeyPoint>& src, vector<KeyPoint>& dst, double threshold) {

    if (src.size() == 0) {
        return;
    } else {
        int iii = 0;

        while ((abs(src.at(iii).response)) > threshold) {
            dst.push_back(src.at(iii));
            iii++;

            if (iii >= src.size()) {
                break;
            }
        }
    }

    return;

}

bool compareMatrices(Mat& mat1, Mat& mat2) {
    if (mat1.size() != mat2.size()) {
        //printf("%s << different sizes..", __FUNCTION__);
        return false;
    }

    if (mat1.depth() != mat2.depth()) {
        //printf("%s << different depths..", __FUNCTION__);
        return false;
    }

    if (mat1.channels() != mat2.channels()) {
        //printf("%s << different channels..", __FUNCTION__);
        return false;
    }

    unsigned long totalBytes;

    int depth;

    if ((mat1.depth() == CV_8U) || (mat1.depth() == CV_8S)) {
        depth = 1;
    } else if ((mat1.depth() == CV_16U) || (mat1.depth() == CV_16S)) {
        depth = 2;
    } else if ((mat1.depth() == CV_32S) || (mat1.depth() == CV_32F)) {
        depth = 4;
    } else if (mat1.depth() == CV_64F) {
        depth = 8;
    }

    unsigned char *ptr1, *ptr2;

    ptr1 = &(mat1.at<unsigned char>(0,0));
    ptr2 = &(mat2.at<unsigned char>(0,0));

    //printf("%s << [%d][%d][%d][%d]\n", __FUNCTION__, mat1.rows, mat1.cols, mat1.channels(), depth);

    totalBytes = mat1.rows * mat1.cols * mat1.channels() * depth;

    //printf("%s << totalBytes = %d\n", __FUNCTION__, totalBytes);

    for (unsigned int i = 0; i < totalBytes; i++) {
        if (ptr1[0] != ptr2[0]) {
            //printf("%s << returning false at byte [%d]", __FUNCTION__, i);
            return false;
        }

        ptr1++;
        ptr2++;
    }

    //printf("%s << matrices identical..\n", __FUNCTION__);

    return true;

}

/*
void trainedFASTDetection(Mat& image, vector<KeyPoint> &keypoints, int maxFeatures) {

    Mat greyIm;
    vector<KeyPoint> tmpKeypoints;

    if (image.channels() > 1) {
        cvtColor(image, greyIm, cv::COLOR_RGB2GRAY);
    } else {
        greyIm = Mat(image);
    }

    byte *ptrToData = greyIm.data;
    ImageRef imSize(image.cols, image.rows); // or other way around??
    int rowStride = image.cols;

    CVD::SubImage<byte> imageForFast(ptrToData, imSize, rowStride);
    vector<ImageRef> vectorForFast;

    int threshold = 0;

    vector<int> scores;

    int b = 0;

    int minScore = 10;

    faster0000_detect(imageForFast, vectorForFast, threshold);
    faster0000_score(imageForFast, vectorForFast, b, scores);

    //printf("%s << vectorForFast.size() = %d\n", __FUNCTION__, vectorForFast.size());

    // then convert ImageRef vector to the KeyPoint vector...

    KeyPoint newPoint;
    Point2f ptCoords;
    double xCoord, yCoord;
    double meanResponse = 0.0;

    for (unsigned int iii = 0; iii < vectorForFast.size(); iii++) {

        meanResponse += (((float) scores.at(iii)) / scores.size());
        //printf("%s << pt(%d) = (%d, %d) %f\n", __FUNCTION__, iii, vectorForFast.at(iii).x, vectorForFast.at(iii).y, (float) scores.at(iii));
        //if (iii < 2*maxFeatures) {

    }

    printf("%s << vectorForFast.size() = %d; meanResponse = %f\n", __FUNCTION__, vectorForFast.size(), meanResponse);

    if (vectorForFast.size() > 20*maxFeatures) {
        minScore = 2*meanResponse;
    } else if (vectorForFast.size() > 5*maxFeatures) {
        minScore = meanResponse;
    } else {
        minScore = 0;
    }


    for (unsigned int iii = 0; iii < vectorForFast.size(); iii++) {
        if (scores.at(iii) >= minScore) {

            xCoord = vectorForFast.at(iii).x;
            yCoord = vectorForFast.at(iii).y;

            //printf("%s << pt(%d) = (%f, %f) %f\n", __FUNCTION__, iii, xCoord, yCoord, (float) scores.at(iii));

            ptCoords = Point2f(xCoord, yCoord);
            newPoint = KeyPoint(ptCoords, 6, -1, (float) scores.at(iii));
            tmpKeypoints.push_back(newPoint);
        }
    }


    // Then try to discard the low scoring ones...
    sortKeypoints(tmpKeypoints);

    //printf("%s << maxFeatures = %d; tmpKeypoints.size() = %d\n", __FUNCTION__, maxFeatures, tmpKeypoints.size());

    if (maxFeatures >= tmpKeypoints.size()) {
        tmpKeypoints.assign(keypoints.begin(), keypoints.end());
    } else {
        double minResponse = tmpKeypoints.at(maxFeatures-1).response;
        obtainSubset(tmpKeypoints, keypoints, minResponse);
    }



    //printf("%s << keypoints.size() = %d\n", __FUNCTION__, keypoints.size());

    //printf("%s << DONE.\n", __FUNCTION__);

}
*/

void applyRandomDescription(vector<KeyPoint> &pts, Mat& descriptors) {

    printf("%s << Entered.\n", __FUNCTION__);

    srand ( (unsigned int)(time(NULL)) );
    waitKey(50);

    int dimensions = 16;

    descriptors = Mat(pts.size(), dimensions, CV_32FC1);

    for (int iii = 0; iii < pts.size(); iii++) {
        for (int jjj = 0; jjj < dimensions; jjj++) {
            float val = float(rand() % 1000) / 1.000;

            printf("%s << val = %f\n", __FUNCTION__, val);

            descriptors.at<float>(iii,jjj) = val;
        }
    }

}

void randomDetection(Mat& image, vector<KeyPoint> &keypoints, int maxFeatures, bool randomScales) {

    srand ( (unsigned int)(time(NULL)) );

    //srand (time(0));

    //printf("%s << random num = %d\n", __FUNCTION__, rand());
    //printf("%s << delaying a few seconds to help random clock...\n", __FUNCTION__);
    waitKey(1000);
    //sleep(500);

    double xCoord, yCoord;

    KeyPoint newPoint;
    Point2f ptCoords;


    if (randomScales == false) {
        for (int iii = 0; iii < 2*maxFeatures; iii++) {
            xCoord = rand() % image.cols;
            yCoord = rand() % image.rows;

            ptCoords = Point2f(xCoord, yCoord);
            newPoint = KeyPoint(ptCoords, 6, -1, iii+1);
            keypoints.push_back(newPoint);
        }
    } else {
        for (int iii = 0; iii < 2*maxFeatures; iii++) {
            xCoord = 4 + (rand() % (image.cols-8));
            yCoord = 4 + (rand() % (image.rows-8));

            ptCoords = Point2f(xCoord, yCoord);
            newPoint = KeyPoint(ptCoords, 128, -1, iii+1); // ranges from 12 - 128
            keypoints.push_back(newPoint);
        }
    }
}


void combineRecallVectors(vector<vector<Point2f> >& totalPrecRecall, vector<Point2f>& finalRecall) {

    //printf("%s << DEBUG %d\n", __FUNCTION__, 0);

    // For the moment, just use the first results
    //finalRecall.assign(totalPrecRecall.at(0).begin(), totalPrecRecall.at(0).end());
    finalRecall.clear();

    Point2f tmpPt;

    //printf("%s << DEBUG: totalPrecRecall.size() = %d\n", __FUNCTION__, totalPrecRecall.size());

    for (int iii = 0; iii < totalPrecRecall.at(0).size(); iii++) {

        tmpPt.x = totalPrecRecall.at(0).at(iii).x;
        tmpPt.y = 0.00;

        double tmpCount = 0.00;

        for (int jjj = 0; jjj < totalPrecRecall.size(); jjj++) {



            //printf("%s << DEBUG: totalPrecRecall.at(%d).size() = %d\n", __FUNCTION__, jjj, totalPrecRecall.at(jjj).size());
        //finalRecall.insert(finalRecall.end(), totalPrecRecall.at(iii).begin(), totalPrecRecall.at(iii).end());

            if (totalPrecRecall.at(jjj).size() > 0) {
                tmpCount = tmpCount + 1.00;
                tmpPt.y += totalPrecRecall.at(jjj).at(iii).y;
            }

        }

        tmpPt.y /= tmpCount;

        finalRecall.push_back(tmpPt);
    }

    //printf("%s << DEBUG %d\n", __FUNCTION__, 1);

    if (finalRecall.size() > 0) {
        sort(finalRecall.begin(), finalRecall.end(), sort_point2f_vector_by_x_val);
    }

    //printf("%s << DEBUG %d\n", __FUNCTION__, 2);

    return;

    int maxLimits = 100;

    finalRecall.resize(maxLimits + 1);

    // TODO get it to average the vectors somehow, by creating a new vector with the right size, and searching for
    // terms that match? Or some kind of quantisation to allow averaging to work more effectively?

    // find min precision, find max precision (from all of them)
    double minVal = 1.00, maxVal = 0.00;

    for (int iii = 0; iii < totalPrecRecall.size(); iii++) {
        for (int jjj = 0; jjj < totalPrecRecall.at(iii).size(); jjj++) {
            if (totalPrecRecall.at(iii).at(jjj).x > maxVal ) {
                maxVal = totalPrecRecall.at(iii).at(jjj).x;
            }
        }

        for (int jjj = 0; jjj < totalPrecRecall.at(iii).size(); jjj++) {
            if (totalPrecRecall.at(iii).at(jjj).x < minVal) {
                minVal = totalPrecRecall.at(iii).at(jjj).x;
            }
        }

    }

    printf("%s << minVal = %f; maxVal = %f\n", __FUNCTION__, minVal, maxVal);

    vector<double> limits;

    limits.push_back(maxVal);



    for (int aaa = 1; aaa < maxLimits; aaa++) {
        double newLimit = 0.00;

        for (int iii = 0; iii < totalPrecRecall.size(); iii++) {

            int index = int((double(aaa) / double(maxLimits)) * double(totalPrecRecall.at(iii).size()));

            //printf("%s << index = %d\n", __FUNCTION__, index);
            //

            newLimit += totalPrecRecall.at(iii).at(index).x;
        }

        newLimit /= totalPrecRecall.size();

        //printf("%s << newLimit = %f\n", __FUNCTION__, newLimit);

        limits.push_back(newLimit);

    }

    printf("%s << DEBUG %d\n", __FUNCTION__, 0);

    limits.push_back(minVal);

    // establish

    int recordedCount[256];

    for (int iii = 0; iii < totalPrecRecall.size(); iii++) {



        for (int jjj = 0; jjj < totalPrecRecall.at(iii).size(); jjj++) {
            int index = 0;

            while (totalPrecRecall.at(iii).at(jjj).x < limits.at(index)) {
                index++;
                printf("%s << index = %d\n", __FUNCTION__, index);

                printf("%s << nextVal = %f\n", __FUNCTION__, totalPrecRecall.at(iii).at(jjj).x);
                printf("%s << absLimit = %f\n", __FUNCTION__, limits.at(100));
            }

            finalRecall.at(index).y += totalPrecRecall.at(iii).at(jjj).y;
            recordedCount[index]++;

        }

    }

    for (int aaa = 0; aaa < limits.size(); aaa++) {
        finalRecall.at(aaa).y /= recordedCount[aaa];
    }




    // create new vec and fill in discretized precisions


}

bool sort_point2f_vector_by_x_val (const Point2f& pt1, const Point2f& pt2) {
    return (pt1.x < pt2.x);
}

bool sort_point2f_vector_by_y_val (const Point2f& pt1, const Point2f& pt2) {
    return (pt1.y < pt2.y);
}

void writeDescriptorResults(char* fileAddress, vector<Point2f>& curve) {
    FILE *file = fopen (fileAddress, "w");

    for (int zzz = 0; zzz < curve.size(); zzz++)
    {
        //printf("%f, %f\n", curve.at(zzz).x, curve.at(zzz).y);
        fprintf( file, "%f, %f\n", curve.at(zzz).x, curve.at(zzz).y);
    }

    fclose( file );
}

void convertBackToMatchVec(Mat& matchingMat, vector<vector<DMatch> >& matches) {

    Mat matchingMatCopy;
    matchingMat.copyTo(matchingMatCopy);

    // Need to go through the entire matchingMatCopy for each row and order the cells

    for (unsigned int iii = 0; iii < matchingMat.rows; iii++) {

        vector<DMatch> currentMatches;

        while (currentMatches.size() < matchingMat.cols) {

            double bestMatch = 9e99;
            unsigned int bestIndex = 0;

            for (unsigned int jjj = 0; jjj < matchingMat.cols; jjj++) {

                //printf("%s << (%d) vs (%d, %d) [%d]\n", __FUNCTION__, iii, jjj);

                if ((matchingMatCopy.at<double>(iii,jjj) < bestMatch) && (matchingMatCopy.at<double>(iii,jjj) >= 0.0)) {
                    bestMatch = matchingMatCopy.at<double>(iii,jjj);
                    bestIndex = jjj;
                }

            }

            DMatch currMatch(iii,bestIndex,0,bestMatch);

            currentMatches.push_back(currMatch);
            matchingMatCopy.at<double>(iii,bestIndex) = -1.0;

        }

        matches.push_back(currentMatches);

    }

}

double calculateEntropy(const Mat& im1, const Mat& im2, const vector<KeyPoint>& pts1, const vector<KeyPoint>& pts2) {

    vector<KeyPoint> pts1a, pts2a;

    pts1a.insert(pts1a.end(), pts1.begin(), pts1.end());
    pts2a.insert(pts2a.end(), pts2.begin(), pts2.end());

    //printf("%s << Assigning responses...\n", __FUNCTION__);

    assignEigenVectorResponses(im1, pts1a);
    assignEigenVectorResponses(im2, pts2a);

    double entropy = 0.00;

    for (unsigned int iii = 0; iii < pts1a.size(); iii++) {
        entropy += pts1a.at(iii).response / ((double) pts1a.size());
    }

    for (unsigned int iii = 0; iii < pts2a.size(); iii++) {
        entropy += pts2a.at(iii).response / ((double) pts2a.size());
    }

    return entropy;

}

void combineMatches(vector<vector<DMatch> >& matches1, vector<vector<DMatch> >& matches2, vector<vector<DMatch> >& combinedMatches, double weighting, double entropy) {

    int indexQuery;
    int indexTrain;
    double distance;

    double primaryWeighting = 1.00;

    if (matches1.size() != matches2.size()) {
        printf("%s << ERROR! match vector lengths don't correspond... (%d vs %d)\n", __FUNCTION__, matches1.size(), matches2.size());
    }

    Mat matchingMatrix1, matchingMatrix2, fullMatchingMatrix;
    createMatchingMatrix(matchingMatrix1, matches1);
    createMatchingMatrix(matchingMatrix2, matches2);

    // combine Matrices

    if (weighting == THERMAL_ONLY) {
        primaryWeighting = 0.00;
        weighting = 1.00;
    }

    static int environmentCounter = -1;

    environmentCounter++;

    int currentIndex = environmentCounter / 20;

    printf("%s << currentIndex = %d\n", __FUNCTION__, currentIndex);
    //cin.get();

    fullMatchingMatrix = Mat::zeros(matchingMatrix1.size(), CV_64FC1);

    // create flag matrix

    Mat flagMatrix = Mat::zeros(fullMatchingMatrix.size(), CV_8UC1);

    double xi, yi, d;
    double min, max;

    double gradient;
    double weight;

    bool firstValProcessed = false;

    Mat lookupMat;
    double maxVal;

    if (weighting == DO_LINEAR_WEIGHTING) {
        gradient = linear_grads[currentIndex];
    } else if (weighting == DO_RECIPROCAL_WEIGHTING) {
        gradient = inverse_grads[currentIndex];
    } else if (weighting == DOUBLE_RECIPROCAL) {
        gradient = double_inverse_grads[currentIndex];
    } else if (weighting == LOOKUP_WEIGHTING) {

        char filename[] = "/home/steve/svidaslib/sparse-coding/raw_vals.txt";


        lookupMat = Mat::zeros(501, 501, CV_64FC1);

        ifstream filestr(filename);

        float inVal = 0.0;

        filestr >> maxVal;

        for (unsigned int iii = 0; iii <= 500; iii++) {
            for (unsigned int jjj = 0; jjj <= 500; jjj++) {

                filestr >> inVal;

                lookupMat.at<double>(iii,jjj) = inVal;

            }
        }

        filestr.close();
    }


    for (unsigned int iii = 0; iii < matchingMatrix1.rows; iii++) {
        for (unsigned int jjj = 0; jjj < matchingMatrix1.rows; jjj++) {

            xi = matchingMatrix1.at<double>(iii,jjj);
            yi = matchingMatrix2.at<double>(iii,jjj);

            if ((xi != 0.0) && (yi != 0.0)) {

                flagMatrix.at<unsigned char>(iii,jjj) = 1;

                if (weighting == MIN_WEIGHTING_CODE) {
                    d = std::min(matchingMatrix1.at<double>(iii,jjj), matchingMatrix2.at<double>(iii,jjj));
                } else if (weighting == MAX_WEIGHTING_CODE) {
                    d = std::max(matchingMatrix1.at<double>(iii,jjj), matchingMatrix2.at<double>(iii,jjj));
                } else if (weighting == RADIAL_DISTANCE) {

                    double center = 0.80;
                    double d1, d2;

                    d1 = pow(xi - center, 2.0);
                    d2 = pow(yi - center, 2.0);

                    if ((d1 + d2) != 0.0) {
                        // set flag as zero
                        flagMatrix.at<unsigned char>(iii,jjj) = 0;
                    } else {
                        d = 1 / pow(d1 + d2, 0.5);
                    }


                } else if (weighting == STAGGERED_WEIGHTING) {

                    double limit = ((double) floor(weighting))/100.0;
                    weighting = weighting - floor(weighting);

                    if (xi < limit) {
                        d = xi;
                    } else {
                        d = ((primaryWeighting * xi) + (weighting * yi)) / (primaryWeighting + weighting);
                    }

                } else if (weighting == MULTIPLY_MODALITIES) {
                    d = xi * yi;
                } else if (weighting == DO_LINEAR_WEIGHTING) {

                    d = xi - (1.0 / gradient) * yi;

                } else if (weighting == DO_RECIPROCAL_WEIGHTING) {
                    d = xi - (1.0 / gradient) * (1.0 / yi);
                } else if (weighting == DOUBLE_RECIPROCAL) {
                    d = 1.0 / ((1.0 / xi) - (1.0 / gradient) * (1.0 / yi));
                } else if (weighting == DO_DEPENDENT_WEIGHTING) {
                    d = xi - (1.0 / -entropy) * yi;
                } else if (weighting == LOOKUP_WEIGHTING) {

                    //printf("%s << lookup starting...\n", __FUNCTION__);
                    d = lookupValue(xi, yi, maxVal, lookupMat);
                    //printf("%s << lookup finished.\n", __FUNCTION__);




                    //double vx = (xi / maxVal) * 500;
                    //double vy = (yi / maxVal) * 500;

                    //int index_1 = std::max(std::min(((int) vy), 500), 0);
                    //int index_2 = std::max(std::min(((int) vx), 500), 0);

                    //d = lookupMat.at<double>(index_1, index_2);

                    if ((xi == 1.00) || (xi == 0.00) || (yi == 0.00)) { // || (yi == 1.00)
                        //printf("%s << distances: [%d, %d] (%f, %f) -> (%f)\n", __FUNCTION__, iii, jjj, xi, yi, d);
                    }

                    //printf("%s << distances: [%d, %d] (%f, %f) -> (%f)\n", __FUNCTION__, iii, jjj, xi, yi, d);


                } else {
                    // Linear weighting

                    d = primaryWeighting * xi + weighting * yi;



                }

                if (yi == 1.00) {
                    d = 10.0 + xi;
                }


                if (flagMatrix.at<unsigned char>(iii,jjj) == 1) {

                    fullMatchingMatrix.at<double>(iii,jjj) = d;



                    if (!firstValProcessed) {
                        min = d;
                        max = d;
                        firstValProcessed = true;
                    } else {
                        if (d < min) {
                            min = d;
                        }

                        if (d > max) {
                            max = d;
                        }
                    }



                }


            }

        }
    }

    //cin.get();

    for (unsigned int iii = 0; iii < matchingMatrix1.rows; iii++) {
        for (unsigned int jjj = 0; jjj < matchingMatrix1.rows; jjj++) {

            if (flagMatrix.at<unsigned char>(iii,jjj) == 1) {
                fullMatchingMatrix.at<double>(iii,jjj) = fullMatchingMatrix.at<double>(iii,jjj) - min + 0.001;
            } else {
                fullMatchingMatrix.at<double>(iii,jjj) = max - min + 0.002;
            }

            //printf("%s << min = %f; max = %f\n", __FUNCTION__, min, max);
            //printf("%s << (%d, %d) -> (%f)\n", __FUNCTION__, iii, jjj, d);

        }
    }

    //cin.get();

    convertBackToMatchVec(fullMatchingMatrix, combinedMatches);

}

void calculateFusionPR( Mat& im1, Mat& im2, Mat& im1b, Mat& im2b, Mat& homography, vector<KeyPoint>& pts1, vector<KeyPoint>& pts2, Mat& desc1, Mat& desc2, Mat& desc1b, Mat& desc2b, vector<Point2f>& results, int correspCount, float overlapThreshold, double weighting, double entropy) {

    Ptr<DescriptorMatcher> dmatcher;

    // Based on the descriptor matrix types, may use a different matcher:
    if (desc1.type() == CV_32FC1) {
        dmatcher = DescriptorMatcher::create("BruteForce");
    } else if (desc1.type() == CV_64FC1) {
        dmatcher = DescriptorMatcher::create("BruteForce");
    } else if (desc1.type() == CV_8UC1) {
        dmatcher = DescriptorMatcher::create("BruteForce-Hamming");
    }

    vector<vector<DMatch> > matches1to2, matches2to1, matches1to2b, matches2to1b;

    vector<vector<uchar> > *correctMatches1to2Mask, buf2;
    correctMatches1to2Mask = &buf2;

    vector<vector<uchar> > *correctMatches2to1Mask, buf2b;
    correctMatches2to1Mask = &buf2b;

    if( pts1.empty() )
        CV_Error( CV_StsBadArg, "keypoints1 must be not empty" );

    bool computeKeypoints2ByPrj = pts2.empty();

    if( computeKeypoints2ByPrj ) {
        assert(0);
    }

    printf("%s << desc1.rows = %d; desc2.rows = %d\n", __FUNCTION__, desc1.rows, desc2.rows);
    printf("%s << desc1b.rows = %d; desc2b.rows = %d\n", __FUNCTION__, desc1b.rows, desc2b.rows);

    dmatcher->knnMatch( desc1, desc2, matches1to2, desc2.rows );
    dmatcher->knnMatch( desc2, desc1, matches2to1, desc1.rows );

    dmatcher->knnMatch( desc1b, desc2b, matches1to2b, desc2b.rows );
    dmatcher->knnMatch( desc2b, desc1b, matches2to1b, desc1b.rows );

    // Now need to fuse these together...
    vector<vector<DMatch> > fusedMatches1to2, fusedMatches2to1;

    Mat visibleMatches, thermalMatches;
    createMatchingMatrix(visibleMatches, matches1to2, matches2to1);
    createMatchingMatrix(thermalMatches, matches1to2b, matches2to1b);

    printf("%s << matches1to2.size() = %d; matches1to2b.size() = %d\n", __FUNCTION__, matches1to2.size(), matches1to2b.size());
    printf("%s << matches2to1.size() = %d; matches2to1b.size() = %d\n", __FUNCTION__, matches2to1.size(), matches2to1b.size());

    combineMatches(matches1to2, matches1to2b, fusedMatches1to2, weighting, entropy);
    combineMatches(matches2to1, matches2to1b, fusedMatches2to1, weighting, entropy);


    printf("%s << fusedMatches1to2.size() = %d; fusedMatches2to1.size() = %d\n", __FUNCTION__, fusedMatches1to2.size(), fusedMatches2to1.size());

    // Figure out "true" matches using homography
    float repeatability;
    int reducedCorrespCount;
    Mat thresholdedOverlapMask; // thresholded allOverlapErrors
    calculateRepeatability( im1, im2, homography, pts1, pts2, repeatability, reducedCorrespCount, overlapThreshold, &thresholdedOverlapMask );




    correctMatches1to2Mask->resize(fusedMatches1to2.size());
    for( size_t i = 0; i < fusedMatches1to2.size(); i++ )
    {
        (*correctMatches1to2Mask)[i].resize(fusedMatches1to2[i].size());
        for( size_t j = 0;j < fusedMatches1to2[i].size(); j++ )
        {
            int indexQuery = fusedMatches1to2[i][j].queryIdx;
            int indexTrain = fusedMatches1to2[i][j].trainIdx;
            (*correctMatches1to2Mask)[i][j] = thresholdedOverlapMask.at<uchar>( indexQuery, indexTrain );
        }
    }

    printf("%s << DEBUG %d\n", __FUNCTION__, 3);

    // For 2-way results
    correctMatches2to1Mask->resize(fusedMatches2to1.size());
    for( size_t i = 0; i < fusedMatches2to1.size(); i++ )
    {
        (*correctMatches2to1Mask)[i].resize(fusedMatches2to1[i].size());
        for( size_t j = 0;j < fusedMatches2to1[i].size(); j++ )
        {
            int indexQuery = fusedMatches2to1[i][j].queryIdx;
            int indexTrain = fusedMatches2to1[i][j].trainIdx;
            (*correctMatches2to1Mask)[i][j] = thresholdedOverlapMask.at<uchar>( indexTrain, indexQuery );
        }
    }

    analyzeMatches(visibleMatches, thermalMatches, thresholdedOverlapMask);

    //int maximumValue = max(correspCount, reducedCorrespCount);
    int maximumValue = min(pts1.size(), pts2.size());

    printf("%s << Computing recall-precision curve (div by %d)\n", __FUNCTION__, maximumValue);
    computeRecallPrecisionCurveModified( fusedMatches1to2, *correctMatches1to2Mask, fusedMatches2to1, *correctMatches2to1Mask, results, maximumValue );
}

void analyzeMatches(Mat& visMat, Mat& thermMat, Mat& mask) {

    //return;

    printf("%s << (%d to %d) : (%d, %d)\n", __FUNCTION__, visMat.rows, visMat.cols, mask.rows, mask.cols);
    //cin.get();

    ofstream myfile;

    char filename[256];

    sprintf(filename, "%s-%d.csv", "/home/steve/sparse-coding/test-file-", ::globalCounter, ".csv");
    ::globalCounter++;
    myfile.open(filename);

    //myfile << "[";

    for (unsigned int iii = 0; iii < visMat.rows; iii++) {
        for (unsigned int jjj = 0; jjj < visMat.cols; jjj++) {
            //printf("%s << (%d, %d) : V = %f; T = %f; M = %d\n", __FUNCTION__, iii, jjj, visMat.at<double>(iii,jjj), thermMat.at<double>(iii,jjj), mask.at<uchar>(iii,jjj));


            myfile << visMat.at<double>(iii,jjj) << "," << thermMat.at<double>(iii,jjj) << ",";



            if (mask.at<uchar>(iii,jjj) != 0) {
                myfile << 1 << endl;
            } else {
                myfile << 0 << endl;
            }
        }
    }

   // myfile << "]";

    myfile.close();
    //cin.get();

}

void calculatePrecisionRecall( Mat& im1, Mat& im2, Mat& homography, vector<KeyPoint>& pts1, vector<KeyPoint>& pts2, Mat& desc1, Mat& desc2, vector<Point2f>& results, int correspCount, float overlapThreshold) {

    Ptr<DescriptorMatcher> dmatcher;

    //printf("%s << desc1.size() = (%d, %d)\n", __FUNCTION__, desc1.rows, desc1.cols);

    //cout << "desc1 = \n" << desc1 << endl;

    // Based on the descriptor matrix types, may use a different matcher:
    if (desc1.type() == CV_32FC1) {
        //printf("%s << Using BruteForce matcher (CV_32FC1).\n", __FUNCTION__);
        //dmatcher = GenericDescriptorMatcher::create("BruteForce");
        dmatcher = DescriptorMatcher::create("BruteForce");
        //dmatcher = defaultDescMatcher;
        //Ptr<GenericDescriptorMatcher> descMatch = commRunParams[di].isActiveParams ? specificDescMatcher : defaultDescMatcher;
        // defaultDescMatcher = GenericDescriptorMatcher::create( algName );
    } else if (desc1.type() == CV_64FC1) {
        //printf("%s << Using BruteForce matcher (CV_64FC1).\n", __FUNCTION__);
        dmatcher = DescriptorMatcher::create("BruteForce");
    } else if (desc1.type() == CV_8UC1) {
        //printf("%s << Using BruteForce-Hamming matcher (CV_8UC1).\n", __FUNCTION__);
        dmatcher = DescriptorMatcher::create("BruteForce-Hamming");
    } else {
        //printf("%s << Unexpected descriptor data type...\n", __FUNCTION__);
    }

    printf("%s << DEBUG X\n", __FUNCTION__);

    // dmatcher->clear();

    //vector<vector<DMatch> > *matches1to2, buf1;
    //matches1to2 = &buf1;

    vector<vector<DMatch> > matches1to2, matches2to1;

    vector<vector<uchar> > *correctMatches1to2Mask, buf2;
    correctMatches1to2Mask = &buf2;



    vector<vector<uchar> > *correctMatches2to1Mask, buf2b;
    correctMatches2to1Mask = &buf2b;

    if( pts1.empty() )
        CV_Error( CV_StsBadArg, "keypoints1 must be not empty" );


    // How do I get around this error? Have I made a mistake in how I initialized the GenericDescriptorMatcher?
    /*
    if( matches1to2->empty() && dmatcher.empty() )
        CV_Error( CV_StsBadArg, "dmatch must be no empty when matches1to2 is empty" );
    */

    //dmatcher->clear();

    //if( dmatcher.empty() )
        //printf("%s << dmatcher is empty...\n", __FUNCTION__);

    bool computeKeypoints2ByPrj = pts2.empty();

    if( computeKeypoints2ByPrj )
    {
        assert(0);
        // TODO: add computing pts2 from pts1 using H1to2
    }

    /*
    imshow("imx", im1);
    waitKey( );
    imshow("imx", im2);
    waitKey( );
    */

    //printf("%s << pts1.size() = %d\n", __FUNCTION__, pts1.size());
    //printf("%s << pts2.size() = %d\n", __FUNCTION__, pts2.size());

    /*
    if( matches1to2->empty() || computeKeypoints2ByPrj )
    {
        dmatcher->clear();
        dmatcher->radiusMatch( im1, pts1, im2, pts2, *matches1to2, std::numeric_limits<float>::max() );
    }
    */


    printf("%s << DEBUG %d (%dx%d) (%dx%d) (%d)\n", __FUNCTION__, 0, desc1.rows, desc1.cols, desc2.rows, desc2.cols, matches1to2.size());

    //dmatcher->clear();
    dmatcher->knnMatch( desc1, desc2, matches1to2, desc2.rows );

    // Match other way if possible
    dmatcher->knnMatch( desc2, desc1, matches2to1, desc1.rows );

    //dmatcher->radiusMatch( desc1, desc2, matches1to2, std::numeric_limits<float>::max() );

    // Should create matches1to2 using just a Flann matcher or something..?

    printf("%s << DEBUG %d\n", __FUNCTION__, 1);

    printf("%s << pts1.size() = %d; pts2.size() = %d\n", __FUNCTION__, pts1.size(), pts2.size());

    // Figure out "true" matches using homography
    float repeatability;
    int reducedCorrespCount;
    Mat thresholdedOverlapMask; // thresholded allOverlapErrors
    calculateRepeatability( im1, im2, homography, pts1, pts2, repeatability, reducedCorrespCount, overlapThreshold, &thresholdedOverlapMask );

    //printf("%s << repeatability = %f; correspCount = %d\n", __FUNCTION__, repeatability, correspCount);
    //cin.get();

    // Count feasible matches...

    /*
    printf("%s << pts1.size() = %d; pts2.size() = %d\n", __FUNCTION__, pts1.size(), pts2.size());
    printf("%s << thresholdedOverlapMask.size() = (%d, %d)\n", __FUNCTION__, thresholdedOverlapMask.rows, thresholdedOverlapMask.cols);
    printf("%s << correspCount = %d\n", __FUNCTION__, correspCount);
    */

    //cin.get();

    /*
    for (int iii = 0; iii < thresholdedOverlapMask.rows; iii++) {
        for (int jjj = 0; jjj < thresholdedOverlapMask.cols; jjj++) {

            if (int(thresholdedOverlapMask.at<uchar>(iii,jjj)) == 1) {
                printf("%s << match between (%d) and (%d)\n", __FUNCTION__, iii, jjj);
            }
            //printf("%s << thresholdedOverlapMask.at<uchar>(iii,jjj) = %d\n", __FUNCTION__, thresholdedOverlapMask.at<uchar>(iii,jjj));
        }




    }
    */

    //cin.get();


    printf("%s << DEBUG %d\n", __FUNCTION__, 2);

    correctMatches1to2Mask->resize(matches1to2.size());
    for( size_t i = 0; i < matches1to2.size(); i++ )
    {
        (*correctMatches1to2Mask)[i].resize(matches1to2[i].size());
        for( size_t j = 0;j < matches1to2[i].size(); j++ )
        {
            int indexQuery = matches1to2[i][j].queryIdx;
            int indexTrain = matches1to2[i][j].trainIdx;
            (*correctMatches1to2Mask)[i][j] = thresholdedOverlapMask.at<uchar>( indexQuery, indexTrain );
        }
    }

    printf("%s << DEBUG %d\n", __FUNCTION__, 3);

    // For 2-way results
    correctMatches2to1Mask->resize(matches2to1.size());
    for( size_t i = 0; i < matches2to1.size(); i++ )
    {
        (*correctMatches2to1Mask)[i].resize(matches2to1[i].size());
        for( size_t j = 0;j < matches2to1[i].size(); j++ )
        {
            int indexQuery = matches2to1[i][j].queryIdx;
            int indexTrain = matches2to1[i][j].trainIdx;
            (*correctMatches2to1Mask)[i][j] = thresholdedOverlapMask.at<uchar>( indexTrain, indexQuery );
        }
    }

    //int maximumValue = max(correspCount, reducedCorrespCount);
    int maximumValue = min(pts1.size(), pts2.size());

    printf("%s << Computing recall-precision curve (div by %d) [%d, %d]\n", __FUNCTION__, maximumValue, matches1to2.size(), matches2to1.size());
    computeRecallPrecisionCurveModified( matches1to2, *correctMatches1to2Mask, matches2to1, *correctMatches2to1Mask, results, maximumValue );
    printf("%s << Finished.\n", __FUNCTION__);
}

void configData::printParameters() {

    printf("%s << Regenerating: ", __FUNCTION__);

    if (regenerateAllResults) {
        printf("(all results)\n");
    }  else {

        if (regenerateHomographies) {
            printf("(homographies) ");
        }

        if (regenerateFeatures) {
            printf("(features) ");
        }

        if (regenerateDetectorResults) {
            printf("(detector results) ");
        }

        if (regenerateDescriptors) {
            printf("(descriptors) ");
        }

        if (regenerateDescriptorResults) {
            printf("(descriptor results) ");
        }

        printf("\n");
    }



    if (testAcrossModalities) {
        printf("%s << Testing across modalities...\n", __FUNCTION__);
    } else {
        printf("%s << Testing within modalities only...\n", __FUNCTION__);
    }

    if (displayMode) {
        printf("%s << Option to display has been selected...\n", __FUNCTION__);
    } else {
        printf("%s << Option to display has been deselected...\n", __FUNCTION__);
    }

    if (writeMode) {
        printf("%s << Option to write images has been selected...\n", __FUNCTION__);
    } else {
        printf("%s << Option to write images has been deselected...\n", __FUNCTION__);
    }

}

configData::configData() {


    singleDatasetOnly = false;
    singleModalityOnly = false;
    singleSubsetOnly = false;

    regenerateHomographies = false;
    testAcrossModalities = false;
    displayMode = false;
    writeMode = false;
    regenerateDetectorResults = false;
    regenerateDescriptorResults = false;
    regenerateFeatures = false;
    regenerateDescriptors = false;
    regenerateAllResults = false;

    singleTypeOnly = false;
    typeCode = -1;

    fusionMode = false;
}

void fileTree::prepareTree(configData& iD) {
    prepVector(datasets, iD.datasetString, iD.singleDatasetOnly, DD_DATASET_NAMES, DD_DATASETS_COUNT);
    prepVector(modalities, iD.modalityString, iD.singleModalityOnly, DD_MODALITY_NAMES, DD_MODALITIES_COUNT);
    prepVector(subsets, iD.subsetString, iD.singleSubsetOnly, DD_SUBSET_NAMES, DD_SUBSETS_COUNT);
}

void fileTree::displayTree() {
    listTestSets("datasets", datasets);
    listTestSets("modalities", modalities);
    listTestSets("subsets", subsets);
}

void initializeSubsetStrings() {
    //DD_SUBSET_NAMES[0] = "profile";

    for (int iii = 0; iii < DD_DIGITAL_TRANSFORMATIONS_COUNT; iii++) {
        //DD_SUBSET_NAMES[1 + iii] = DD_DIGITAL_TRANSFORMATIONS[iii];
    }

    for (int jjj = 0; jjj < DD_ANALOG_TRANSFORMATIONS_COUNT; jjj++) {
        //DD_SUBSET_NAMES[1 + DD_DIGITAL_TRANSFORMATIONS_COUNT + jjj] = DD_ANALOG_TRANSFORMATIONS[jjj];
    }
}

void prepVector(vector<string>& stringVector, char* inputString, bool singleOnly, const string *nameStrings, const int stringCount) {

    if (singleOnly) {
        //printf("%s << HERE.\n", __FUNCTION__);
        string input2str(inputString);
        //printf("%s << AND.\n", __FUNCTION__);
        stringVector.push_back(input2str);
        //printf("%s << THERE.\n", __FUNCTION__);
    } else {
        for (int iii = 0; iii < stringCount; iii++) {
            stringVector.push_back(nameStrings[iii]);
        }
    }

    //printf("%s << Exiting...\n", __FUNCTION__);
}

void listTestSets(string debugString, vector<string> setList) {
    /*
    printf("%s << debugString = %s\n", __FUNCTION__, debugString.c_str());
    printf("%s << setList.size() = %d\n", __FUNCTION__, setList.size());
    */

    printf("%s << List of %s:\n", __FUNCTION__, (debugString).c_str());

    for (unsigned int iii = 0; iii < setList.size(); iii++) {
        printf("\t%s\n", (setList.at(iii)).c_str());
    }
}

/*
double getInterpolatedVal(Mat& img, Point2f& coord) {

	double retVal = 0.00;



	// Find four neighbours
	Point dcoord[4];
	double val[4];
	double dist[4], total_dist;

	//printf("%s << coord = (%f, %f)\n", __FUNCTION__, coord.x, coord.y);

	// #1:
	dcoord[0] = Point(floor(coord.x), floor(coord.y));
	val[0] = img.at<unsigned char>(dcoord[0].y, dcoord[0].x);
	dist[0] = pow(pow(coord.x - ((double) dcoord[0].x), 2.0) + pow(coord.y - ((double) dcoord[0].y), 2.0), 0.5);
	total_dist += dist[0];
	//printf("%s << coord[0] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[0].x, dcoord[0].y, val[0], dist[0]);

	// #2:
	dcoord[1] = Point(ceil(coord.x), floor(coord.y));
	val[1] = img.at<unsigned char>(dcoord[1].y, dcoord[1].x);
	dist[1] = pow(pow(coord.x - ((double) dcoord[1].x), 2.0) + pow(coord.y - ((double) dcoord[1].y), 2.0), 0.5);
	total_dist += dist[1];
	//printf("%s << coord[1] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[1].x, dcoord[1].y, val[1], dist[1]);

	// #3:
	dcoord[2] = Point(ceil(coord.x), ceil(coord.y));
	val[2] = img.at<unsigned char>(dcoord[2].y, dcoord[2].x);
	dist[2] = pow(pow(coord.x - ((double) dcoord[2].x), 2.0) + pow(coord.y - ((double) dcoord[2].y), 2.0), 0.5);
	total_dist += dist[2];
	//printf("%s << coord[2] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[2].x, dcoord[2].y, val[2], dist[2]);

	// #4:
	dcoord[3] = Point(floor(coord.x), ceil(coord.y));
	val[3] = img.at<unsigned char>(dcoord[3].y, dcoord[3].x);
	dist[3] = pow(pow(coord.x - ((double) dcoord[3].x), 2.0) + pow(coord.y - ((double) dcoord[3].y), 2.0), 0.5);
	total_dist += dist[3];
	//printf("%s << coord[3] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[3].x, dcoord[3].y, val[3], dist[3]);

	//printf("%s << (%f, %f, %f, %f) : (%f, %f, %f, %f) : (%f)\n", __FUNCTION__, val[0], val[1], val[2], val[3], dist[0], dist[1], dist[2], dist[3], total_dist);

	Point ref_coord = Point(floor(coord.x), floor(coord.y));

	if (total_dist == 0.0) {
		retVal = val[0];
		//printf("%s << (%f, %f, %f, %f) : (%f, %f) vs (%d, %d) : (%f)\n", __FUNCTION__, val[0], val[1], val[2], val[3], coord.x, coord.y, ref_coord.x, ref_coord.y, retVal);
		return retVal;
	}



	Mat x_mat(1, 2, CV_64FC1);
	x_mat.at<double>(0,0) = 1 - abs(coord.x - ((double) ref_coord.x));
	x_mat.at<double>(0,1) = abs(coord.x - ((double) ref_coord.x));

	Mat y_mat(2, 1, CV_64FC1);
	y_mat.at<double>(0,0) = 1 - abs(coord.y - ((double) ref_coord.y));
	y_mat.at<double>(1,0) = abs(coord.y - ((double) ref_coord.y));

	Mat f_vals(2, 2, CV_64FC1);
	f_vals.at<double>(0,0) = val[0];
	f_vals.at<double>(0,1) = val[3];
	f_vals.at<double>(1,0) = val[1];
	f_vals.at<double>(1,1) = val[2];

	Mat A = x_mat * f_vals * y_mat;

	retVal = A.at<double>(0,0);
	//printf("%s << (%f, %f, %f, %f) : (%f, %f) vs (%d, %d) : (%f)\n", __FUNCTION__, val[0], val[1], val[2], val[3], coord.x, coord.y, ref_coord.x, ref_coord.y, retVal);

	return retVal;

	total_dist = 4 - total_dist;

	// Then add weightings
}
*/

/*
bool constructPatch(Mat& img, Mat& patch, Point2f& center, double radius, int cells) {

    //printf("%s << ENTERED.\n", __FUNCTION__);

	patch.release();

    if ((cells % 2) == 0) {
        cells++;
    }

    patch = Mat::zeros(cells, cells, CV_64FC1);

    int range = (cells-1)/2;

	for (int iii = -range; iii <= +range; iii++) {
		for (int jjj = -range; jjj <= +range; jjj++) {

			Point2f currPt = Point2f(center.x + ((double) iii)*radius, center.y + ((double) jjj)*radius);

			//printf("%s << currPt = (%f, %f)\n", __FUNCTION__, currPt.x, currPt.y);

			if ((currPt.x <= 0.0) || (currPt.x >= ((double) img.cols)) || (currPt.y >= ((double) img.rows)) || (currPt.y <= 0.0)) {
				return false;
			}

            //printf("%s << Extracting (%f, %f) of (%d, %d)\n", __FUNCTION__, currPt.x, currPt.y, img.cols, img.rows);
			double val = getInterpolatedVal(img, currPt);
			//printf("%s << Assigning (%d, %d) of (%d, %d)\n", __FUNCTION__, iii+range, jjj+range, cells, cells);
			patch.at<double>(iii+range,jjj+range) = val;

		}
	}

	//printf("%s << EXITING.\n", __FUNCTION__);

	return true;

}
*/

/*
double estimateStability(Mat& img, Point2f& center, double radius) {
    double stability = 0.0;

    if (((center.x - radius) < 0) || ((center.y - radius) < 0) || ((center.x + radius) >= img.cols) || ((center.y + radius) >= img.rows)) {
		//printf("%s << Returning, error.\n", __FUNCTION__);
		return stability;
	}

    Mat patch;
	constructPatch(img, patch, center, radius, 15);

    double minVal, maxVal;
    minMaxLoc(patch, &minVal, &maxVal);

	double variance = getPatchVariance(patch);

    Mat largerMat;
    //patch /= maxVal;
    //resize(patch, largerMat, Size(patch.rows*4, patch.cols*4), 0, 0, INTER_NEAREST);

    // largerMat /=


	//imshow("patch", largerMat);
	//waitKey();

	//printf("%s << variance = %f\n", __FUNCTION__, variance);

	stability = 1 / variance;

	return stability;

}
*/

/*
double estimateSalience(Mat& img, Point2f& center, double radius) {

	// Assumes 8-bit image
	double salience = 0.0;



	// Find center
	//cv::Point center = Point(((int) kp.pt.x), ((int) kp.pt.y));

	// Find radius


	if (((center.x - radius) < 0) || ((center.y - radius) < 0) || ((center.x + radius) >= img.cols) || ((center.y + radius) >= img.rows)) {
		return salience;
	}

	Mat patch;
	int patchSize = radius * 2;
	if ((patchSize % 2) == 0) {
        //patchSize++;
	}

	constructPatch(img, patch, center, radius, patchSize);
	//cout << patch << endl;

	salience = getValueFromPatch(patch);


	return salience;

}
*/

/*
double getPatchVariance(const Mat& patch) {
    double mean = 0.0;
	double sigma = 0.0;

	for (unsigned int iii = 0; iii < patch.rows; iii++) {
		for (unsigned int jjj = 0; jjj < patch.cols; jjj++) {
			mean += patch.at<double>(iii,jjj) / ((double) (patch.rows * patch.cols));
		}
	}

	for (unsigned int iii = 0; iii < patch.rows; iii++) {
		for (unsigned int jjj = 0; jjj < patch.cols; jjj++) {
			//printf("%s << (%f vs %f)\n", __FUNCTION__, patch.at<double>(iii,jjj), mean);
			sigma += pow((patch.at<double>(iii,jjj) - mean), 2.0) / ((double) (patch.rows * patch.cols));
		}
	}

	sigma = pow(sigma, 0.5);

	return sigma;
}
*/

/*
double getValueFromPatch(Mat& patch) {

    Mat convertedPatch = Mat::zeros(patch.size(), CV_32FC1);

	for (unsigned int iii = 0; iii < patch.rows; iii++) {
		for (unsigned int jjj = 0; jjj < patch.cols; jjj++) {
			convertedPatch.at<float>(iii,jjj) = ((float) patch.at<double>(iii,jjj));
		}
	}

    Mat eigenMat;
	cornerMinEigenVal(convertedPatch, eigenMat, patch.rows, CV_SCHARR);

    double eigenValue = 0.0, blankVal;

    minMaxLoc(eigenMat, &blankVal, &eigenValue);

    for (unsigned int iii = 0; iii < patch.rows; iii++) {
		for (unsigned int jjj = 0; jjj < patch.cols; jjj++) {
			//eigenValue += eigenMat.at<float>(iii,jjj) / ((float) (eigenMat.rows * eigenMat.cols));
		}
	}

	return eigenValue;

}
*/

void assignStabilityResponses(const Mat& img, vector<KeyPoint>& pts) {

    Mat outVals, workIm;

    if (img.channels() == 1) {
        img.copyTo(workIm);
    } else if (img.channels() == 3) {
        cvtColor(img, workIm, cv::COLOR_RGB2GRAY);
    }

    for (int iii = 0; iii < pts.size(); iii++) {

        double radius = ((double) pts.at(iii).size) / 2.0; // consider dividing radius by a further 2 so that it only looks at central part of MSER
        double stability = estimateStability(workIm, pts.at(iii).pt, radius);

        //printf("%s << stability(%d) = %f\n", __FUNCTION__, iii, stability);

        pts.at(iii).response = stability;

    }

}

void assignEigenVectorResponses(const Mat& img, vector<KeyPoint>& pts) {

    Mat outVals, workIm;

    //printf("%s << img.channels() = %d\n", __FUNCTION__, img.channels());

    if (img.channels() == 1) {
        img.copyTo(workIm);
    } else if (img.channels() == 3) {
        cvtColor(img, workIm, cv::COLOR_RGB2GRAY);
    }

    //printf("%s << Entered.\n", __FUNCTION__);

    // initial test...
    cornerMinEigenVal(workIm, outVals, 3, 3);

    //imshow("eigenwin", outVals);
    //waitKey();

    for (int iii = 0; iii < pts.size(); iii++) {

        double radius = ((double) pts.at(iii).size) / 2.0;
        double salience = estimateSalience(workIm, pts.at(iii).pt, radius);
        //printf("%s << response1(%d) = %f\n", __FUNCTION__, iii, pts.at(iii).response);

        pts.at(iii).response = salience;

        //printf("%s << response2(%d) = %f\n", __FUNCTION__, iii, pts.at(iii).response);

        // Extract patch from image

        // Mask patch

        // Normalize patch (using only mask)

        // Mask again (if needed)

        // Determine eigenvector

        // Use eigen vector as feature response
    }
}

void assignChronologicalResponses(vector<KeyPoint>& pts) {
    for (int iii = 0; iii < pts.size(); iii++) {
        pts.at(iii).response = double(iii); // double(pts.size()) - double(iii);

    }
}

void assignMinimumRadii(vector<KeyPoint>& pts) {
    for (int iii = 0; iii < pts.size(); iii++) {
        pts.at(iii).size = 6.0;

        //printf("%s << response = %f\n", __FUNCTION__, pts.at(iii).response);

    }
}

void makeAnnotationPair(Mat& annotationPair, Mat im1, Mat im2) {
    annotationPair = Mat(Size(im1.cols+im2.cols, im1.rows), im1.type());

    for (int iii = 0; iii < im1.rows; iii++) {
        for (int jjj = 0; jjj < im1.cols; jjj++) {
            if (im1.type() == CV_8UC3) {
                annotationPair.at<Vec3b>(iii,jjj) = im1.at<Vec3b>(iii,jjj);
                annotationPair.at<Vec3b>(iii,jjj+im1.cols) = im2.at<Vec3b>(iii,jjj);
            } else if (im1.type() == CV_8UC1) {
                annotationPair.at<unsigned char>(iii,jjj) = im1.at<unsigned char>(iii,jjj);
                annotationPair.at<unsigned char>(iii,jjj+im1.cols) = im2.at<unsigned char>(iii,jjj);
            }

        }
    }
}



double evaluateDifference(Mat& targetImage, Mat& transformedImage) {
    double retVal = 0;
    int activePixelCount = 0;

    Mat diffMat = targetImage - transformedImage;

    /*
    printf("%s << Showing difference image.\n", __FUNCTION__);
    imshow("warp", diffMat);
    waitKey(0);
    */

    for (int iii = 0; iii < targetImage.rows; iii++) {
        for (int jjj = 0; jjj < targetImage.cols; jjj++) {
            unsigned char diff, targ, tran;

            diff = diffMat.at<unsigned char>(iii,jjj);
            targ = targetImage.at<unsigned char>(iii,jjj);
            tran = transformedImage.at<unsigned char>(iii,jjj);

            // Probably an unmappable point so don't add to record
            if ((diff == targ) && (tran == 0)) {
                // ...
            } else {
                retVal += double(diff);
                activePixelCount++;
            }
        }
    }

    retVal /= activePixelCount;

    return retVal;

}

bool retainMostAppropriateHomography(Mat& originalImage, Mat& targetImage, Mat& originalCandidate, Mat newCandidate) {

    bool newCandidateIsValid = true;

    for (int iii = 0; iii < originalCandidate.rows; iii++) {
        for (int jjj = 0; jjj < originalCandidate.cols; jjj++) {
            float a = originalCandidate.at<float>(iii,jjj);
            float b = newCandidate.at<float>(iii,jjj);

            //printf("%s << a = %f; b = %f\n", __FUNCTION__, a, b);

            if (jjj == 2) {
                if (iii == 2) {
                    // should both be about 1.00
                    if ((abs(a - 1.00) > 0.01) || (abs(b - 1.00) > 0.01)) {
                        newCandidateIsValid = false;
                    }
                } else {
                    // translations, can be very lenient (in fact, don't check at all!)
                }
            } else {
                // if (max(abs(a),abs(b)) > 2.0*min(abs(a),abs(b))) {    }

                // These should all be fairly similar, right? And around the value of 1??
                if (abs(a-b) > 1.00) {
                    newCandidateIsValid = false;
                }
            }
        }
    }

    if (!newCandidateIsValid) {
        return false;
    } else {
        //printf("%s << New candidate is potentially valid, checking difference images...\n", __FUNCTION__);
    }

    /*
    printf("%s << Showing target image.\n", __FUNCTION__);
    imshow("warp", targetImage);
    waitKey(0);
    */

    Mat warpedImage, diffImage;

    warpPerspective(originalImage, warpedImage, originalCandidate, originalImage.size());

    /*
    printf("%s << Showing warped image using original candidate.\n", __FUNCTION__);
    imshow("warp", warpedImage);
    waitKey(0);
    */

    double differenceScore1, differenceScore2;

    differenceScore1 = evaluateDifference(targetImage, warpedImage);

    warpPerspective(originalImage, warpedImage, newCandidate, originalImage.size());

    /*
    printf("%s << Showing warped image using new candidate.\n", __FUNCTION__);
    imshow("warp", warpedImage);
    waitKey(0);
    */

    differenceScore2 = evaluateDifference(targetImage, warpedImage);

    // printf("%s << original differenceScore = %f\n, new differenceScore = %f\n", __FUNCTION__, differenceScore1, differenceScore2);

    if (differenceScore2 < differenceScore1) {
        newCandidate.copyTo(originalCandidate);
        return true;
    } else {
        return false;
    }

}

homographyHandler::homographyHandler() {
    homographyExtractor = DescriptorExtractor::create( "SURF" );
    homographyDetector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SURF"), 400, 500, 10);
    ransacReprojThreshold = 1.0;
    descriptorMatcher = DescriptorMatcher::create("BruteForce");
}

void initializeKeypointsTripleVec(vector< vector< vector<KeyPoint> > > *kpVec) {

    for (int zzz = 0; zzz < MAX_TRANSFORM_LEVELS; zzz++) {
        for (int iii = 0; iii < DD_DETECTORS_COUNT; iii++) {
            vector< vector<KeyPoint> > doubleVec;
            for (int jjj = 0; jjj < MAX_IMAGES_PER_FOLDER; jjj++) {
                vector<KeyPoint> singleVec;
                doubleVec.push_back(singleVec);
            }
            kpVec[zzz].push_back(doubleVec);
        }
    }

}

eccBlockWrapper::eccBlockWrapper() {
    // eccBlock variables
    number_of_iterations = 50;
    termination_eps      = 0.001;
    motion               = "perspective"; // "affine"
    warp_init            = NULL;
    warp_to_file         = NULL;
    image_to_file        = "warped.png";
    verbose              = false;
    target_image         = NULL;
    template_image       = NULL;
    warp_matrix          = NULL;
    warp_mode = (!strcmp(motion,"affine")) ? WARP_MODE_AFFINE : WARP_MODE_HOMOGRAPHY;
}

void fileTree::makeDirectories(const configData& iD, unsigned int level, unsigned int idx_0, unsigned int idx_1, unsigned int idx_2) {

    char directory_path[256];

    if (level == 0) {
        sprintf(directory_path, "%s/homographies/%s", iD.dataPath, datasets.at(idx_0).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/keypoints/%s", iD.dataPath, datasets.at(idx_0).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/descriptors/%s", iD.dataPath, datasets.at(idx_0).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/results/%s", iD.dataPath, datasets.at(idx_0).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/match_results/%s", iD.dataPath, datasets.at(idx_0).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/descresults/%s", iD.dataPath, datasets.at(idx_0).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);
    } else if (level == 1) {
        sprintf(directory_path, "%s/homographies/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/keypoints/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/results/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/match_results/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/descriptors/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/descresults/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);
    } else if (level == 2) {
        sprintf(directory_path, "%s/homographies/%s/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str(), subsets.at(idx_2).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/keypoints/%s/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str(), subsets.at(idx_2).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/descriptors/%s/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str(), subsets.at(idx_2).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/results/%s/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str(), subsets.at(idx_2).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/match_results/%s/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str(), subsets.at(idx_2).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);

        sprintf(directory_path, "%s/descresults/%s/%s/%s", iD.dataPath, datasets.at(idx_0).c_str(), modalities.at(idx_1).c_str(), subsets.at(idx_2).c_str());
        mkdir(directory_path, DEFAULT_MKDIR_PERMISSIONS);
    }


}

void assembleSubDirectories(char *imageParentPath, vector<string>& subDirectories) {

    path pathname = imageParentPath;
    directory_iterator end_iter;

    for( directory_iterator dir_iter(imageParentPath) ; dir_iter != end_iter ; ++dir_iter) {
        printf("%s << folder = %s\n", __FUNCTION__, dir_iter->path().string().c_str());

        string lastPart = (dir_iter->path().string()).substr(dir_iter->path().string().length()-4, 4);

        printf("%s << last part = %s\n", __FUNCTION__, lastPart.c_str());
        subDirectories.push_back(lastPart);

    }

    std::sort(subDirectories.begin(), subDirectories.end());

}

void addressNode::processImage(string imageName, vector<Mat>& rawImageVec, vector<Mat>& grayImageVec, vector<Mat>& colImageVec) {

    sprintf(imageAddress, "%s/%s", imagePath, imageName.c_str());

    Mat currImg, grayImg, colImg, drawImg;

    currImg = imread(imageAddress, -1);

    imshow("testWin", currImg);
    waitKey( 50 ); // 50

    rawImageVec.push_back(currImg);

    if (currImg.channels() == 3) {
        currImg.copyTo(colImg);
        cvtColor(currImg, grayImg, cv::COLOR_RGB2GRAY);

    } else {
        currImg.copyTo(grayImg);
        cvtColor(currImg, colImg, cv::COLOR_GRAY2RGB);
    }

    grayImageVec.push_back(grayImg);
    colImageVec.push_back(colImg);

}

void displayMatchesNeatly(Mat& im1, Mat& im2, vector<KeyPoint>& pts1, vector<KeyPoint>& pts2, vector<DMatch>& matches, Mat& dst) {

    // anti-alias circles
    // draw circles AFTER lines


    Mat im1_cpy, im2_cpy, im1_cpy2, im2_cpy2;

    cvtColor(im1, im1_cpy, cv::COLOR_GRAY2RGB);
    cvtColor(im2, im2_cpy, cv::COLOR_GRAY2RGB);

    vector<KeyPoint> fPts1, fPts2;

    for (unsigned int iii = 0; iii < matches.size(); iii++) {
        fPts1.push_back(pts1.at(matches.at(iii).queryIdx));
        fPts2.push_back(pts2.at(matches.at(iii).trainIdx));
        fPts2.at(iii).pt.x += im1.cols + 32.0;
    }

    // Create new image

    dst = Mat::zeros(im1.rows, 2*im1.cols + 32, CV_8UC3);

    for (unsigned int iii = 0; iii < im1.rows; iii++) {
        for (unsigned int jjj = im1.cols; jjj < im1.cols+32; jjj++) {
            dst.at<Vec3b>(iii,jjj)[0] = 255;
            dst.at<Vec3b>(iii,jjj)[1] = 255;
            dst.at<Vec3b>(iii,jjj)[2] = 255;
        }
    }

    printf("%s << DEBUG %d\n", __FUNCTION__, 3);


    //drawKeypoints(im1_cpy, pts1, im1_cpy2);
    //drawKeypoints(im2_cpy, pts1, im2_cpy2);

    for (unsigned int iii = 0; iii < im1.rows; iii++) {
        for (unsigned int jjj = 0; jjj < im1.cols; jjj++) {
            dst.at<Vec3b>(iii,jjj)[0] = im1_cpy.at<Vec3b>(iii,jjj)[0];
            dst.at<Vec3b>(iii,jjj)[1] = im1_cpy.at<Vec3b>(iii,jjj)[1];
            dst.at<Vec3b>(iii,jjj)[2] = im1_cpy.at<Vec3b>(iii,jjj)[2];

            dst.at<Vec3b>(iii,jjj+im1.cols+32)[0] = im2_cpy.at<Vec3b>(iii,jjj)[0];
            dst.at<Vec3b>(iii,jjj+im1.cols+32)[1] = im2_cpy.at<Vec3b>(iii,jjj)[1];
            dst.at<Vec3b>(iii,jjj+im1.cols+32)[2] = im2_cpy.at<Vec3b>(iii,jjj)[2];
        }
    }

    printf("%s << DEBUG %d\n", __FUNCTION__, 4);

    for (unsigned int iii = 0; iii < matches.size(); iii++) {
        Point p1, p2;

        p1 = Point(fPts1.at(iii).pt.x * 16, fPts1.at(iii).pt.y * 16);
        p2 = Point((fPts2.at(iii).pt.x) * 16, fPts2.at(iii).pt.y * 16);
        //  + im1.cols + 32
        //printf("%s << p1 = (%d, %d); p2 = (%d, %d)\n");
        line(dst, p1, p2, cv::Scalar(0,0,255), 1, cv::LINE_AA, 4);

//        fPts1.push_back(pts1.at(matches.at(iii).trainIdx));
//        fPts2.push_back(pts2.at(matches.at(iii).queryIdx));
    }

    printf("%s << DEBUG %d\n", __FUNCTION__, 5);

    displayKeypoints(dst, fPts1, dst, cv::Scalar(255,0,0), 0);
    displayKeypoints(dst, fPts2, dst, cv::Scalar(255,0,0), 0);

    printf("%s << DEBUG %d\n", __FUNCTION__, 7);
}

#endif