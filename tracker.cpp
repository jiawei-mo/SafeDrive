#include "tracker.hpp"

int Tracker::checkArea(Point2f p, int n_c, int n_r) {
    int step_c = n_c / num_grid + 1;
    int step_r = n_r / num_grid + 1;
    int c = p.x / step_c;
    int r = p.y / step_r;
    return r * num_grid + c;
}

Tracker::Tracker()
{
    blur_size = BS*2 + 1;
    blur_var = BV / 10.0f;
    max_num_features = MNF;
    quality_level = QL / 100.0f;
    min_distance = MD;
    match_thres_tight = NMT / 100.0f;
    ransac_thres_tight = RT / 1.0f;
    board_size = BDS*2 + 1;
    num_grid = NG;
    pp_grid = PG;
    blur_size_grid = BSG*2 + 1;
    blur_var_grid = BVG / 10.0f;
    match_thres_loose = MTG / 100.0f;
    ransac_thres_loose = RTG / 1.0f;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::changeParam(int mnf, float ql, int md, int bs, float bv, float nmt, float rt, int bds, int ng, int pg, int bsg, int bvg, float mtg, float rtg)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    blur_size = bs;
    blur_var = bv;
    match_thres_tight = nmt;
    ransac_thres_tight = rt;
    board_size = bds;
    num_grid = ng;
    pp_grid = pg;
    blur_size_grid = bsg;
    blur_var_grid = bvg;
    match_thres_loose = mtg;
    ransac_thres_loose = rtg;
    targetKp.clear();
}

void Tracker::setTarget(const Mat& frame)
{
    targetFrame = frame.clone();
    Mat targetFrameBlured;
    GaussianBlur(targetFrame, targetFrameBlured, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(targetFrameBlured, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    for( size_t i = 0; i < corners.size(); i++ ) {
        targetKp.push_back(KeyPoint(corners[i], 1.f));
    }
    detector->compute(targetFrameBlured, targetKp, targetDesc);
}

double Tracker::featureMatch(const Mat& frame, Mat& homography, vector<Point2f> *inline_matched, float match_thres, float ransac_thres, bool showImg, string windowName)
{
    if(match_thres<0) {
        match_thres = match_thres_tight;
    }
    if(ransac_thres<0) {
        ransac_thres = ransac_thres_tight;
    }
    //detect feature points and extract descriptors on curFrame
    Mat curBlured;
    GaussianBlur(frame, curBlured, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(curBlured, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> curKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        curKp.push_back(KeyPoint(corners[i], 1.f));
    }

    Mat curDesc;
    detector->compute(curBlured, curKp, curDesc);

    // match result image
    Mat matchedImg;
    if(showImg) {
        matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
        Mat curFrameFeatures = curBlured.clone();
        Mat targetFrameFeatures = targetFrame.clone();
        drawKeypoints(curFrameFeatures, curKp, curFrameFeatures);
        drawKeypoints(targetFrameFeatures, targetKp, targetFrameFeatures);
        curFrameFeatures.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
        targetFrameFeatures.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));
    }

    //************************************************grid matches**************************************************************
    vector<Point2f> targetMatchedKp, curMatchedKp;
    int grid_num = num_grid*num_grid;

    vector<vector<KeyPoint> > gridTargetKp(grid_num, vector<KeyPoint>());
    vector<Mat> gridTargetDesc(grid_num, Mat());

    vector<vector<KeyPoint> > gridCurKp(grid_num, vector<KeyPoint>());
    vector<Mat> gridCurDesc(grid_num, Mat());

    int n_c = targetFrame.cols;
    int n_r = targetFrame.rows;
    for(int i=0; i<(int)targetKp.size(); i++) {
        int area = checkArea(targetKp[i].pt, n_c, n_r);
        gridTargetKp[area].push_back(targetKp[i]);
    }
    for(int i=0; i<(int)curKp.size(); i++) {
        int area = checkArea(curKp[i].pt, n_c, n_r);
        gridCurKp[area].push_back(curKp[i]);
    }

    Mat targetFrameBlured;
    GaussianBlur(targetFrame, targetFrameBlured, Size(blur_size,blur_size), blur_var, blur_var);
    for(int i=0; i<grid_num; i++) {
        detector->compute(targetFrameBlured, gridTargetKp[i], gridTargetDesc[i]);
        detector->compute(curBlured, gridCurKp[i], gridCurDesc[i]);
    }

    for(int i=0; i<grid_num; i++) {
        vector<vector<DMatch> > matches;
        if(gridTargetDesc[i].empty() || gridCurDesc[i].empty()) {
            continue;
        }
        matcher->knnMatch(gridCurDesc[i], gridTargetDesc[i], matches, 2);
        vector<DMatch> goodMatches;
        for(int i=0; i<(int)matches.size(); i++) {
            if(matches[i][0].distance < match_thres*matches[i][1].distance)
            {
                matches[i][0].distance = matches[i][0].distance / matches[i][1].distance;
                goodMatches.push_back(matches[i][0]);
            }
        }
        vector<DMatch> matchHeap;
        if((int)goodMatches.size() > pp_grid) {
            matchHeap = vector<DMatch>(goodMatches.begin(), goodMatches.begin()+pp_grid);
            make_heap(matchHeap.begin(), matchHeap.end(), matchComp());

            for(int j=pp_grid; j<(int)goodMatches.size(); j++) {
                if(matchHeap.front().distance > goodMatches[j].distance) {
                    pop_heap(matchHeap.begin(), matchHeap.end(), matchComp());
                    matchHeap.pop_back();
                    matchHeap.push_back(goodMatches[j]);
                    push_heap(matchHeap.begin(), matchHeap.end(), matchComp());
                }
            }
        } else {
            matchHeap = goodMatches;
        }

        for(int j=0; j<(int)matchHeap.size(); j++) {
            curMatchedKp.push_back(gridCurKp[i][matchHeap[j].queryIdx].pt);
            targetMatchedKp.push_back(gridTargetKp[i][matchHeap[j].trainIdx].pt);
        }
    }

    if(curMatchedKp.size() < 20) {
        cout<<"Match fail, not enough matches: "<<curMatchedKp.size()<<endl;
        return PROJ_ERR_THRES;
    }
    //************************************************grid matches**************************************************************


    //find homography based on matches using RANSAC
    Mat inliner_mask;
    homography = findHomography(curMatchedKp, targetMatchedKp, RANSAC, ransac_thres, inliner_mask);

    //FAIL: norm exceeded
    if(norm(homography)>=HOMO_FAIL_NORM)
    {
        if(showImg) {
            for(int i=0; i<(int)targetMatchedKp.size(); i++)
            {
                targetMatchedKp[i].x += frame.cols;
                line(matchedImg, curMatchedKp[i], targetMatchedKp[i], CV_RGB(255, 0, 0));
            }
            namedWindow("Match fail Result", WINDOW_NORMAL);
            imshow("Match fail Result", matchedImg);
        }
        cout<<"Match fail, norm exceeded!"<<endl;
        return PROJ_ERR_THRES;
    }

    vector<Point2f> projectedKp;
    if(curMatchedKp.size() > 0){
        perspectiveTransform(curMatchedKp, projectedKp, homography);
    }

    //show inliner pairs between two images side by side
    int inliner_counter = 0;
    float inliner_dist = 0.0;
    inline_matched->clear();
    for(int i=0; i<(int)targetMatchedKp.size(); i++)
    {
        if(inliner_mask.at<uchar>(i))
        {
            inliner_counter++;
            inliner_dist += (projectedKp[i].x - targetMatchedKp[i].x)*(projectedKp[i].x - targetMatchedKp[i].x);
            inliner_dist += (projectedKp[i].y - targetMatchedKp[i].y)*(projectedKp[i].y - targetMatchedKp[i].y);
            if(showImg) {
                line(matchedImg, curMatchedKp[i], Point2f(targetMatchedKp[i].x+frame.cols, targetMatchedKp[i].y), CV_RGB(255, 0, 0));
                circle(matchedImg, Point2f(projectedKp[i].x+frame.cols, projectedKp[i].y), 2, CV_RGB(0, 255, 0));
            }
            inline_matched->push_back(curMatchedKp[i]);
        }
    }

    //FAIL: proj error exceeded
    inliner_dist = inliner_dist / inliner_counter;

    if(inliner_counter < 20) {
        cout<<"Match fail, not enough inliner: "<<inliner_counter<<endl;
        return PROJ_ERR_THRES;
    }

    if(showImg) {
        namedWindow(windowName, WINDOW_NORMAL);
        imshow(windowName, matchedImg);
    }

    cout<<"norm: "<<norm(homography)<<" inliner_dist: "<<inliner_dist<<endl;
    return inliner_dist;
}

Mat Tracker::pixelMatch(const Mat& recMatchedFrame)
{
    vector<Point2f> inline_matched;
    Mat initialH;
    featureMatch(recMatchedFrame, initialH, &inline_matched, match_thres_loose, ransac_thres_loose, true, "Grid inliners");
    Mat matchedROI = Mat::zeros(recMatchedFrame.size(), CV_32F);
    if(inline_matched.size() == 0) {
        return Mat::ones(3, 3, CV_32F);
    }
    if(inline_matched.size() == 0) {
        CV_Error(Error::StsNoConv, "No inliner for mask!");
    }
    for(unsigned int i=0; i<inline_matched.size(); i++) {
        matchedROI.at<float>(inline_matched[i].y, inline_matched[i].x) = 1.0;
    }

    blur(matchedROI, matchedROI, Size(board_size, board_size));
    threshold(matchedROI, matchedROI, 0, 1, cv::THRESH_BINARY);
    matchedROI.convertTo(matchedROI, CV_8U);

    Mat combineImg = Mat::zeros(recMatchedFrame.rows, 2*recMatchedFrame.cols, recMatchedFrame.type());
    recMatchedFrame.copyTo(combineImg(Rect(0, 0, recMatchedFrame.cols, recMatchedFrame.rows)), matchedROI);
    targetFrame.copyTo(combineImg(Rect(recMatchedFrame.cols, 0, recMatchedFrame.cols, recMatchedFrame.rows)));
    namedWindow("Pixel-wise matchedROI", WINDOW_NORMAL);
    imshow("Pixel-wise matchedROI", combineImg);

    Mat recMatched_gray, target_gray;
    cvtColor(recMatchedFrame, recMatched_gray, CV_BGR2GRAY);
    cvtColor(targetFrame, target_gray, CV_BGR2GRAY);
    Mat wrap_matrix = Mat::eye(3,3,CV_32F);
//    initialH.convertTo(wrap_matrix, CV_32F);

    int number_of_iterations = 50;
    double termination_eps = 1e-10;

    TermCriteria criteria (TermCriteria::COUNT+TermCriteria::EPS, number_of_iterations, termination_eps);

    ECC *myECC = new ECC();
    myECC->findTransformECC(recMatched_gray,
                     target_gray,
                     wrap_matrix,
                     MOTION_HOMOGRAPHY,
                     criteria,
                     matchedROI);
    return wrap_matrix;
}

void Tracker::showDifference(const Mat& image1, const Mat& image2, string title)
{
    Mat img1, img2;
    image1.convertTo(img1, CV_32FC3);
    image2.convertTo(img2, CV_32FC3);
    if(img1.channels() != 1)
        cvtColor(img1, img1, CV_RGB2GRAY);
    if(img2.channels() != 1)
        cvtColor(img2, img2, CV_RGB2GRAY);

    Mat imgDiff;
    img1.copyTo(imgDiff);
    imgDiff -= img2;
    imgDiff /= 2.f;
    imgDiff += 128.f;
    Mat imgSh;
    imgDiff.convertTo(imgSh, CV_8UC3);
    namedWindow(title, WINDOW_NORMAL);
    imshow(title, imgSh);
}

void Tracker::showDifferenceEdge(const Mat& image1, const Mat& image2, string title)
{
    Mat img1Tmp, img2Tmp, img1Edge, img2Edge;
    image1.convertTo(img1Tmp, CV_8UC3);
    image2.convertTo(img2Tmp, CV_8UC3);
    Canny( img1Tmp, img1Edge, 50, 150, 3);
    Canny( img2Tmp, img2Edge, 50, 150, 3);

    Mat res(image1.size(), CV_8UC3, Scalar(0, 0, 0));
    Mat redImg(image1.size(), CV_8UC3, Scalar(255, 0, 0));
    Mat blueImg(image1.size(), CV_8UC3, Scalar(0, 0, 255));
    redImg.copyTo(res, img1Edge);
    blueImg.copyTo(res, img2Edge);

    namedWindow(title, WINDOW_NORMAL);
    imshow(title, res);

    return;
}
