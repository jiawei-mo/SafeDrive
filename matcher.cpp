#include "matcher.hpp"

int Matcher::checkArea(Point2f p, int n_c, int n_r, int num_grid) {
    int step_c = n_c / num_grid + 1;
    int step_r = n_r / num_grid + 1;
    int c = p.x / step_c;
    int r = p.y / step_r;
    return r * num_grid + c;
}

Matcher::Matcher()
{
    blur_size = BS*2 + 1;
    blur_var = BV / 10.0f;
    max_num_features = MNF;
    quality_level = QL / 100.0f;
    min_distance = MD;
    num_grid_feature = NGF;
    match_thres_feature = MTF / 100.0f;
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Matcher::changeParam(int bs, float bv, int mnf, float ql, int md,  int ngf, float mtf)
{
    blur_size = bs;
    blur_var = bv;
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    num_grid_feature = ngf;
    match_thres_feature = mtf;
}

void Matcher::setTarget(const Mat& frame)
{
    targetFrame = frame.clone();
    Mat targetFrameBlured;
    GaussianBlur(targetFrame, targetFrameBlured, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(targetFrameBlured, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    targetKp.clear();
    for( size_t i = 0; i < corners.size(); i++ ) {
        targetKp.push_back(KeyPoint(corners[i], 1.f));
    }
    detector->compute(targetFrameBlured, targetKp, targetDesc);
}

void Matcher::match(const Mat& db_frame, vector<Point2f>& targetMatchedKp, vector<Point2f>& dbMatchedKp, bool showImg, string windowName, int num_grid, float match_thres)
{
    if(db_frame.empty()) {
        return;
    }
    if(num_grid<0) {
        num_grid = num_grid_feature;
    }
    if(match_thres<0) {
        match_thres = match_thres_feature;
    }

    //detect feature points and extract descriptors on dbFrame
    Mat dbBlured;
    GaussianBlur(db_frame, dbBlured, Size(blur_size,blur_size), blur_var, blur_var);
    Mat dbGray;
    vector<Point2f> corners;
    cvtColor(dbBlured, dbGray, CV_BGR2GRAY);
    goodFeaturesToTrack(dbGray, corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> dbKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        dbKp.push_back(KeyPoint(corners[i], 1.f));
    }

    // match result image
    Mat matchedImg;
    if(showImg)
    {
        matchedImg = Mat::zeros(db_frame.rows, 2*db_frame.cols, db_frame.type());
        Mat dbFrameFeatures = dbBlured.clone();
        Mat targetFrameFeatures = targetFrame.clone();
        drawKeypoints(dbFrameFeatures, dbKp, dbFrameFeatures);
        drawKeypoints(targetFrameFeatures, targetKp, targetFrameFeatures);
        dbFrameFeatures.copyTo(matchedImg(Rect(0, 0, db_frame.cols, db_frame.rows)));
        targetFrameFeatures.copyTo(matchedImg(Rect(db_frame.cols, 0, db_frame.cols, db_frame.rows)));
    }

    //************************************************grid matches**************************************************************
    int grid_num = num_grid*num_grid;

    vector<vector<KeyPoint> > gridTargetKp(grid_num, vector<KeyPoint>());
    vector<Mat> gridTargetDesc(grid_num, Mat());

    vector<vector<KeyPoint> > griddbKp(grid_num, vector<KeyPoint>());
    vector<Mat> griddbDesc(grid_num, Mat());

    int n_c = targetFrame.cols;
    int n_r = targetFrame.rows;
    for(int i=0; i<(int)targetKp.size(); i++) {
        int area = checkArea(targetKp[i].pt, n_c, n_r, num_grid);
        gridTargetKp[area].push_back(targetKp[i]);
    }
    for(int i=0; i<(int)dbKp.size(); i++) {
        int area = checkArea(dbKp[i].pt, n_c, n_r, num_grid);
        griddbKp[area].push_back(dbKp[i]);
    }

    Mat targetFrameBlured;
    GaussianBlur(targetFrame, targetFrameBlured, Size(blur_size,blur_size), blur_var, blur_var);

    for(int i=0; i<grid_num; i++) {
        detector->compute(targetFrameBlured, gridTargetKp[i], gridTargetDesc[i]);
        detector->compute(dbBlured, griddbKp[i], griddbDesc[i]);
    }

    for(int i=0; i<grid_num; i++) {
        vector<vector<DMatch> > matches;
        if(gridTargetDesc[i].empty() || griddbDesc[i].empty()) {
            continue;
        }
        matcher->knnMatch(griddbDesc[i], gridTargetDesc[i], matches, 2);
        vector<DMatch> goodMatches;
        for(int i=0; i<(int)matches.size(); i++) {
            if(matches[i][0].distance < match_thres*matches[i][1].distance)
            {
                matches[i][0].distance = matches[i][0].distance / matches[i][1].distance;
                goodMatches.push_back(matches[i][0]);
            }
        }
        vector<DMatch> matchHeap;
        if((int)goodMatches.size() > 40) {
            matchHeap = vector<DMatch>(goodMatches.begin(), goodMatches.begin()+40);
            make_heap(matchHeap.begin(), matchHeap.end(), matchComp());

            for(int j=40; j<(int)goodMatches.size(); j++) {
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
//            cout<<griddbKp[i][matchHeap[j].queryIdx].pt<<endl;
            dbMatchedKp.push_back(griddbKp[i][matchHeap[j].queryIdx].pt);
            targetMatchedKp.push_back(gridTargetKp[i][matchHeap[j].trainIdx].pt);
        }
    }
    if(dbMatchedKp.size() == 0) {
        return;
    }
    //************************************************grid matches**************************************************************


    //show matches between two images side by side
    for(int i=0; i<(int)targetMatchedKp.size(); i++)
    {
        if(showImg) {
            line(matchedImg, dbMatchedKp[i], Point2f(targetMatchedKp[i].x+db_frame.cols, targetMatchedKp[i].y), CV_RGB(255, 0, 0));
        }
    }


    if(showImg) {
        namedWindow(windowName, WINDOW_NORMAL);
        imshow(windowName, matchedImg);
    }

    return;
}

void Matcher::showDifference(const Mat& image1, const Mat& image2, string title)
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
    if(title.size()>1) {
        namedWindow(title, WINDOW_NORMAL);
        imshow(title, imgSh);
    }
}

void Matcher::showDifferenceEdge(const Mat& image1, const Mat& image2, string title)
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

    if(title.size()>1) {
        namedWindow(title, WINDOW_NORMAL);
        imshow(title, res);
    }

    return;
}
