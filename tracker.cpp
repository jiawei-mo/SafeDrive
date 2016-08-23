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
    nn_match_thres = NMT / 100.0f;
    ransac_thres = RT / 1.0f;
    board_size = BDS*2 + 1;
    num_grid = NG;
    pp_grid = PG;
    blur_size_grid = BSG*2 + 1;
    blur_var_grid = BVG / 10.0f;
    blur_scale_grid = BSCG*2 + 1;
    match_thres_grid = MTG / 100.0f;
    ransac_thres_grid = RTG / 1.0f;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::changeParam(int mnf, float ql, int md, int bs, float bv, float nmt, float rt, int bds, int ng, int pg, int bsg, int bvg, int bscg, float mtg, float rtg)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    blur_size = bs;
    blur_var = bv;
    nn_match_thres = nmt;
    ransac_thres = rt;
    board_size = bds;
    num_grid = ng;
    pp_grid = pg;
    blur_size_grid = bsg;
    blur_var_grid = bvg;
    blur_scale_grid = bscg;
    match_thres_grid = mtg;
    ransac_thres_grid = rtg;
    targetKp.clear();
}

void Tracker::setTarget(const Mat& frame)
{
    GaussianBlur(frame, targetFrame, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(targetFrame, grayImg, CV_BGR2GRAY);
    mask = Mat::zeros(frame.size(), CV_8UC1);
    mask(Rect(0,0,mask.cols/3,mask.rows/2)).setTo(Scalar::all(255));
    mask(Rect(mask.cols*2/3,0,mask.cols/3,mask.rows/2)).setTo(Scalar::all(255));
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    for( size_t i = 0; i < corners.size(); i++ ) {
        targetKp.push_back(KeyPoint(corners[i], 1.f));
    }
    detector->compute(targetFrame, targetKp, targetDesc);
}

int Tracker::featureMatch(const Mat& frame, Mat& homography, bool showImg, string windowName)
{
    //detect feature points and extract descriptors on curFrame
    Mat curFrame;
    GaussianBlur(frame, curFrame, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(curFrame, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> curKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        curKp.push_back(KeyPoint(corners[i], 1.f));
    }

    Mat curDesc;
    detector->compute(curFrame, curKp, curDesc);

    // match result image
    Mat matchedImg;
    if(showImg) {
        matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
        Mat curFrameFeatures = curFrame.clone();
        Mat targetFrameFeatures = targetFrame.clone();
        drawKeypoints(curFrameFeatures, curKp, curFrameFeatures);
        drawKeypoints(targetFrameFeatures, targetKp, targetFrameFeatures);
        curFrameFeatures.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
        targetFrameFeatures.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));
    }

    //two-way matches
    vector<Point2f> targetMatchedKp, curMatchedKp;
    vector< vector<DMatch> > ctMatches, tcMatches;
    matcher->knnMatch(curDesc, targetDesc, ctMatches, 2);
    matcher->knnMatch(targetDesc, curDesc, tcMatches, 2);
    unordered_map<int, int> matchMap;

    for(int i=0; i<(int)tcMatches.size(); i++)
    {
        if(tcMatches[i][0].distance < nn_match_thres*tcMatches[i][1].distance)
        {
            matchMap[tcMatches[i][0].trainIdx] = tcMatches[i][0].queryIdx;
        }
    }

    for(int i=0; i<(int)ctMatches.size(); i++)
    {
        if(ctMatches[i][0].distance < nn_match_thres*ctMatches[i][1].distance)
        {
            if(matchMap[ctMatches[i][0].queryIdx] == ctMatches[i][0].trainIdx)
            {
                curMatchedKp.push_back(curKp[ctMatches[i][0].queryIdx].pt);
                targetMatchedKp.push_back(targetKp[ctMatches[i][0].trainIdx].pt);
            }
        }
    }

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
            imshow("Match fail Result", matchedImg);
        }
        cout<<"Match fail, norm exceeded!"<<endl;
        return -1;
    }

    vector<Point2f> projectedKp;
    perspectiveTransform(curMatchedKp, projectedKp, homography);

    //show inliner pairs between two images side by side
    int inliner_counter = 0;
    float inliner_dist = 0.0;
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
        }
    }

    //FAIL: proj error exceeded
    inliner_dist = inliner_dist / inliner_counter;
    cout<<"norm: "<<norm(homography)<<" inliner_dist: "<<inliner_dist<<endl;
    if(inliner_dist > PROJ_ERR_THRES) {
        cout<<"Match fail, proj error exceeded!"<<endl;
        return -1;
    }

    if(showImg) {
        imshow(windowName, matchedImg);
    }

    return inliner_counter;
}

void Tracker::gridInline(const Mat& frame, vector<Point2f> *inline_target, vector<Point2f> *inline_matched)
{
    //detect feature points and extract descriptors on curFrame
    Mat curFrame;
    GaussianBlur(frame, curFrame, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(curFrame, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> curKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        curKp.push_back(KeyPoint(corners[i], 1.f));
    }

    Mat matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
    Mat curFrameFeatures = curFrame.clone();
    Mat targetFrameFeatures = targetFrame.clone();
    drawKeypoints(curFrameFeatures, curKp, curFrameFeatures);
    drawKeypoints(targetFrameFeatures, targetKp, targetFrameFeatures);
    curFrameFeatures.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
    targetFrameFeatures.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));

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

    for(int i=0; i<grid_num; i++) {
        detector->compute(targetFrame, gridTargetKp[i], gridTargetDesc[i]);
        detector->compute(frame, gridCurKp[i], gridCurDesc[i]);
    }

    for(int i=0; i<grid_num; i++) {
        vector<vector<DMatch> > matches;
        if(gridTargetDesc[i].empty() || gridCurDesc[i].empty()) {
            continue;
        }
        matcher->knnMatch(gridCurDesc[i], gridTargetDesc[i], matches, 2);
        vector<DMatch> goodMatches;
        for(int i=0; i<(int)matches.size(); i++) {
            if(matches[i][0].distance < match_thres_grid*matches[i][1].distance)
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
    //************************************************grid matches**************************************************************

    //RANSAC to remove outliers
    Mat inliner_mask;
    findHomography(curMatchedKp, targetMatchedKp, RANSAC, ransac_thres_grid, inliner_mask);

    for(int i=0; i<(int)targetMatchedKp.size(); i++)
    {
        if(inliner_mask.at<uchar>(i))
        {
            line(matchedImg, curMatchedKp[i], Point2f(targetMatchedKp[i].x+frame.cols, targetMatchedKp[i].y), CV_RGB(255, 0, 0));
            inline_target->push_back(targetMatchedKp[i]);
            inline_matched->push_back(curMatchedKp[i]);
        }
    }

    imshow("Grid Match", matchedImg);
}

Mat Tracker::pixelMatch(const Mat& recMatchedFrame)
{
    vector<Point2f> inline_target, inline_matched;
    gridInline(recMatchedFrame, &inline_target, &inline_matched);
    Mat targetDots = Mat::zeros(targetFrame.size(), CV_32F);;
    Mat recMatchedDots = Mat::zeros(recMatchedFrame.size(), CV_32F);
    if(inline_target.size() == 0) {
        return Mat::ones(3, 3, CV_32F);
    }
    Mat targetGray, recMatchedGray;
    cvtColor(targetFrame, targetGray, CV_BGR2GRAY);
    cvtColor(recMatchedFrame, recMatchedGray, CV_BGR2GRAY);
    for(unsigned int i=0; i<inline_target.size(); i++) {
        targetDots.at<float>(inline_target[i].y, inline_target[i].x) = 1.0;
        recMatchedDots.at<float>(inline_matched[i].y, inline_matched[i].x) = 1.0;
    }

    Mat targetROI, recMatchedROI;
    GaussianBlur(targetFrame, targetROI, Size(3*blur_size_grid, 3*blur_size_grid), blur_var_grid*blur_scale_grid, blur_var_grid*blur_scale_grid);
    GaussianBlur(recMatchedFrame, recMatchedROI, Size(3*blur_size_grid, 3*blur_size_grid), blur_var_grid*blur_scale_grid, blur_var_grid*blur_scale_grid);

    blur(targetDots, targetDots, Size(board_size, board_size));
    threshold(targetDots, targetDots, 0, 1, cv::THRESH_BINARY);
    targetDots.convertTo(targetDots, CV_8U);
    targetFrame.copyTo(targetROI, targetDots);
    GaussianBlur(targetROI, targetROI, Size(blur_size_grid, blur_size_grid), blur_var_grid, blur_var_grid);

    blur(recMatchedDots, recMatchedDots, Size(board_size, board_size));
    threshold(recMatchedDots, recMatchedDots, 0, 1, cv::THRESH_BINARY);
    recMatchedDots.convertTo(recMatchedDots, CV_8U);
    recMatchedFrame.copyTo(recMatchedROI, recMatchedDots);
    GaussianBlur(recMatchedROI, recMatchedROI, Size(blur_size_grid, blur_size_grid), blur_var_grid, blur_var_grid);

    Mat combineImg = Mat::zeros(recMatchedROI.rows, 2*recMatchedROI.cols, recMatchedROI.type());
    recMatchedROI.copyTo(combineImg(Rect(0, 0, recMatchedROI.cols, recMatchedROI.rows)));
    targetROI.copyTo(combineImg(Rect(recMatchedROI.cols, 0, recMatchedROI.cols, recMatchedROI.rows)));
    imshow("Pixel-wise ROI", combineImg);

    Mat recMatchedFrame64F, targetFrame64F;
    recMatchedROI.convertTo(recMatchedFrame64F, CV_64FC3);
    targetROI.convertTo(targetFrame64F, CV_64FC3);
    Mat homo = pixelWiseMatch(recMatchedFrame64F, targetFrame64F);
    return homo;
}

Mat Tracker::pixelWiseMatch(const Mat& img1, const Mat& img2)
{
    Ptr<Map> res;
    MapperGradProj mapper;
    MapperPyramid mappPyr(mapper);
//    mappPyr.numIterPerScale_ = 10;
    mappPyr.calculate(img1, img2, res);

    MapProjec* mapProj = dynamic_cast<MapProjec*>(res.get());
    mapProj->normalize();

    // Generate diffImg
    Mat dest;
    mapProj->inverseWarp(img2, dest);
    showDifferenceEdge(img1, dest, "Pixel Difference");

    return Mat(mapProj->getProjTr());
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
    imshow(title, imgSh);
}

void Tracker::showDifferenceEdge(const Mat& image1, const Mat& image2, string title)
{
    Mat img1Tmp, img2Tmp, img1Edge, img2Edge;
    image1.convertTo(img1Tmp, CV_8UC3);
    image2.convertTo(img2Tmp, CV_8UC3);
    Canny( img1Tmp, img1Edge, 50, 100, 3);
    Canny( img2Tmp, img2Edge, 50, 100, 3);

    Mat res(image1.size(), CV_8UC3, Scalar(0, 0, 0));
    Mat redImg(image1.size(), CV_8UC3, Scalar(255, 0, 0));
    Mat blueImg(image1.size(), CV_8UC3, Scalar(0, 0, 255));
    redImg.copyTo(res, img1Edge);
    blueImg.copyTo(res, img2Edge);

    imshow(title, res);

    return;
}
