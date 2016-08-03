#include "tracker.hpp"

Tracker::Tracker()
{
    max_num_features = 1000;
    quality_level = 0.01f;
    min_distance = 7;
    blur_size = 3;
    blur_var = 1.2;
    nn_match_thres = 0.9f;
    ransac_thres = 20.0f;
    board_thres = 200;
    board_size = 10;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

Tracker::Tracker(int mnf, float ql, int md, int bs, float bv, float nmt, float rt, int bdt, int bds)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    blur_size = bs;
    blur_var = bv;
    nn_match_thres = nmt;
    ransac_thres = rt;
    board_thres = bdt;
    board_size = bds;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::changeParam(int mnf, float ql, int md, int bs, float bv, float nmt, float rt, int bdt, int bds)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    blur_size = bs;
    blur_var = bv;
    nn_match_thres = nmt;
    ransac_thres = rt;
    board_thres = bdt;
    board_size = bds;
    targetKp.clear();
}

void Tracker::setTarget(const Mat& frame)
{
    targetFrame = frame.clone();
    GaussianBlur(targetFrame, targetFrame, Size(blur_size,blur_size), blur_var, blur_var);
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

Mat Tracker::featureMatch(const Mat& frame, bool showImg)
{
    //detect feature points on curFrame
    Mat curFrame = frame.clone();
    GaussianBlur(curFrame, curFrame, Size(blur_size,blur_size), blur_var, blur_var);
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

    Mat matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
    curFrame.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
    targetFrame.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));

    //matches
    vector< vector<DMatch> > matches;
    matcher->knnMatch(curDesc, targetDesc, matches, 2);
    vector<Point2f> targetMatchedKp, curMatchedKp;
    for(int i=0; i<(int)matches.size(); i++)
    {
        circle(matchedImg, curKp[matches[i][0].queryIdx].pt, 2, CV_RGB(0, 0, 255));
        Point2f targetKpTmp = targetKp[matches[i][0].trainIdx].pt;
        targetKpTmp.x += frame.cols;
        circle(matchedImg, targetKpTmp, 2, CV_RGB(0, 0, 255));
        if(matches[i][0].distance < nn_match_thres*matches[i][1].distance)
        {
            curMatchedKp.push_back(curKp[matches[i][0].queryIdx].pt);
            targetMatchedKp.push_back(targetKp[matches[i][0].trainIdx].pt);
        }
    }

    //find homography based on matches
    Mat homography, inliner_mask;
    if(targetMatchedKp.size() >= NN_MATCH_NUMBER)
    {
        homography = findHomography(curMatchedKp, targetMatchedKp, RANSAC, ransac_thres, inliner_mask);
    }

    //track fail
    if(targetMatchedKp.size() < NN_MATCH_NUMBER || homography.empty() || norm(homography)>=HOMO_FAIL_NORM)
    {
        for(int i=0; i<(int)targetMatchedKp.size(); i++)
        {
            targetMatchedKp[i].x += frame.cols;
            line(matchedImg, curMatchedKp[i], targetMatchedKp[i], CV_RGB(255, 0, 0));
        }
        imshow("Match Result", matchedImg);
        cout<<"Match fail, norm exceeded!"<<endl;
        return homography;
    }

    vector<Point2f> projectedKp;
    perspectiveTransform(curMatchedKp, projectedKp, homography);

    //show inliner pairs between two images side by side
    int count = 0;
    float dist = 0.0;
    for(int i=0; i<(int)targetMatchedKp.size(); i++)
    {
        if(inliner_mask.at<uchar>(i))
        {
            count++;
            dist += (projectedKp[i].x - targetMatchedKp[i].x)*(projectedKp[i].x - targetMatchedKp[i].x);
            dist += (projectedKp[i].y - targetMatchedKp[i].y)*(projectedKp[i].y - targetMatchedKp[i].y);
            targetMatchedKp[i].x += frame.cols;
            projectedKp[i].x += frame.cols;
            line(matchedImg, curMatchedKp[i], targetMatchedKp[i], CV_RGB(255, 0, 0));
            circle(matchedImg, projectedKp[i], 2, CV_RGB(0, 255, 0));
        }
    }
    if(showImg) {
        imshow("Match Result", matchedImg);
    }

    dist = dist / count;
    if(dist > PROJ_ERR_THRES) {
        Mat failHomo = Mat::ones(3,3,CV_8U);
        failHomo.at<unsigned int>(0,0) = HOMO_FAIL_NORM;
        cout<<"Match fail, proj error exceeded!"<<endl;
        return failHomo;
    }
    return homography;
}

Mat Tracker::pixelMatch(const Mat& matchedFrame, const Mat& featureRes)
{
    Mat matchedEdge, targetEdge, matchedBoarder, targetBoarder;
    Canny( matchedFrame, matchedEdge, board_thres/3, board_thres, 3);
    Canny( targetFrame, targetEdge, board_thres/3, board_thres, 3);

    matchedEdge.convertTo(matchedEdge, CV_32F);
    blur(matchedEdge, matchedEdge, Size(board_size, board_size));
    threshold(matchedEdge, matchedEdge, 0, 1, cv::THRESH_BINARY);
    matchedEdge.convertTo(matchedEdge, CV_8U);
    matchedFrame.copyTo(matchedBoarder, matchedEdge);

    targetEdge.convertTo(targetEdge, CV_32F);
    blur(targetEdge, targetEdge, Size(board_size, board_size));
    threshold(targetEdge, targetEdge, 0, 1, cv::THRESH_BINARY);
    targetEdge.convertTo(targetEdge, CV_8U);
    targetFrame.copyTo(targetBoarder, targetEdge);

    Rect mask(0, 0, matchedFrame.cols, matchedFrame.rows);
    Mat matchedROI(matchedBoarder, mask);
    Mat targetROI(targetBoarder, mask);

    imshow("1", matchedROI);
    imshow("2", targetROI);

    Mat matchedFrame64F, targetFrame64F;
    matchedROI.convertTo(matchedFrame64F, CV_64FC3);
    targetROI.convertTo(targetFrame64F, CV_64FC3);
    Mat homo = pixelWiseMatch(matchedFrame64F, targetFrame64F, featureRes);
    return homo;
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

void Tracker::showDifferenceEdge(const Mat& image1, const Mat& image2, string name)
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

    imshow(name, res);

    return;
}

//return pair<homo, diffImg>
Mat Tracker::pixelWiseMatch(const Mat& img1, const Mat& img2, const Mat& initialHomo)
{
    Ptr<Map> res = new MapProjec(initialHomo);
    MapperGradProj mapper;
    MapperPyramid mappPyr(mapper);
    mappPyr.numIterPerScale_ = 10;
    mappPyr.calculate(img1, img2, res);

    MapProjec* mapProj = dynamic_cast<MapProjec*>(res.get());
    mapProj->normalize();

    // Generate diffImg
    Mat dest;
    mapProj->inverseWarp(img2, dest);
    showDifferenceEdge(img1, dest, "Pixel Difference");

    return Mat(mapProj->getProjTr());
}
