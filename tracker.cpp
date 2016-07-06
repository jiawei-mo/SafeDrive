#include "tracker.hpp"

Tracker::Tracker()
{
    MAX_NUM_FEATURE = 1000;
    QUALITY_LEVEL = 0.01f;
    MIN_DISTANCE = 7;
    NN_MATCH_THRES = 0.9f;
    NN_MATCH_NUMBER = 4;
    RANSAC_THRES = 20.0f;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

Tracker::Tracker(int mnf, float ql, int md, float nmt, int nmn, float rt)
{
    MAX_NUM_FEATURE = mnf;
    QUALITY_LEVEL = ql;
    MIN_DISTANCE = md;
    NN_MATCH_THRES = nmt;
    NN_MATCH_NUMBER = nmn;
    RANSAC_THRES = rt;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::changeParam(int mnf, float ql, int md, float nmt, int nmn, float rt)
{
    MAX_NUM_FEATURE = mnf;
    QUALITY_LEVEL = ql;
    MIN_DISTANCE = md;
    NN_MATCH_THRES = nmt;
    NN_MATCH_NUMBER = nmn;
    RANSAC_THRES = rt;
    targetKp.clear();
}

void Tracker::setTarget(const Mat frame)
{
    targetFrame = frame.clone();
    GaussianBlur(targetFrame, targetFrame, Size(5,5), 1.2, 1.2);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(targetFrame, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, MAX_NUM_FEATURE, QUALITY_LEVEL, MIN_DISTANCE);

    for( size_t i = 0; i < corners.size(); i++ ) {
        targetKp.push_back(KeyPoint(corners[i], 1.f));
    }

    detector->compute(targetFrame, targetKp, targetDesc);
}

TrackRes Tracker::match(const Mat frame)
{
    Mat curFrame = frame.clone();
    GaussianBlur(curFrame, curFrame, Size(5,5), 1.2, 1.2);

    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(curFrame, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, MAX_NUM_FEATURE, QUALITY_LEVEL, MIN_DISTANCE);

    vector<KeyPoint> curKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        curKp.push_back(KeyPoint(corners[i], 1.f));
    }

    Mat curDesc;
    detector->compute(curFrame, curKp, curDesc);

    vector< vector<DMatch> > matches;
    matcher->knnMatch(curDesc, targetDesc, matches, 2);
    vector<Point2f> targetMatchedKp, curMatchedKp;
    for(int i=0; i<(int)matches.size(); i++)
    {
        circle(curFrame, curKp[matches[i][0].trainIdx].pt, 2, CV_RGB(0, 0, 255));
        if(matches[i][0].distance < NN_MATCH_THRES*matches[i][1].distance)
        {
            curMatchedKp.push_back(curKp[matches[i][0].queryIdx].pt);
            targetMatchedKp.push_back(targetKp[matches[i][0].trainIdx].pt);
        }
    }
    Mat homography, inliner_mask;
    if(targetMatchedKp.size() >= NN_MATCH_NUMBER)
    {
        homography = findHomography(curMatchedKp, targetMatchedKp, RANSAC, RANSAC_THRES, inliner_mask);
    }

    Mat matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
    curFrame.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
    targetFrame.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));

    if(targetMatchedKp.size() < NN_MATCH_NUMBER || homography.empty() || norm(homography)>HOMO_NORM_THRES)
    {
        for(int i=0; i<(int)targetMatchedKp.size(); i++)
        {
            targetMatchedKp[i].x += frame.cols;
            line(matchedImg, curMatchedKp[i], targetMatchedKp[i], CV_RGB(255, 0, 0));
        }
        Mat failHomo;
        return TrackRes{matchedImg, failHomo, HOMO_FAIL_SCORE+1};
    }

    vector<Point2f> projectedKp;
    perspectiveTransform(curMatchedKp, projectedKp, homography);

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
    dist = dist / count;
    return TrackRes{matchedImg, homography, dist/count};
}
