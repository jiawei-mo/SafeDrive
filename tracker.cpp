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
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

Tracker::Tracker(int mnf, float ql, int md, int bs, float bv, float nmt, float rt)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    blur_size = bs;
    blur_var = bv;
    nn_match_thres = nmt;
    ransac_thres = rt;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::changeParam(int mnf, float ql, int md, int bs, float bv, float nmt, float rt)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    blur_size = bs;
    blur_var = bv;
    nn_match_thres = nmt;
    ransac_thres = rt;
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
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance, mask);

    for( size_t i = 0; i < corners.size(); i++ ) {
        targetKp.push_back(KeyPoint(corners[i], 1.f));
    }
    detector->compute(targetFrame, targetKp, targetDesc);

    for( size_t i = 0; i < corners.size(); i++ ) {
        circle(targetFrame, corners[i], 2, CV_RGB(0, 0, 255));
    }
}

Mat Tracker::match(const Mat& frame)
{
    //detect feature points on curFrame
    Mat curFrame = frame.clone();
    GaussianBlur(curFrame, curFrame, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(curFrame, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance, mask);

    vector<KeyPoint> curKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        curKp.push_back(KeyPoint(corners[i], 1.f));
    }

    Mat curDesc;
    detector->compute(curFrame, curKp, curDesc);

    //matches
    vector< vector<DMatch> > matches;
    matcher->knnMatch(curDesc, targetDesc, matches, 2);
    vector<Point2f> targetMatchedKp, curMatchedKp;
    for(int i=0; i<(int)matches.size(); i++)
    {
        circle(curFrame, curKp[matches[i][0].trainIdx].pt, 2, CV_RGB(0, 0, 255));
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

    //show inliner pairs between two images side by side
    Mat matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
    curFrame.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
    targetFrame.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));

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
    imshow("Match Result", matchedImg);

    dist = dist / count;
    cout<<"Proj Err: "<<dist<<endl;
    if(dist > PROJ_ERR_THRES) {
        Mat failHomo = Mat::ones(3,3,CV_8U);
        failHomo.at<unsigned int>(0,0) = HOMO_FAIL_NORM;
        cout<<"Match fail!"<<endl;
        return failHomo;
    }
    return homography;
}
