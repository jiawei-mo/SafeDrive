#ifndef TRACKER_HPP
#define TRACKER_HPP

#endif // TRACKER_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
#include <parameters.hpp>
using namespace cv;
using namespace std;

struct TrackRes
{
    Mat matchedImg;
    Mat homo;
    float score;
};

class Tracker
{
protected:
    Ptr<ORB> detector;
    Ptr<DescriptorMatcher> matcher;

    Mat targetFrame, targetDesc;
    vector<KeyPoint> targetKp;

    int MAX_NUM_FEATURE;
    float QUALITY_LEVEL;
    int MIN_DISTANCE;

    float NN_MATCH_THRES;
    unsigned int NN_MATCH_NUMBER;
    float RANSAC_THRES;

public:
    Tracker();
    Tracker(int mnf, float ql, int md, float nmt, int nmn, float rt);
    void changeParam(int mnf, float ql, int md, float nmt, int nmn, float rt);
    void setTarget(const Mat frame);
    TrackRes match(const Mat frame);
};
