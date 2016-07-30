#ifndef TRACKER_HPP
#define TRACKER_HPP

#endif // TRACKER_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;

#define HOMO_FAIL_NORM 20000.0f
#define PROJ_ERR_THRES 100.0f
#define NN_MATCH_NUMBER 4

class Tracker
{
protected:
    Ptr<ORB> detector;
    Ptr<DescriptorMatcher> matcher;

    Mat mask;
    Mat targetFrame;
    Mat targetDesc;
    vector<KeyPoint> targetKp;

    int max_num_features;
    float quality_level;
    int min_distance;
    int blur_size;
    float blur_var;
    float nn_match_thres;
    float ransac_thres;

public:
    Tracker();
    Tracker(int mnf, float ql, int md, int bs, float bv, float nmt, float rt);
    void changeParam(int mnf, float ql, int md, int bs, float bv, float nmt, float rt);
    void setTarget(const Mat& frame);
    Mat match(const Mat& frame);
};
