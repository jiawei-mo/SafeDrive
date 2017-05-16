#ifndef THREEDHANDLER_H
#define THREEDHANDLER_H

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "headers/lane_detector.hpp"
#include "headers/matcher.hpp"
#include "headers/parameters.hpp"

using namespace cv;
using namespace std;
class ThreeDHandler
{
private:
    shared_ptr<Matcher> matcher;
    shared_ptr<LaneDetector> lane_detector;
    Mat camera_K;
    Mat camera_coeff;

    float ransac_thres_essential;
    float ransac_thres_pnp;

public:
    ThreeDHandler();
    ThreeDHandler(const shared_ptr<Matcher> _matcher);
    ~ThreeDHandler();
    void changeParam(const shared_ptr<Matcher> matcher, float rte, float rtp);
    void findDisparity(vector<KeyPoint> &feature_disp, Mat &marker_disp, Mat &Q, Mat &left_img, Mat &right_img);
    void project(const Mat& obj_img, Mat& cur_img, vector<KeyPoint> &feature_disp, const Mat& marker_disp, const Mat &Q);
};

#endif // THREEDHANDLER_H
