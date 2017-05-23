#ifndef THREEDHANDLER_H
#define THREEDHANDLER_H

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "headers/polarcalibration.h"
#include "headers/lane_detector.hpp"
#include "headers/matcher.hpp"
#include "headers/parameters.hpp"

using namespace cv;
using namespace std;
class ThreeDHandler
{
private:
    shared_ptr<PolarCalibration> calibrator;
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
    void findCorrespondence(const Mat &left_img, const Mat &right_img, vector<pair<Point2f, Point2f> > &marker_corres);
    void project(const Mat& left_img, const Mat &right_img, Mat &cur_img, const vector<pair<Point2f, Point2f> >& marker_corres);
};

#endif // THREEDHANDLER_H
