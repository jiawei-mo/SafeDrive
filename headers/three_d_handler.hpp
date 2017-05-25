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
    Mat K;
    Mat camera_coeff;

    float ransac_thres_essential;
    float ransac_thres_pnp;

public:
    ThreeDHandler();
    ThreeDHandler(const shared_ptr<Matcher> _matcher);
    ~ThreeDHandler();
    void changeParam(const shared_ptr<Matcher> matcher, float rte, float rtp);
    void find3DPoints(const Mat& left_img, const Mat& right_img, vector<Point2f> &features, vector<Point3f> &feature_pts, vector<Point3f> &marker_pts, vector<Vec3b> &marker_color);
    bool project(const Mat& obj_img, Mat &cur_img, const vector<Point2f> &features, const vector<Point3f>& feature_pts, const vector<Point3f>& marker_pts, const vector<Vec3b> &marker_color);
};

#endif // THREEDHANDLER_H
