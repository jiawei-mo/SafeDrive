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
    int max_lane_reproj_dist;
    int max_theta_offset;

    bool getPose(const Mat& left_img, const Mat& right_img, Mat& R, Mat& t, vector<Point2f>& left_kp_inliner, vector<Point2f>& right_kp_inliner);
    void matchRoadMarkers(const Mat& left_rectified, const Mat& right_rectified,
                          const vector<vector<Point2d> >& left_marker_detected_cartesian, const vector<vector<Point2d> >& right_marker_detected_cartesian,
                          vector<vector<Point2d> >& left_marker_cartesian, vector<vector<Point2d> >& right_marker_cartesian);

public:
    ThreeDHandler();
    ThreeDHandler(const shared_ptr<Matcher> _matcher);
    ~ThreeDHandler();
    void setCamK(const vector<float>& _K);
    void changeParam(const shared_ptr<Matcher> matcher, float rte, float rtp, int mlrd, int mto);
    bool find3DPoints(const Mat& left_img, const Mat& right_img, vector<Point2f> &features, Mat& additional_desc, vector<Point3f> &feature_pts, vector<vector<Point3f> > &marker_pts);
    bool project(const Mat& obj_img, const Mat &cur_img, const vector<Point2f> &features, const Mat& additional_desc, const vector<Point3f>& feature_pts, const vector<vector<Point3f> > &marker_pts, Mat &output);
};

#endif // THREEDHANDLER_H
