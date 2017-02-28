#ifndef THREEDHANDLER_H
#define THREEDHANDLER_H

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include "headers/parameters.hpp"

using namespace cv;
using namespace std;
class ThreeDHandler
{
private:

    Mat camera_K;
    Mat camera_coeff;

    float ransac_thres_feature;
    int SADWindowSize;
    int numberOfDisparities;
    int preFilterCap;
    int minDisparity;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    int SP1;
    int SP2;

    boost::thread showPoints;

public:
    ThreeDHandler();
    ~ThreeDHandler();
    void changeParam(float rtf, int sws, int nd, int pfc, int mod, int ur, int sw, int sr, int dmd, int s1, int s2);
    void findDisparity(Mat &disp_img, Mat &Q, Mat &left_img, vector<Point2f> &left_kp, Mat &right_img, vector<Point2f> &right_kp);
    void project(Mat& cur_img, const vector<Point2f>& p_cur, const Mat& disp_img, const Mat& obj_img, const vector<Point2f>& p_obj, const Mat& Q, const vector<Point2f>& p_marker);
};

#endif // THREEDHANDLER_H
