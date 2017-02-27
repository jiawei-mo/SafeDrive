#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include "parameters.hpp"

using namespace cv;
using namespace std;
class Reconstructor
{
private:
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

public:
    Reconstructor();
    void changeParam(float rtf, int sws, int nd, int pfc, int mod, int ur, int sw, int sr, int dmd, int s1, int s2);
    void reconstruct(const Mat& left_img, const vector<Point2f> &left_kp, const Mat& right_img, const vector<Point2f> &right_kp, const Mat& cam_K, const Mat& cam_coeff);
};

#endif // RECONSTRUCTOR_H
