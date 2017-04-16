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
#include "headers/lane_detector.hpp"
#include "headers/matcher.hpp"

using namespace cv;
using namespace std;

struct Point2fHasher
{
  std::size_t operator()(const Point2f& p) const
  {
    using std::size_t;
    using std::hash;
    using std::string;

    return (hash<float>()(p.x)
             ^ ((hash<float>()(p.y) << 1)) >> 1);
  }
};

class ThreeDHandler
{
private:
    shared_ptr<Matcher> matcher;
    shared_ptr<LaneDetector> lane_detector;
    Mat camera_K;
    Mat camera_coeff;

    vector<Point2f> left_kp;
    vector<Point2f> right_kp;
    unordered_map<Point2f, Point3f, Point2fHasher> three_d_pts_map;
    vector<Point3f> lane_pts;
    Mat left_img;
    Mat right_img;

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
    void find3DPoints(const Mat& _left_img, const Mat& _right_img);
    void project(Mat& target_img);
};

#endif // THREEDHANDLER_H
