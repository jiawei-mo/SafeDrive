#ifndef TRACKER_HPP
#define TRACKER_HPP

#endif // TRACKER_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"

#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapperpyramid.hpp"

using namespace cv;
using namespace std;
using namespace cv::reg;

#define HOMO_FAIL_NORM 20000.0f
#define PROJ_ERR_THRES 200.0f
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

    vector<Point2f> inline_target;
    vector<Point2f> inline_matched;

    int max_num_features;
    float quality_level;
    int min_distance;
    int blur_size;
    float blur_var;
    float nn_match_thres;
    float ransac_thres;

    int board_thres;
    int board_size;

public:
    Tracker();
    Tracker(int mnf, float ql, int md, int bs, float bv, float nmt, float rt, int bds);
    void changeParam(int mnf, float ql, int md, int bs, float bv, float nmt, float rt, int bds);
    void setTarget(const Mat& frame);
    int featureMatch(const Mat& frame, Mat& homography, bool showImg);
    Mat pixelMatch(const Mat& recMatchedFrame);

    //helpers
    void showDifference(const Mat& image1, const Mat& image2, string title);
    void showDifferenceEdge(const Mat& image1, const Mat& image2, string name);
    Mat pixelWiseMatch(const Mat& img1, const Mat& img2);
};
