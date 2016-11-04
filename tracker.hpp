#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"

#include "ecc.hpp"

#include "parameters.hpp"

using namespace cv;
using namespace std;

struct matchComp {
    bool operator()(DMatch a, DMatch b) {
        return a.distance < b.distance;
    }
};

class Tracker
{
private:
    int checkArea(Point2f p, int n_c, int n_r, int num_grid);
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
    int num_grid_feature;
    float match_thres_feature;
    float ransac_thres_feature;
    int board_size;
    int pp_grid;
    int num_grid_pixel;
    float match_thres_pixel;
    float ransac_thres_pixel;

public:
    Tracker();
    void changeParam(int bs, float bv, int mnf, float ql, int md,  int ngf, float mtf, float rtf, int bds, int pg, int ngp, float mtp, float rtp);
    void setTarget(const Mat& frame);
    int featureMatch(const Mat& frame, Mat& homography, vector<Point2f> *inline_matched, bool showImg=false, string windowName="No Window", int num_grid=-1, float match_thres=-1, float ransac_thres=-1, const Mat &showROI=Mat::ones(1,1,CV_8U));
    Mat pixelMatch(const Mat& recMatchedFrame);

    //helpers
    void showDifference(const Mat& image1, const Mat& image2, string title);
    void showDifferenceEdge(const Mat& image1, const Mat& image2, string name);
    Mat pixelWiseMatch(const Mat& img1, const Mat& img2, const Mat &roi, const Mat &initialH);
};

#endif // TRACKER_HPP
