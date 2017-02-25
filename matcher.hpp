#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "parameters.hpp"

using namespace cv;
using namespace std;

struct matchComp {
    bool operator()(DMatch a, DMatch b) {
        return a.distance < b.distance;
    }
};

class Matcher
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

public:
    Matcher();
    void changeParam(int bs, float bv, int mnf, float ql, int md,  int ngf, float mtf);
    void setTarget(const Mat& frame);
    void match(const Mat& frame, vector<Point2f>& targetMatchedKp, vector<Point2f>& dbMatchedKp, bool showImg=false, string windowName="No Window", int num_grid=-1, float match_thres=-1);

    //helpers
    void showDifference(const Mat& image1, const Mat& image2, string title);
    void showDifferenceEdge(const Mat& image1, const Mat& image2, string name);
};

#endif // MATCHER_HPP
