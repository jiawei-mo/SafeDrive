#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "headers/parameters.hpp"

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

    int max_num_features;
    float quality_level;
    int min_distance;
    float match_thres_feature;

public:
    Matcher();
    void changeParam(int mnf, float ql, int md, float mtf);
    void match(const Mat& left_img, vector<Point2f> &left_matched_kp, const Mat &right_img, vector<Point2f>& right_matched_kp);
    void rectified_match(const Mat& left_img, vector<Point2f> &left_matched_kp, const Mat &right_img, vector<Point2f>& right_matched_kp);
    size_t matchCounter(const Mat& left_img, const Mat& right_img);
    void showMatches(const Mat& left_img, const vector<Point2f> &left_p, const Mat& right_img, const vector<Point2f> &right_p, const string &windowName, const Mat& inliers=Mat());

    //helpers
    void showDifference(const Mat& image1, const Mat& image2, string title);
    void showDifferenceEdge(const Mat& image1, const Mat& image2, string name);
};

#endif // MATCHER_HPP
