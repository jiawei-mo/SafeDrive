#ifndef MATCHER_HPP
#define MATCHER_HPP

#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "headers/parameters.hpp"

using namespace cv;
using namespace std;

class Matcher
{
private:
    Ptr<ORB> detector;
    Ptr<DescriptorMatcher> matcher;

    Mat mask;

    int max_num_features;
    float quality_level;
    int min_distance;
    float match_thres_feature;
    float match_thres_given_kp;

public:
    Matcher();
    void changeParam(int mnf, float ql, int md, float mtf, float mtg);
    void match_desc(const Mat& left_desc, const Mat& right_desc, vector<pair<int, int> >& matches, float thres, bool bi_direct=true);
    void match(const Mat& left_img, vector<Point2f> &left_matched_kp, const Mat &right_img, vector<Point2f>& right_matched_kp);
    void match_given_kp(const Mat& template_img, const vector<Point2f> &template_kp, const Mat& match_img, vector<Point2f>& matched_kp, vector<int> &inliner);
    void match_given_desc(const Mat& template_desc, const Mat& match_img, vector<Point2f>& matched_kp, vector<int>& inliner);
    void rectified_match(const Mat& left_img, vector<Point2f> &left_matched_kp, const Mat &right_img, vector<Point2f>& right_matched_kp);
    size_t matchCounter(const Mat& left_img, const Mat& right_img);
    void showMatches(const Mat& left_img, const vector<Point2f> &left_p, const Mat& right_img, const vector<Point2f> &right_p, const string &windowName);
    void denseMatchAndGeneratePCL(Mat &left_img, const Mat& right_img, const Mat& Q);

    //helpers
    void showDifference(const Mat& image1, const Mat& image2, string title);
    void showDifferenceEdge(const Mat& image1, const Mat& image2, string name);

    template <class T>
    bool get_desc(const Mat& img, const vector<T>& kp, Mat& desc)
    {
        if(img.empty()) {
            return false;
        }

        vector<KeyPoint> _kp;
        for( size_t i = 0; i < kp.size(); i++ ) {
            _kp.push_back(KeyPoint(kp[i], 1.f));
        }

        detector->compute(img, _kp, desc);
        return true;
    }
};

#endif // MATCHER_HPP
