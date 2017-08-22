#ifndef LANE_DETECTOR_HPP
#define LANE_DETECTOR_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
#include "headers/parameters.hpp"

using namespace cv;
using namespace std;

class LaneDetector
{
public:
    void detect(const Mat& img, vector<Mat> &white_yellow_mask);
    void houghDetect(const Mat& img, Mat& mask);
};

#endif // LANE_DETECTOR_HPP
