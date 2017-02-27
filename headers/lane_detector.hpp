#ifndef LANE_DETECTOR_HPP
#define LANE_DETECTOR_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;

class LaneDetector
{
public:
    void detect(const Mat& img, vector<Point2f> &markerPoints);
    void detectAndShow(const Mat& detImg, Mat &projImg);
};

#endif // LANE_DETECTOR_HPP
