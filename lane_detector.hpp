#ifndef LANE_DETECTOR_HPP
#define LANE_DETECTOR_HPP

#endif // LANE_DETECTOR_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;

struct LaneRes
{
    Mat laneImg;
    vector<Point2f> dots;
};

class LaneDetector
{
public:
    LaneRes process(const Mat img);
};
