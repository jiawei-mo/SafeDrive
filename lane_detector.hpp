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
    vector<Point2f> whitePoints;
    vector<Point2f> yellowPoints;
    LaneRes(Mat l, vector<Point2f> w, vector<Point2f> y) {
        laneImg = l.clone();
        whitePoints = w;
        yellowPoints = y;
    }
    ~LaneRes() {
        cout<<"LaneRes deallocation"<<endl;
    }
};

class LaneDetector
{
public:
    LaneRes* process(const Mat& img);
};
