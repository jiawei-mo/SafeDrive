#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;


class Line_detector
{
public:
	Line_detector(){};
	Mat process(const Mat img);
};
