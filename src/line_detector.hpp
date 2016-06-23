#include "opencv2/opencv.hpp"
#include <opencv2/features2d.hpp>
#include <dirent.h>
#include <string.h>
using namespace cv;
using namespace std;

#define NN_MATCH_RATIO 0.5
#define NUM_FEATURES 5000

class Line_detector
{
public:
	Line_detector(){};
	Mat process(const Mat img);
};
