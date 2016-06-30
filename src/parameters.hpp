#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define MAX_NUM_FEATURE 1000
#define QUALITY_LEVEL 0.01
#define MIN_DISTANCE 7

#define NN_MATCH_THRES 0.9f
#define NN_MATCH_NUMBER 4
#define RANSAC_THRES 20.0f

#define HOMO_NORM_THRES 30.0f
#define HOMO_FAIL_SCORE 100.0f
