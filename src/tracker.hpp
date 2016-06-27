#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

#define NN_MATCH_THRES 50.0f
#define NN_MATCH_NUMBER 20
#define RANSAC_THRES 2.5f
#define HOMO_NORM_THRES 50.0f

class Tracker
{
protected:
	Ptr<ORB> detector;
	Ptr<DescriptorMatcher> matcher;

	Mat targetFrame, targetDesc;
	vector<KeyPoint> targetKp;

public:
	Tracker();
	void setTarget(const Mat frame);
	Mat match(const Mat frame);
};
