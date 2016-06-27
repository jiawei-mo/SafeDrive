#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

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
